
package freechips.rocketchip.tile

import Chisel._

import chisel3.{VecInit}
import chisel3.SyncReadMem
import chisel3.experimental._
import freechips.rocketchip.config._
import freechips.rocketchip.subsystem._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.rocket._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._
import NetworkHelpers._
import LNICConsts._

// the first word of every msg sent by application
class TxAppHdr extends Bundle {
  val dst_ip = UInt(32.W)
  val dst_context = UInt(LNIC_CONTEXT_BITS.W)
  val msg_len = UInt(MSG_LEN_BITS.W)
}

/**
 * LNIC Packetization Module
 *
 * Tasks:
 *   - Reverse byte order of words coming from CPU
 *   - Consume message words from the CPU and transform into pkts
 *   - Store msgs and transmit pkts, also support retransmitting pkts when told to do so
 */

class PacketizeIO extends Bundle {
  val net_in = Flipped(Decoupled(new MsgWord))
  val net_out = Decoupled(new StreamChannel(NET_DP_BITS))
  val meta_out = Valid(new PISAEgressMetaIn)
}

/* Descriptor that is used to schedule TX pkts */
class TxPktDescriptor extends Bundle {
  val tx_msg_id   = UInt(LNIC_MSG_ID_BITS.W)
  val tx_pkts     = UInt(MAX_PKTS_PER_MSG.W)
  val buf_ptr     = UInt(BUF_PTR_BITS.W)
  val size_class  = UInt(SIZE_CLASS_BITS.W)
  val tx_app_hdr  = new TxAppHdr()
  val src_context = UInt(LNIC_CONTEXT_BITS.W)
}

/* State maintained per-context on enqueue */
class ContextEnqState extends Bundle {
  val tx_app_hdr = new TxAppHdr()
  val tx_msg_id  = UInt(LNIC_MSG_ID_BITS.W)
  val size_class = UInt(SIZE_CLASS_BITS.W)
  val buf_ptr    = UInt(BUF_PTR_BITS.W)
  val pkt_offset = UInt(PKT_OFFSET_BITS.W)
  val rem_bytes  = UInt(MSG_LEN_BITS.W)
  val pkt_bytes  = UInt(16.W)
  val word_count = UInt(16.W)
}

@chiselName
class LNICPacketize(implicit p: Parameters) extends Module {
  val num_contexts = p(LNICKey).maxNumContexts

  val io = IO(new PacketizeIO)

  /* Memories (i.e. tables) and Queues */
  // freelist to keep track of available tx_msg_ids
  val tx_msg_ids = for (id <- 0 until NUM_MSG_BUFFERS) yield id.U
  val tx_msg_id_freelist = Module(new FreeList(tx_msg_ids))
  // RAM used to store msgs while they are being reassembled and delivered to the CPU.
  //   Msgs are stored in words that are the same size as the datapath width.
  val msg_buffer_ram = SyncReadMem(NUM_MSG_BUFFER_WORDS, Vec(NET_DP_BITS/XLEN, UInt(XLEN.W)))

  // Vector of Regs containing the buffer size of each size class
  val size_class_buf_sizes = RegInit(VecInit(MSG_BUFFER_COUNT.map({ case (size: Int, count: Int) => size.U(MSG_LEN_BITS.W) }).toSeq))
  // Vector of freelists to keep track of available buffers to store msgs.
  //   There is one free list for each size class.
  val size_class_freelists_io = MsgBufHelpers.make_size_class_freelists() 

  // FIFO queue to schedule delivery of TX pkts
  // TODO(sibanez): this should become a PIFO ideally
  val scheduled_pkts_enq = Wire(Decoupled(new TxPktDescriptor))
  val scheduled_pkts_deq = Wire(Flipped(Decoupled(new TxPktDescriptor)))
  scheduled_pkts_deq <> Queue(scheduled_pkts_enq, SCHEDULED_PKTS_Q_DEPTH)

  /**
   * Msg Enqueue State Machine.
   * Tasks:
   *   - Wait for MsgWord from the TxQueue
   *   - When a MsgWord arrives, record msg_len and keep track of remaining_bytes
   *   - One the first word of a msg, allocate a buffer and tx_msg_id
   *   - For subsequent words of the msg, write into the appropriate buffer location
   *   - When either (1) an MTU has been accumulated, or (2) the full msg has arrived,
   *     schedule a tx pkt descriptor for transmission
   *
   * NOTE: All state variables are per-context
   *
   * Description of states:
   * sWaitAppHdr:
   *   - Wait for TxAppHdr to arrive (the first MsgWord of a msg)
   *   - If there is no buffer available to store this msg => assert backpressure
   *   - Otherwise, allocate a buffer and tx_msg_id and store in per-context state
   *
   * sWriteMsg:
   *   - Wait for a MsgWord to arrive
   *   - Write the MsgWord into the correct buffer location (using a masked write)
   *   - When either (1) an MTU has been accumulated, or (2) the full msg has arrived,
   *     schedule a tx pkt descriptor for transmission
   */
  val sWaitAppHdr :: sWriteMsg :: Nil = Enum(2)
  // need one state per context
  val enqStates = RegInit(VecInit(Seq.fill(num_contexts)(sWaitAppHdr)))

  // defaults
  val enq_context = io.net_in.bits.src_context
  io.net_in.ready := true.B

  val tx_app_hdr = Wire((new TxAppHdr).fromBits(io.net_in.bits.data))

  // bitmap of valid signals for all size classes
  val free_classes = size_class_freelists_io.map(_.deq.valid)
  // bitmap of size_classes that are large enough to store the whole msg
  val candidate_classes = size_class_buf_sizes.map(_ >= tx_app_hdr.msg_len)
  // bitmap indicates classes with available buffers that are large enough
  val available_classes = free_classes.asUInt & candidate_classes.asUInt

  // Per-context enq state
  val context_enq_state = RegInit(VecInit(Seq.fill(num_contexts)((new ContextEnqState).fromBits(0.U))))

  // msg buffer write port
  val enq_buf_ptr = Wire(UInt(BUF_PTR_BITS.W))
  val enq_msg_buf_ram_port = msg_buffer_ram(enq_buf_ptr)

  switch (enqStates(enq_context)) {
    is (sWaitAppHdr) {
      // assert back pressure to the CPU if there are no available buffers for this msg
      io.net_in.ready := (available_classes > 0.U)
      when (io.net_in.valid && io.net_in.ready) {
        assert (tx_msg_id_freelist.io.deq.valid, "There is an available buffer but not an available tx_msg_id?")
        // read tx_msg_id_freelist
        tx_msg_id_freelist.io.deq.ready := true.B
        val tx_msg_id = tx_msg_id_freelist.io.deq.bits
        // read from target size class freelist
        val target_size_class = PriorityEncoder(available_classes)
        val target_freelist = size_class_freelists_io(target_size_class)
        target_freelist.deq.ready := true.B
        val buf_ptr = target_freelist.deq.bits
        // record per-context enqueue state
        val ctx_state = context_enq_state(enq_context)
        ctx_state.tx_app_hdr := tx_app_hdr
        ctx_state.tx_msg_id := tx_msg_id
        ctx_state.size_class := target_size_class
        ctx_state.buf_ptr := buf_ptr
        ctx_state.pkt_offset := 0.U
        ctx_state.rem_bytes := tx_app_hdr.msg_len
        ctx_state.pkt_bytes := 0.U
        ctx_state.word_count := 0.U // counts the number of words written by CPU for this msg (not including app hdr)
        // TODO(sibanez): initialize state that is indexed by tx_msg_id (for transport support)
        // state transition
        enqStates(enq_context) := sWriteMsg
      }
    }
    is (sWriteMsg) {
      // wait for a MsgWord to arrive
      when (io.net_in.valid) {
        val ctx_state = context_enq_state(enq_context)
        // compute where to write MsgWord in the buffer
        val word_offset_bits = log2Up(NET_DP_BITS/XLEN)
        val word_offset = ctx_state.word_count(word_offset_bits-1, 0)
        val word_ptr = ctx_state.word_count(15, word_offset_bits)
        enq_buf_ptr := ctx_state.buf_ptr + word_ptr
        // TODO(sibanez): check that this properly implements a sub-word write
        enq_msg_buf_ram_port(word_offset) := reverse_bytes(io.net_in.bits.data, XBYTES)
        // build tx pkt descriptor just in case we need to schedule a pkt
        val tx_pkt_desc = Wire(new TxPktDescriptor)
        tx_pkt_desc.tx_msg_id := ctx_state.tx_msg_id
        tx_pkt_desc.tx_pkts := 1.U << ctx_state.pkt_offset
        tx_pkt_desc.buf_ptr := ctx_state.buf_ptr
        tx_pkt_desc.size_class := ctx_state.size_class
        tx_pkt_desc.tx_app_hdr := ctx_state.tx_app_hdr
        tx_pkt_desc.src_context := enq_context

        val is_last_word = ctx_state.rem_bytes <= XBYTES.U
        val is_full_pkt = ctx_state.pkt_bytes + XBYTES.U === MAX_PKT_LEN_BYTES.U

        when (is_last_word || is_full_pkt) {
          // schedule pkt for tx
          // TODO(sibanez): this isn't really a bug, this can legit happen, how best to deal with it?
          assert (scheduled_pkts_enq.ready, "scheduled_pkts queue is full during enqueue!")
          scheduled_pkts_enq.valid := true.B
          scheduled_pkts_enq.bits := tx_pkt_desc
        }

        when (is_full_pkt) {
          // reset/increment pkt counter state
          ctx_state.pkt_offset := ctx_state.pkt_offset + 1.U
          ctx_state.pkt_bytes := 0.U         
        }

        when (is_last_word) {
          // state transition
          enqStates(enq_context) := sWaitAppHdr
        }

        // update context state variables
        ctx_state.rem_bytes := ctx_state.rem_bytes - XBYTES.U
        ctx_state.word_count := ctx_state.word_count + 1.U
      }
    }
  }

  /**
   * Pkt Dequeue State Machine:
   * Tasks:
   *   - Wait for a pkt to be scheduled
   *   - Record the tx pkt descriptor
   *   - Transmit all pkts indicated by the descriptor
   *   - Repeat
   *
   * Description of states:
   * sWaitTxPkts:
   *   - Wait for a TxPktDescriptor to be scheduled
   *   - Start reading the first word of the pkt from the msg buffer
   *   - Record an updated descriptor indicating the next pkt to transmit after the first one
   *   - Record the number of bytes remaining for the current pkt
   *   - Transition to the sSendTxPkts state
   *
   * sSendTxPkts:
   *   - Transmit all bytes of the current pkt
   *   - Update descriptor when done sending a pkt
   *   - Transition back to sWaitTxPkts when done sending all pkts
   */
  val sWaitTxPkts :: sSendTxPkts :: Nil = Enum(2)
  val deqState = RegInit(sWaitTxPkts)

  // msg buffer read port
  val deq_buf_ptr = Wire(UInt(BUF_PTR_BITS.W))
  val deq_msg_buf_ram_port = msg_buffer_ram(deq_buf_ptr)
  val deq_buf_ptr_reg = RegInit(0.U(BUF_PTR_BITS.W))

  // register to store descriptor of pkt(s) currently being transmitted
  val active_tx_desc_reg = Reg(new TxPktDescriptor)
  // register to track the number of bytes remaining to send for the current pkt
  val deq_pkt_rem_bytes_reg = RegInit(0.U(16.W))
  // register to track current pkt offset
  val deq_pkt_offset_reg = RegInit(0.U(PKT_OFFSET_BITS.W))
  // register to track first word of each pkt (used to drive meta_out.valid)
  val is_first_word_reg = RegInit(false.B)

  // defaults
  io.net_out.valid     := false.B
  io.net_out.bits.data := deq_msg_buf_ram_port.asUInt
  io.net_out.bits.keep := NET_DP_FULL_KEEP
  io.net_out.bits.last := false.B

  io.meta_out.valid               := is_first_word_reg
  io.meta_out.bits.dst_ip         := active_tx_desc_reg.tx_app_hdr.dst_ip
  io.meta_out.bits.dst_context    := active_tx_desc_reg.tx_app_hdr.dst_context
  io.meta_out.bits.msg_len        := active_tx_desc_reg.tx_app_hdr.msg_len
  io.meta_out.bits.pkt_offset     := deq_pkt_offset_reg
  io.meta_out.bits.src_context    := active_tx_desc_reg.src_context
  io.meta_out.bits.tx_msg_id      := active_tx_desc_reg.tx_msg_id
  io.meta_out.bits.buf_ptr        := active_tx_desc_reg.buf_ptr
  io.meta_out.bits.buf_size_class := active_tx_desc_reg.size_class

  switch (deqState) {
    is (sWaitTxPkts) {
      // wait for a TxPktDescriptor to be scheduled
      when (scheduled_pkts_deq.valid) {
        // read descriptor
        val descriptor = scheduled_pkts_deq.bits
        scheduled_pkts_deq.ready := true.B
        tx_next_pkt(descriptor)
        // state transition
        deqState := sSendTxPkts
      }
    }
    is (sSendTxPkts) {
      io.net_out.valid := true.B
      val is_last_word = deq_pkt_rem_bytes_reg <= NET_DP_BYTES.U
      io.net_out.bits.keep := Mux(is_last_word,
                                  (1.U << deq_pkt_rem_bytes_reg) - 1.U,
                                  NET_DP_FULL_KEEP)
      io.net_out.bits.last := is_last_word

      // wait for no backpressure
      when (io.net_out.ready) {
        when (is_last_word) {
          when (active_tx_desc_reg.tx_pkts === 0.U) {
            // no more pkts to transmit
            deqState := sWaitTxPkts
          } .otherwise {
            // there are more pkts to transmit
            tx_next_pkt(active_tx_desc_reg)
          }
        } .otherwise {
          // start reading the next word
          deq_buf_ptr := deq_buf_ptr_reg + 1.U
          deq_buf_ptr_reg := deq_buf_ptr
          // update deq_pkt_rem_bytes_reg
          deq_pkt_rem_bytes_reg := deq_pkt_rem_bytes_reg - NET_DP_BYTES.U
          // no longer the first word
          is_first_word_reg := false.B
        }
      }
    }
  }

  def tx_next_pkt(descriptor: TxPktDescriptor) = {
    // find the next pkt to transmit
    val pkt_offset = PriorityEncoder(descriptor.tx_pkts)
    deq_pkt_offset_reg := pkt_offset
    // find the word offset from the buf_ptr: pkt_offset*words_per_mtu
    require(isPow2(MAX_PKT_LEN_BYTES), "MAX_PKT_LEN_BYTES must be a power of 2!")
    val word_offset = pkt_offset << (log2Up(MAX_PKT_LEN_BYTES/NET_DP_BYTES)).U
    // start reading the first word of the first pkt
    deq_buf_ptr := descriptor.buf_ptr + word_offset
    deq_buf_ptr_reg := deq_buf_ptr
    // register to track first word of each pkt
    is_first_word_reg := true.B
    // record the updated descriptor
    val new_descriptor = Wire(new TxPktDescriptor)
    new_descriptor := descriptor
    new_descriptor.tx_pkts := descriptor.tx_pkts ^ (1.U << pkt_offset)
    active_tx_desc_reg := new_descriptor
    // record bytes remaining for the current pkt
    val num_pkts = MsgBufHelpers.compute_num_pkts(descriptor.tx_app_hdr.msg_len)
    when (pkt_offset === num_pkts - 1.U) {
        // this is the last pkt of the msg
        // compute the number of bytes in the last pkt of the msg
        val msg_len_mod_mtu = descriptor.tx_app_hdr.msg_len(log2Up(MAX_PKT_LEN_BYTES)-1, 0) 
        val final_pkt_bytes = Mux(msg_len_mod_mtu === 0.U,
                                  MAX_PKT_LEN_BYTES.U,
                                  msg_len_mod_mtu)
        deq_pkt_rem_bytes_reg := final_pkt_bytes
    } .otherwise {
        // this is not the last pkt of the msg
        deq_pkt_rem_bytes_reg := MAX_PKT_LEN_BYTES.U
    }
  }

}

