
package freechips.rocketchip.tile

import Chisel._

import chisel3.{VecInit}
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
  val msg_len = UInt(16.W)
}

/**
 * LNIC Pktization Module
 *
 * Tasks:
 *   - Reverse byte order of words coming from CPU
 *   - Consume message words from the CPU and transform into pkts
 *   - Store msgs and transmit pkts, also support retransmitting pkts when told to do so
 */

class PktizeIO extends Bundle {
  val net_in = Flipped(Decoupled(new MsgWord))
  val net_out = Decoupled(new StreamChannel(NET_DP_WIDTH))
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
class LNICPktize(implicit p: Parameters) extends Module {
  val num_contexts = p(LNICKey).maxNumContexts

  val io = IO(new PktizeIO)

  /* TODO(sibanez): consolidate with Msg Assembly module */

  // parameter computations
  val num_msg_buffer_words = LNICConsts.MSG_BUFFER_COUNT.map( (size: Int, count: Int) => (size/LNICConsts.NET_DP_BYTES)*count ).reduce(_ + _)
  val num_msg_buffers = LNICConsts.MSG_BUFFER_COUNT.map( (size: Int, count: Int) => count ).reduce(_ + _)
  val buf_ptr_width = log2ceil(num_msg_buffer_words)
  require (isPow2(num_msg_buffers))

  /* Memories (i.e. tables) and Queues */
  // freelist to keep track of available tx_msg_ids
  // TODO(sibanez): update FreeList initialization to use Seq[UInt]
  val tx_msg_id_freelist = Module(new FreeList(num_msg_buffers))
  // RAM used to store msgs while they are being reassembled and delivered to the CPU.
  //   Msgs are stored in words that are the same size as the datapath width.
  val msg_buffer_ram = SyncReadMem(num_msg_buffer_words, Vec(LNICConsts.NET_DP_WIDTH/xLen, UInt(xLen.W)))

  // Vector of Regs containing the buffer size of each size class
  val size_class_buf_sizes = RegInit(Vec(LNICConsts.MSG_BUFFER_COUNT.map( (size: Int, count: Int) => size.U(16.W) )))
  // Vector of freelists to keep track of available buffers to store msgs.
  //   There is one free list for each size class.
  var ptr = 0
  val size_class_freelists = for ((size, count) <- LNICConsts.MSG_BUFFER_COUNT) yield {
    require(size % LNICConsts.NET_DP_BYTES == 0, "Size of each buffer must be evenly divisible by word size.")
    // compute the buffer pointers to insert into each free list
    val buf_ptrs = for (i <- 0 until count) yield {
        val p = ptr
        ptr = p + size/LNICConsts.NET_DP_BYTES
        p.U(buf_ptr_width.W)
    }
    val free_list = Module(new FreeList(buf_ptrs))
    free_list
  }
  val size_class_freelists_io = Vec(size_class_freelists.map(_.io))
  // FIFO queue to schedule delivery of TX pkts
  // TODO(sibanez): this should become a PIFO ideally
  val scheduled_pkts_enq = Wire(Decoupled(new TxPktDescriptor(buf_ptr_width))
  val scheduled_pkts_deq = Wire(Flipped(Decoupled(new TxPktDescriptor(buf_ptr_width)))
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
  val context_enq_state = Reg(VecInit(Seq.fill(num_contexts)(new ContextEnqState)))

  // msg buffer write port
  val enq_buf_ptr = Wire(UInt(buf_ptr_width.W))
  val enq_msg_buf_ram_port = msg_buffer_ram(enq_buf_ptr)

  switch (enqStates(enq_context)) {
    is (sWaitAppHdr) {
      io.net_in.ready := (available_classes > 0.U)
      when (io.net_in.valid && io.net_in.ready) {
        assert (tx_msg_id_freelist.valid, "There is an available buffer but not an available tx_msg_id?")
        // read tx_msg_id_freelist
        tx_msg_id_freelist.ready := true.B
        val tx_msg_id = tx_msg_id_freelist.bits
        // read from target size class freelist
        val target_size_class = PriorityEncoder(available_classes)
        val target_freelist = size_class_freelists_io(target_size_class)
        target_freelist.deq.ready := true.B
        // record per-context enqueue state
        val buf_ptr = target_freelist.deq.bits
        context_enq_state(enq_context).tx_app_hdr := tx_app_hdr
        context_enq_state(enq_context).tx_msg_id := tx_msg_id
        context_enq_state(enq_context).size_class := target_size_class
        context_enq_state(enq_context).buf_ptr := buf_ptr
        context_enq_state(enq_context).pkt_offset := 0.U
        context_enq_state(enq_context).rem_bytes := tx_app_hdr.msg_len
        context_enq_state(enq_context).pkt_bytes := 0.U
        context_enq_state(enq_context).word_count := 0.U // counts the number of words written by CPU for this msg (not including app hdr)
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
        val word_offset_bits = log2ceil(LNICConsts.NET_DP_WIDTH/xLen)
        val word_offset = ctx_state.word_count(word_offset_bits-1, 0)
        val word_ptr = ctx_state.word_count(15, word_offset_bits)
        enq_buf_ptr := ctx_state.buf_ptr + word_ptr
        // TODO(sibanez): check that this properly implements a sub-word write
        enq_msg_buf_ram_port(word_offset) := io.net_in.bits.data
        // build tx pkt descriptor just in case we need to schedule a pkt
        val tx_pkt_desc = Wire(new TxPktDescriptor(buf_ptr_width))
        tx_pkt_desc.tx_msg_id := ctx_state.tx_msg_id
        tx_pkt_desc.tx_pkts := 1.U << ctx_state.pkt_offset
        tx_pkt_desc.buf_ptr := ctx_state.buf_ptr
        tx_pkt_desc.size_class := ctx_state.size_class
        tx_pkt_desc.tx_app_hdr := ctx_state.tx_app_hdr
        tx_pkt_desc.src_context := enq_context

        val is_last_word = ctx_state.rem_bytes <= (xLen/8).U
        val is_full_pkt = ctx_state.pkt_bytes + (xLen/8).U === LNICConsts.MAX_PKT_LEN_BYTES

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
        ctx_state.rem_bytes := ctx_state.rem_bytes - (xLen/8).U
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
  val deq_buf_ptr = Wire(UInt(buf_ptr_width.W))
  val deq_msg_buf_ram_port = msg_buffer_ram(deq_buf_ptr)
  val deq_buf_ptr_reg = RegInit(0.U(buf_ptr_width.W))

  // register to store descriptor of pkt(s) currently being transmitted
  val active_tx_desc_reg = Reg(new TxPktDescriptor(buf_ptr_width))
  // register to track the number of bytes remaining to send for the current pkt
  val deq_pkt_rem_bytes_reg = RegInit(0.U(16.W))
  // register to track current pkt offset
  val deq_pkt_offset_reg = RegInit(0.U(16.W))
  // register to track first word of each pkt (used to drive meta_out.valid)
  val is_first_word_reg = RegInit(false.B)

  // defaults
  io.net_out.valid := false.B
  io.net_out.bits.data := deq_msg_buf_ram_port.asUInt
  io.net_out.bits.keep := LNICConsts.NET_DP_FULL_KEEP
  io.net_out.bits.last := false.B

  io.meta_out.valid = false.B

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
      val is_last_word = deq_pkt_rem_bytes_reg <= LNICConsts.NET_DP_BYTES.U
      io.net_out.data.keep := Mux(is_last_word,
                                  (1.U << deq_pkt_rem_bytes_reg) - 1.U,
                                  LNICConsts.NET_DP_FULL_KEEP)
      io.net_out.data.last := is_last_word
      io.meta_out.valid := is_first_word_reg
      io.meta_out.bits.dst_ip := active_tx_desc_reg.tx_app_hdr.dst_ip
      io.meta_out.bits.dst_context := active_tx_desc_reg.tx_app_hdr.dst_context
      io.meta_out.bits.msg_len := active_tx_desc_reg.tx_app_hdr.msg_len
      io.meta_out.bits.pkt_offset := deq_pkt_offset_reg
      io.meta_out.bits.src_context := active_tx_desc_reg.src_context
      io.meta_out.bits.tx_msg_id := active_tx_desc_reg.tx_msg_id
      io.meta_out.bits.buf_ptr := active_tx_desc_reg.buf_ptr
      io.meta_out.bits.buf_size_class := active_tx_desc_reg.size_class

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
          deq_pkt_rem_bytes_reg := deq_pkt_rem_bytes_reg - LNICConsts.NET_DP_BYTES.U
          // no longer the first word
          is_first_word_reg := false.B
        }
      }
    }
  }

  def tx_next_pkt(val descriptor: TxPktDescriptor) = {
    // find the next pkt to transmit
    val pkt_offset = PriorityEncoder(descriptor.tx_pkts)
    deq_pkt_offset_reg := pkt_offset
    // find the word offset from the buf_ptr: pkt_offset*words_per_mtu
    require(isPow2(LNICConsts.MAX_PKT_LEN_BYTES), "MAX_PKT_LEN_BYTES must be a power of 2!")
    val word_offset = pkt_offset << (log2ceil(LNICConsts.MAX_PKT_LEN_BYTES/LNICConsts.NET_DP_BYTES)).U
    // start reading the first word of the first pkt
    deq_buf_ptr := descriptor.buf_ptr + word_offset
    deq_buf_ptr_reg := deq_buf_ptr
    // register to track first word of each pkt
    is_first_word_reg := true.B
    // record the updated descriptor
    val new_descriptor = descriptor
    new_descriptor.tx_pkts := descriptor.tx_pkts ^ (1.U << pkt_offset)
    active_tx_desc_reg := new_descriptor
    // record bytes remaining for the current pkt
    val num_pkts = compute_num_pkts(descriptor.msg_len)
    when (pkt_offset === num_pkts - 1.U) {
        // this is the last pkt of the msg
        // compute the number of bytes in the last pkt of the msg
        val msg_len_mod_mtu = descriptor.msg_len(log2ceil(LNICConsts.MAX_PKT_LEN_BYTES)-1, 0) 
        val final_pkt_bytes = Mux(msg_len_mod_mtu === 0.U,
                                  LNICConsts.MAX_PKT_LEN_BYTES.U,
                                  msg_len_mod_mtu)
        deq_pkt_rem_bytes_reg := final_pkt_bytes
    } .otherwise {
        // this is not the last pkt of the msg
        deq_pkt_rem_bytes_reg := LNICConsts.MAX_PKT_LEN_BYTES.U
    }
  }

}

