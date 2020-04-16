
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

/**
 * LNIC Assemble classes.
 *
 * Tasks:
 *   - Reassemble potentially multi-pkt msgs then deliver to CPU
 *   - Reverse byte order of data going to CPU
 *   - Allocate rx_msg_ids and buffers to msgs
 */
class AssembleIO extends Bundle {
  val net_in = Flipped(Decoupled(new StreamChannel(LNICConsts.NET_DP_WIDTH)))
  val meta_in = Flipped(Valid(new PISAIngressMetaOut))
  val net_out = Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH))
  val meta_out = Valid(new AssembleMetaOut)
  val getRxMsgInfo = Flipped(new GetRxMsgInfoIO)

  override def cloneType = new AssembleIO().asInstanceOf[this.type]
}

class AssembleMetaOut extends Bundle {
  val dst_context = UInt(LNICConsts.LNIC_CONTEXT_BITS.W)
}

/* TODO(sibanez): move this to LNICPISA.scala */
class GetRxMsgInfoIO extends Bundle {
  val req = Valid(new GetRxMsgInfoReq)
  val resp = Flipped(Valid(new GetRxMsgInfoResp))
}

class GetRxMsgInfoReq extends Bundle {
  val src_ip = UInt(32.W)
  val src_context = UInt(LNICConsts.LNIC_CONTEXT_BITS.W)
  val tx_msg_id = UInt(LNICConsts.LNIC_MSG_ID_BITS.W)
  val msg_len = UInt(16.W)
}

class GetRxMsgInfoResp extends Bundle {
  val fail = Bool()
  val rx_msg_id = UInt(LNICConsts.LNIC_MSG_ID_BITS.W)
  // TODO(sibanez): add additional fields for transport processing
}

class BufInfoTableEntry(val buf_ptr_width: Int) extends Bundle {
  val buf_ptr = UInt(buf_ptr_width.W)
  val buf_size = UInt(16.W)
  // index of the corresponding size_class_freelist
  val size_class = UInt(log2ceil(LNICConsts.MSG_BUFFER_COUNT.size()).W)
}

class RxMsgIdTableEntry extends Bundle {
  val valid = Bool()
  val rx_msg_id = UInt(LNICConsts.LNIC_MSG_ID_BITS.W)
}

// The first word delivered to the application at the start of
// every received msg
class RxAppHdr extends Bundle {
  val src_ip = UInt(32.W)
  val src_context = UInt(LNICConsts.LNIC_CONTEXT_BITS.W)
  val msg_len = UInt(16.W)
}

// The MsgDescriptor is the element that is actually scheduled.
// It is inserted into the scheduler when the msg is fully reassembled.
// When the scheduler selects a descriptor the dequeue logic uses
// the info to deliver the indicated msg to the CPU.
class MsgDescriptor(val buf_ptr_width: Int) extends Bundle {
  val rx_msg_id = UInt(LNICConsts.LNIC_MSG_ID_BITS.W)
  val size_class = UInt(log2ceil(LNICConsts.MSG_BUFFER_COUNT.size()).W)
  val buf_ptr = UInt(buf_ptr_width.W)
  val dst_context = UInt(LNICConsts.LNIC_CONTEXT_BITS.W)
  val rx_app_hdr = new RxAppHdr()
}

def compute_num_pkts(msg_len: UInt) = {
  require(isPow2(LNICConsts.MAX_PKT_LEN_BYTES))
  // check if msg_len is divisible by MAX_PKT_LEN_BYTES
  val num_pkts = Mux(msg_len(log2ceil(LNICConsts.MAX_PKT_LEN_BYTES)-1, 0) === 0.U,
                     msg_len >> log2ceil(LNICConsts.MAX_PKT_LEN_BYTES).U,
                     msg_len >> log2ceil(LNICConsts.MAX_PKT_LEN_BYTES).U + 1.U)
  num_pkts
}

@chiselName
class LNICAssemble(implicit p: Parameters) extends Module {
  val io = IO(new AssembleIO)

  // parameter computations
  val num_msg_buffer_words = LNICConsts.MSG_BUFFER_COUNT.map( (size: Int, count: Int) => (size/LNICConsts.NET_DP_BYTES)*count ).reduce(_ + _)
  val num_msg_buffers = LNICConsts.MSG_BUFFER_COUNT.map( (size: Int, count: Int) => count ).reduce(_ + _)
  val buf_ptr_width = log2ceil(num_msg_buffer_words)
  require (isPow2(num_msg_buffers))

  /* Memories (i.e. tables) and Queues */
  // freelist to keep track of available rx_msg_ids
  val rx_msg_id_freelist = Module(new FreeList(num_msg_buffers))
  // table mapping unique msg identifier to rx_msg_id
  // TODO(sibanez): this should eventually turn into a D-left exact-match table
  val rx_msg_id_table = SyncReadMem(num_msg_buffers, new RxMsgIdTableEntry())
  // TODO(sibanez): is there a way to move the msg_buffer_ram and size_class_freelists
  //   into a separate module (e.g. MessageBuffer) so that the implementation is
  //   usable for both the Reassembly and Packetization modules?
  // RAM used to store msgs while they are being reassembled and delivered to the CPU.
  //   Msgs are stored in words that are the same size as the datapath width.
  val msg_buffer_ram = SyncReadMem(num_msg_buffer_words, UInt(LNICConsts.NET_DP_WIDTH.W))
  // table mapping {rx_msg_id => received_bitmap}
  val received_table = SyncReadMem(num_msg_buffers, UInt(LNICConsts.MAX_PKTS_PER_MSG.W))
  // table mapping {rx_msg_id => buffer info}
  val buf_info_table = SyncReadMem(num_msg_buffers, new BufInfoTableEntry(buf_ptr_width))
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
  // FIFO queue to schedule delivery of fully assembled msgs to the CPU
  // TODO(sibanez): this should become a PIFO ideally, or at least per-context queues each with an associated priority
  val scheduled_msgs_enq = Wire(Decoupled(new MsgDescriptor(buf_ptr_width))
  val scheduled_msgs_enq = Wire(Flipped(Decoupled(new MsgDescriptor(buf_ptr_width)))
  scheduled_msgs_deq <> Queue(scheduled_msgs_enq, num_msg_buffers)

  /* GetRxMsgInfo State Machine:
   *   - Process getRxMsgInfo() extern function calls
   *   - Returns rx_msg_id (or all 1's if failure)
   *   - Initialize rx msg state
   *
   * Description states:
   * sLookupMsg:
   *   - Lookup the unique msg identifier in the rx_msg_id_table: key = {src_ip, src_context, tx_msg_id}
   *   - Perform computations to see if there is a buffer and rx_msg_id available (in case this is a new msg)
   * sWriteback:
   *   - Return result of extern function call: {failure, rx_msg_id}
   *   - Write rx_msg_id_table[key] = rx_msg_id
   *   - Write buf_info_table[rx_msg_id] = buf_info
   *   - Write received_table[rx_msg_id] = 0
   *   - Write app_hdr_table[rx_msg_id] = {src_ip, src_context, msg_len}
   * TODO(sibanez): Need to read received_table[rx_msg_id] in sWriteback and add a
   *   pipeline stage to process the received bitmap and return the result.
   */
  val sLookupMsg :: sWriteback :: Nil = Enum(2)
  val stateRxMsgInfo = RegInit(sLookupMsg)

  // register the extern function call request parameters
  // NOTE: this assumes requests will not arrive on back-to-back cycles - TODO: may want to check this assumption
  val getRxMsgInfo_req_reg = RegNext(io.getRxMsgInfo.req)

  // TODO(sibanez): update msg_key to include src_ip and src_context,
  //   which requires rx_msg_id_table to become a D-left lookup table.
  val msg_key = getRxMsgInfo_req_reg.bits.tx_msg_id
  val rx_msg_id_table_port = rx_msg_id_table(msg_key)
  val cur_rx_msg_id_table_entry = Wire(new RxMsgIdTableEntry())
  // TODO(sibanez): add defaults for the above?

  // True if both an rx_msg_id and buffer are available for this msg
  val allocation_success_reg = RegInit(false.B)
  // The size_class from which to allocate a buffer, only valid if allocation_success_reg === true.B
  val size_class_reg = RegInit(0.U)
  // bitmap of valid signals for all size classes
  val free_classes = size_class_freelists_io.map(_.deq.valid)
  // bitmap of size_classes that are large enough to store the whole msg
  val candidate_classes = size_class_buf_sizes.map(_ >= getRxMsgInfo_req_reg.bits.msg_len)
  // bitmap indicates classes with available buffers that are large enough
  val available_classes = free_classes.asUInt & candidate_classes.asUInt

  switch (stateRxMsgInfo) {
    is (sLookupMsg) {
      // wait for a request to arrive
      when (getRxMsgInfo_req_reg.valid) {
        stateRxMsgInfo := sWriteback
        // read rx_msg_id_table[msg_key] => msg_key is updated on this cycle, result is available on the next one
        // check if there is an available buffer for this msg
        allocation_success_reg := available_classes > 0.U
        // find the smallest available buffer that can hold the msg
        size_class_reg := PriorityEncoder(available_classes)
      }
    }
    is (sWriteback) {
      stateRxMsgInfo := sLookupMsg
      // return extern call response
      io.getRxMsgInfo.resp.valid := true.B
      // Get result of reading the rx_msg_id_table
      cur_rx_msg_id_table_entry := rx_msg_id_table_port
      when (cur_rx_msg_id_table_entry.valid) {
        // This msg has already been allocated an rx_msg_id
        io.getRxMsgInfo.resp.bits.fail := false.B
        io.getRxMsgInfo.resp.bits.rx_msg_id := cur_rx_msg_id_table_entry.rx_msg_id
      } .elsewhen (allocation_success_reg) {
        // This is a new msg and we can allocate a buffer and rx_msg_id
        io.getRxMsgInfo.resp.bits.fail := false.B
        val rx_msg_id = rx_msg_id_freelist.io.deq.bits
        io.getRxMsgInfo.resp.bits.rx_msg_id := rx_msg_id
        // read from rx_msg_id freelist
        assert(rx_msg_id_freelist.io.deq.valid, "There is an available buffer but not an available rx_msg_id?")
        rx_msg_id_freelist.io.deq.ready := true.B
        // update rx_msg_id_table
        val new_rx_msg_id_table_entry = Wire(new RxMsgIdTableEntry())
        new_rx_msg_id_table_entry.valid := true.B
        new_rx_msg_id_table_entry.rx_msg_id := rx_msg_id
        rx_msg_id_table_port := new_rx_msg_id_table_entry
        // update buf_info_table
        val target_freelist = size_class_freelists_io(size_class_reg)
        target_freelist.deq.ready := true.B // read from freelist
        val new_buf_info_table_entry = Wire(new BufInfoTableEntry())
        new_buf_info_table_entry.buf_ptr := target_freelist.deq.bits
        new_buf_info_table_entry.buf_size := size_class_buf_sizes(size_class_reg)
        new_buf_info_table_entry.size_class := size_class_reg
        buf_info_table(rx_msg_id) := new_buf_info_table_entry
        // update received_table
        received_table(rx_msg_id) := 0.U
      } .otherwise {
        // This is a new msg and we cannot allocate a buffer and rx_msg_id
        io.getRxMsgInfo.resp.bits.fail = true.B
        io.getRxMsgInfo.resp.bits.rx_msg_id = 0.U
      }
    }
  }

  // TODO(sibanez): this state machine exerts backpressure on the first cycle of every pkt.
  //   It does this because it needs 2 cycles to perform RMW of received_table[rx_msg_id].
  //   There is a way to avoid backpressuring all but single-cycle pkts, but is it worth implementing?
  //   This will still process pkts at line rate if the clock is 400MHz.
  /* Enqueue State Machine:
   *   - Enqueue incomming pkts into the appropriate msg buffer
   *   - Mark the corresponding pkt as having been received
   *
   * Description states:
   * sEnqStart state:
   *   - Lookup buf_info_table[rx_msg_id]
   *   - Lookup received_table[rx_msg_id]
   *   - Exert backpressure
   *
   * sEnqWordOne state:
   *   - Write received_table[rx_msg_id] |= (1 << pkt_offset)
   *   - Write msg_buffer_ram[buf_ptr + pkt_offset*max_words_per_pkt] = first word of pkt
   *   - Check if the full msg has been received
   *   - Schedule msg for delivery to CPU if needed
   *
   * sEnqFinishPkt state:
   *   - Write the rest of the words of the pkt into the msg buffer
   *   - Schedule msg for delivery to CPU if needed
   */
  val sEnqStart :: sEnqWordOne :: sEnqFinishPkt :: Nil = Enum(3)
  val stateEnq = RegInit(sEnqStart)

  val meta_in_bits_reg = Reg(new PISAIngressMetaOut())

  val max_words_per_pkt = LNICConsts.MAX_PKT_LEN_BYTES/LNICConsts.NET_DP_BYTES
  require(isPow2(max_words_per_pkt))

  val enq_rx_msg_id = Wire(UInt(LNICConsts.LNIC_MSG_ID_BITS.W))
  // buf_info_table read port
  val enq_buf_info_table_port = buf_info_table(enq_rx_msg_id)
  val buf_info = Wire(new BufInfoTableEntry())
  val buf_info_reg = Reg(new BufInfoTableEntry())
  // received_table read/write port
  val enq_received_table_port = received_table(enq_rx_msg_id)
  val enq_received = Wire(UInt(LNICConsts.MAX_PKTS_PER_MSG.W))
  // msg_buffer_ram write port
  val pkt_word_ptr = Wire(UInt(buf_ptr_width.W))
  val enq_msg_buffer_ram_port = msg_buffer_ram(pkt_word_ptr)

  val msg_complete_reg = Reg(Bool())
  val pkt_word_count = RegInit(0.U(log2ceil(max_words_per_pkt).W))

  // defaults
  io.net_in.ready := true.B
  enq_rx_msg_id := io.meta_in.bits.rx_msg_id

  switch (stateEnq) {
    is (sEnqStart) {
        // wait for the first word to arrive
        when (io.net_in.valid) {
            stateEnq := sEnqWordOne
            // backpressure one cycle
            io.net_in.ready := false.B
            // register pkt metadata
            meta_in_bits_reg := io.meta_in.bits
        }
    }
    is (sEnqWordOne) {
        // NOTE: this assumes that io.net_in and io.meta_in are the same as they were in the sEnqStart state.
        // write pkt word into msg buffer
        buf_info := enq_buf_info_table_port
        buf_info_reg := buf_info
        val pkt_ptr = compute_pkt_ptr(buf_info.buf_ptr, io.meta_in.bits.pkt_offset)
        pkt_word_ptr := pkt_ptr
        enq_msg_buffer_ram_port := io.net_in.bits.data
        pkt_word_count := pkt_word_count + 1.U
        // mark pkt as received
        enq_received := enq_received_table_port
        val new_enq_received = enq_received | (1.U << io.meta_in.bits.pkt_offset)
        enq_received_table_port := new_enq_received
        // check if the whole msg has been received
        val num_pkts = compute_num_pkts(io.meta_in.bits.msg_len)
        val msg_complete = (new_enq_received === (1.U << num_pkts) - 1.U)
        msg_complete_reg := msg_complete
        // state transition
        when (io.net_in.bits.last) {
            stateEnq := sEnqStart
            when (msg_complete) {
                // schedule msg for delivery to the CPU
                schedule_msg(meta_in_bits_reg.dst_context,
                             meta_in_bits_reg.rx_msg_id,
                             buf_info.buf_ptr,
                             buf_info.size_class,
                             meta_in_bits_reg.src_ip,
                             meta_in_bits_reg.src_context,
                             meta_in_bits_reg.msg_len)
            }
        } .otherwise {
            stateEnq := sEnqFinishPkt
        }
    }
    is (sEnqFinishPkt) {
        // write pkt word into msg buffer
        when (io.net_in.valid) {
            val pkt_ptr = compute_pkt_ptr(buf_info_reg.buf_ptr, meta_in_bits_reg.pkt_offset)
            pkt_word_ptr := pkt_ptr + pkt_word_count
            // Make sure we are not writing beyond the buffer
            when (pkt_word_count < (buf_info_reg.buf_size >> log2ceil(LNICConsts.NET_DP_BYTES))) {
                enq_msg_buffer_ram_port := io.net_in.bits.data
                pkt_word_count := pkt_word_count + 1.U
            }
            when (io.net_in.bits.last) {
                // state transition
                stateEnq := sEnqStart
                when (msg_complete_reg) {
                    // schedule msg for delivery to the CPU
                    schedule_msg(meta_in_bits_reg.dst_context,
                                 meta_in_bits_reg.rx_msg_id,
                                 buf_info_reg.buf_ptr,
                                 buf_info_reg.size_class,
                                 meta_in_bits_reg.src_ip,
                                 meta_in_bits_reg.src_context,
                                 meta_in_bits_reg.msg_len)
                }
            }
        }
    }
  }

  def compute_pkt_ptr(buf_ptr: UInt, pkt_offset: UInt) = {
    val pkt_ptr = buf_ptr + (pkt_offset<<log2ceil(max_words_per_pkt).U)
    pkt_ptr
  }

  def schedule_msg(dst_context: UInt, rx_msg_id: UInt, buf_ptr: UInt, size_class: UInt, src_ip: UInt, src_context: UInt, msg_len: UInt) = {
    // TODO(sibanez): this should be inserting into a PIFO or per-context queues rather than
    //   a single fifo queue. This is just a temporary simplification.
    assert (scheduled_msgs_enq.ready, "scheduled_msgs FIFO is full when trying to schedule a msg")
    scheduled_msgs_enq.valid := true.B
    scheduled_msgs_enq.bits.rx_msg_id := rx_msg_id
    scheduled_msgs_enq.bits.size_class := size_class
    scheduled_msgs_enq.bits.buf_ptr := buf_ptr
    scheduled_msgs_enq.bits.dst_context := dst_context
    scheduled_msgs_enq.bits.rx_app_hdr.src_ip := src_ip
    scheduled_msgs_enq.bits.rx_app_hdr.src_context := src_context
    scheduled_msgs_enq.bits.rx_app_hdr.msg_len := msg_len
  }

  /* Delivery State Machine:
   *   - Schedule and perform delivery of fully reassembled msgs to the CPU.
   *   - The order in which msgs are delivered to the CPU should be determined by the priority of the dst_context.
   *   - Free rx_msg_id and msg buffer after the msg is delivered to the CPU
   *   - TODO(sibanez): implement msg delivery scheduling to improve performance (rather than FIFO).
   *
   * Description of states:
   * sScheduleMsg:
   *   - Wait for a msg descriptor to arrive in the scheduled_msgs fifo
   *   - Read the first word of the msg from the msg buffer
   *
   * sDeliverMsg:
   *   - Deliver the selected msg to the CPU
   *   - Reverse byte order of all words
   *   - Free the rx_msg_id and the msg buffer
   */
  val sScheduleMsg :: sDeliverMsg :: Nil = Enum(2)
  val stateDeq = RegInit(sScheduleMsg)

  // message buffer read port
  val deq_buf_word_ptr = scheduled_msgs_deq.bits.buf_ptr // default
  val deq_msg_buf_ram_port = msg_buffer_ram(deq_buf_word_ptr)

  val msg_desc_reg = Reg(new MsgDescriptor(buf_ptr_width))
  val msg_word_count = RegInit(0.U(16.W))
  val rem_bytes_reg = RegInit(0.U(16.W))

  // 512-bit and 64-bit words from the msg buffer
  val buf_net_out_wide = Decoupled(new StreamChannel(LNICConsts.NET_DP_WIDTH))
  val buf_net_out_narrow = Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH))

  // defaults
  // 512-bit => 64-bit
  StreamWidthAdapter(buf_net_out_narrow, // output
                     buf_net_out_wide)   // input
  io.net_out <> buf_net_out_narrow
  io.net_out.bits.data := reverse_bytes(buf_net_out_narrow.bits.data, LNICConsts.NET_IF_BYTES)

  buf_net_out_wide.valid := false.B
  buf_net_out_wide.bits.keep := LNICConsts.NET_DP_FULL_KEEP
  buf_net_out_wide.bits.last := false.B
  scheduled_msgs_deq.ready = false.B

  // TODO(sibanez): update this depending on how the RxQueues module wants to assert backpressure.
  // NOTE: the metadata here is treated differently than usual.
  //   This dst_context metadata field is used to drive io.net_out.ready
  io.meta_out.valid := true.B
  io.meta_out.bits.dst_context := msg_desc_reg.dst_context

  switch (stateDeq) {
    is (sScheduleMsg) {
      // Wait for a msg descriptor to be scheduled
      when (scheduled_msgs_deq.valid) {
        // Write app_hdr
        io.net_out.valid = true.B
        io.net_out.bits.data := scheduled_msgs_deq.bits.rx_app_hdr.asUInt
        io.net_out.bits.keep = LNICConsts.NET_IF_FULL_KEEP
        io.net_out.bits.last = false.B
        io.meta_out.bits.dst_context := scheduled_msgs_deq.bits.dst_context
        when (io.net_out.ready) {
          // read the head scheduled msg
          scheduled_msgs_deq.ready = true.B
          // init regs
          msg_desc_reg := scheduled_msgs_deq.bits
          msg_word_count := 0.U
          rem_bytes_reg := scheduled_msgs_deq.bits.rx_app_hdr.msg_len
          // state transition
          stateDeq := sDeliverMsg
        }
      }
    }
    is (sDeliverMsg) {
      buf_net_out_wide.valid := true.B
      // read current buffer word
      buf_net_out_wide.bits.data := deq_msg_buf_ram_port
      val is_last_word = rem_bytes_reg < LNICConsts.NET_DP_BYTES.U
      buf_net_out_wide.bits.keep := Mux(is_last_word,
                                        (1.U << rem_bytes_reg) - 1.U,
                                        LNICConsts.NET_DP_FULL_KEEP)
      buf_net_out_wide.bits.last := is_last_word
      when (buf_net_out_wide.ready) {
        // move to the next word
        deq_buf_word_ptr := msg_desc_reg.buf_ptr + msg_word_count + 1.U
        msg_word_count := msg_word_count + 1.U
        rem_bytes_reg := rem_bytes_reg - LNICConsts.NET_DP_BYTES.U
        when (is_last_word) {
          // state transition
          stateDeq := sScheduleMsg
          // free the rx_msg_id and the msg buffer
          rx_msg_id_freelist.io.enq.valid := true.B
          rx_msg_id_freelist.io.enq.bits := msg_desc_reg.rx_msg_id
          val target_buf_freelist_io = size_class_freelists_io(msg_desc_reg.size_class)
          target_buf_freelist_io.enq.valid := true.B
          target_buf_freelist_io.enq.bits := msg_desc_reg.buf_ptr
        }
      } .otherwise {
        // stay at the same word
        deq_buf_word_ptr := msg_desc_reg.buf_ptr + msg_word_count
      }
    }
  }

}


