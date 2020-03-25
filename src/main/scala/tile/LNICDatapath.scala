
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
 * LNIC Pktization Buffer classes.
 *
 * Tasks:
 *   - Reverse byte order of words coming from CPU
 *   - Consume messages from the CPU and transform into pkts
 *   - For now, assume all msgs consist of one pkt
 *   - Maintain per-context FIFOs
 *   - Insert / remove contexts when told to do so
 *   - Store msg until either:
 *     (1) the whole thing arrives
 *     (2) an MTU arrives, then send that MTU -- TODO(sibanez): unimplemented
 */
class PktizeMetaOut extends Bundle {
  val msg_id = UInt(16.W)
  val offset = UInt(16.W) // pkt offset within msg
  val lnic_src = UInt(LNICConsts.LNIC_CONTEXT_BITS.W) // src context ID

  override def cloneType = new PktizeMetaOut().asInstanceOf[this.type]
}

class PktizeIO extends Bundle {
  val net_in = Flipped(Decoupled(UInt(width = 64))) // words written from the CPU
  val net_out = Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH))
  val meta_out = Valid(new PktizeMetaOut)

  val cur_context = Input(UInt(width = LNICConsts.LNIC_CONTEXT_BITS))
  val insert = Input(Bool())
  //val remove = Input(Bool()) // TODO(sibanez): we won't support remove for now

  override def cloneType = new PktizeIO().asInstanceOf[this.type]
}

class FIFOWord(val ptrBits: Int) extends Bundle {
  val word = UInt(width = LNICConsts.NET_IF_WIDTH)
  val next = UInt(width = ptrBits)

  override def cloneType = new FIFOWord(ptrBits).asInstanceOf[this.type]
}

class HeadTableEntry(val ptrBits: Int) extends Bundle {
  val valid = Bool()
  val head = UInt(width = ptrBits)

  override def cloneType = new HeadTableEntry(ptrBits).asInstanceOf[this.type]
}

class TailTableEntry(val ptrBits: Int) extends Bundle {
  val valid = Bool()
  val tail = UInt(width = ptrBits)

  override def cloneType = new TailTableEntry(ptrBits).asInstanceOf[this.type]
}

class DequeueReq extends Bundle {
  val context_id = UInt(LNICConsts.LNIC_CONTEXT_BITS.W) // the context to dequeue from
  val msg_len = UInt(16.W)

  override def cloneType = new DequeueReq().asInstanceOf[this.type]
}

@chiselName
class LNICPktize(implicit p: Parameters) extends Module {
  val entries = p(LNICKey).pktizePktBufFlits
  val num_contexts = p(LNICKey).maxNumContexts

  val io = IO(new PktizeIO)

  val ptrBits = log2Ceil(entries)
  // create memory used to store msg words
  val ram = Mem(entries, new FIFOWord(ptrBits))
  // create free list for msg words
  val freelist = Module(new FreeList(entries))
  // create tables for indexing head/tail of FIFOs in the RAM
  // TODO(sibanez): currently just using context_id to directly index the tables.
  //   These should eventually turn into propper d-left lookup tables
  val head_table = Mem(num_contexts, new HeadTableEntry(ptrBits))
  val tail_table = Mem(num_contexts, new TailTableEntry(ptrBits))

  val cur_head_entry = Wire(new HeadTableEntry(ptrBits))
  val new_head_entry = Wire(new HeadTableEntry(ptrBits))
  val cur_tail_entry = Wire(new TailTableEntry(ptrBits))
  val new_tail_entry = Wire(new TailTableEntry(ptrBits))

  // queue to store dequeue requests sent from the enqueue side to the dequeue side
  val deqReq_in = Wire(Decoupled(new DequeueReq))
  val deqReq_out = Wire(Flipped(Decoupled(new DequeueReq)))
  deqReq_out <> Queue(deqReq_in, num_contexts*8) // TODO(sibanez): what is the best way to size this queue?

  // default deqReq_in / deqReq_out - do not enq or deq
  deqReq_in.valid := false.B
  deqReq_in.bits.context_id := 0.U
  deqReq_in.bits.msg_len := 0.U
  deqReq_out.ready := false.B


  // create regs to store count for each context
  val counts = RegInit(VecInit(Seq.fill(num_contexts)(0.U(log2Ceil(entries + 1).W))))

  // create regs to store the number of remaining bytes to be enqueued for the current msg of each context
  val msg_lens = RegInit(VecInit(Seq.fill(num_contexts)(0.U(16.W))))
  val enq_rem_bytes = RegInit(VecInit(Seq.fill(num_contexts)(0.U(16.W))))

  // generate msg_ids using simple counter
  val reg_msg_count = RegInit(0.U(16.W))

  val cur_index = io.cur_context
  val deq_index = Wire(UInt())
  deq_index := 0.U  // default

  val do_enq = Wire(Bool())
  val do_deq = Wire(Bool())
  do_enq := false.B
  do_deq := false.B

  // defaults
  // do not enq or deq from the free list
  freelist.io.enq.valid := false.B
  freelist.io.enq.bits := 0.U
  freelist.io.deq.ready := false.B

  // don't receive incomming words from the CPU
  io.net_in.ready := false.B

  io.net_out.valid := false.B
  io.net_out.bits.data := 0.U
  io.net_out.bits.keep := LNICConsts.NET_FULL_KEEP
  io.net_out.bits.last := false.B

  io.meta_out.valid := false.B
  io.meta_out.bits.msg_id := reg_msg_count
  io.meta_out.bits.offset := 0.U // TODO(sibanez): assuming single pkt msgs for now
  io.meta_out.bits.lnic_src := 0.U // default


  // Logic to insert new contexts.
  //   when insert is asserted, insert the cur_context into the head/tail_tables
  //   and set count to 0
  when (io.insert) {
    // do not perform an enqueue because we need to update tail_table
    io.net_in.ready := false.B
    // update head_table, try to read from free list
    freelist.io.deq.ready := true.B
    assert (freelist.io.deq.valid, "LNICPktize: freelist is empty during context insertion!")
    new_head_entry.valid := freelist.io.deq.valid
    new_head_entry.head := freelist.io.deq.bits
    head_table(cur_index) := new_head_entry
    // update tail_table
    new_tail_entry.valid := freelist.io.deq.valid
    new_tail_entry.tail := freelist.io.deq.bits
    tail_table(cur_index) := new_tail_entry
    // initialize count
    counts(cur_index) := 0.U
  } .otherwise {

    /**
     * State machine to enqueue into the FIFOs.
     * Tasks:
     *   - Record msg length
     *   - Count number of words that have been enqueued so far for this msg
     *   - Trigger dequeue for cur_context when the last word has been enqueued
     * TODO(sibanez): for multi-pkt msgs will also need to trigger dequeue when one MTU has been enqueued
     * TODO(sibanez): should we drop any message here (before enqueue?)
     */
    val sStartEnq :: sEnqueue :: Nil = Enum(2)
    // need one state per context
    val enqStates = RegInit(VecInit(Seq.fill(num_contexts)(sStartEnq)))

    // this module should not assert back pressure (except during context insertion)
    // TODO(sibanez): need to figure out what to do when queue is full and incomming words are dropped.
    //   Probably need to detect this scenario then tell the dequeue logic to drop the msg.
    //   Do we need to divide the buffer space amongst contexts so that high priority msgs are not
    //   dropped because of low priority apps using all the buffer space?
    //   Does this buffer actually fill up? Only if the applications are sending faster than the NIC
    //   can put pkts onto the wire, which ideally shouldn't be possible.
    io.net_in.ready := true.B

    switch(enqStates(cur_index)) {
      is (sStartEnq) {
        when (io.net_in.valid) {
          val msg_len = io.net_in.bits(15, 0)
          msg_lens(cur_index) := msg_len
          enq_rem_bytes(cur_index) := msg_len
          perform_enq(cur_index)
          enqStates(cur_index) := sEnqueue
        }
      }
      is (sEnqueue) {
        when (io.net_in.valid) {
          perform_enq(cur_index)
          // NOTE: following logic assumes 8B words (64-bit architecture)
          when (enq_rem_bytes(cur_index) <= 8.U) {
            // this is the last word of the msg so submit a dequeue request for the current context
            deqReq_in.valid := true.B
            deqReq_in.bits.context_id := cur_index
            deqReq_in.bits.msg_len := msg_lens(cur_index)
            assert (deqReq_in.ready, "LNICPktize: dequeue request FIFO full during enqueue!")
            enqStates(cur_index) := sStartEnq
          } .otherwise {
            enq_rem_bytes(cur_index) := enq_rem_bytes(cur_index) - 8.U
          }
        }
      }
    }

    /**
     * State machine to perform dequeue operations.
     * Tasks:
     *   - Wait until there is a dequeue request to process
     *   - Perform dequeue indicated by the request and set the last bit
     *   - Swap bytes
     *   - TODO(sibanez): support drop requests sent from enqueue side
     */
    val sStartDeq :: sDequeue :: Nil = Enum(2)
    val deqState = RegInit(sStartDeq)

    val reg_deq_index = RegInit(0.U(LNICConsts.LNIC_CONTEXT_BITS.W))
    val deq_rem_bytes = RegInit(0.U(16.W))

    switch (deqState) {
      is (sStartDeq) {
        when (deqReq_out.valid) {
          // there a msg waiting for dequeue, tell the arbiter about it
          io.net_out.valid := true.B
          io.meta_out.valid := true.B
          when (io.net_out.ready) {
            // the arbiter is ready to receive data
            deqReq_out.ready := true.B
            deq_index := deqReq_out.bits.context_id
            deq_rem_bytes := deqReq_out.bits.msg_len
            reg_deq_index := deq_index
            // write metadata
            io.meta_out.bits.lnic_src := deq_index
            // dequeue app hdr
            perform_deq(deq_index)
            // state transition
            deqState := sDequeue
            reg_msg_count := reg_msg_count + 1.U
            // NOTE: cannot perform dequeue in this state because LNICArbiter (the attached module)
            //   waits for valid to be set before asserting ready
          }
        }
      }
      is (sDequeue) {
        io.net_out.valid := true.B
        when (io.net_out.ready) {
          deq_index := reg_deq_index
          perform_deq(deq_index)
          when (deq_rem_bytes <= 8.U) {
            // this is the last word of the msg
            io.net_out.bits.last := true.B
            io.net_out.bits.keep := (1.U << deq_rem_bytes) - 1.U
            deqState := sStartDeq
          } .otherwise {
            deq_rem_bytes := deq_rem_bytes - 8.U
          }
        }
      }
    }

    // Logic to count the size of each FIFO, updated on enq/deq operations
    for (i <- 0 until counts.size) {
      val enq_inc = Mux(do_enq && (cur_index === i.U), 1.U, 0.U)
      val deq_dec = Mux(do_deq && (deq_index === i.U), 1.U, 0.U)
      counts(i.U) := counts(i.U) + enq_inc - deq_dec
    }
  }

  def perform_enq(index: UInt) = {
    do_enq := true.B
    // lookup current tail_ptr for this context
    cur_tail_entry := tail_table(index)
    val tail_ptr = cur_tail_entry.tail
    assert (cur_tail_entry.valid, "Attempting to perform an enqueue for an invalid contextID")

    // read free list
    val tail_ptr_next = freelist.io.deq.bits
    freelist.io.deq.ready := true.B
    assert (freelist.io.deq.valid, "Free list is empty during msg enqueue!")

    // write new word to RAM
    val enq_fifo_word = Wire(new FIFOWord(ptrBits))
    enq_fifo_word.word := io.net_in.bits
    enq_fifo_word.next := tail_ptr_next
    ram(tail_ptr) := enq_fifo_word

    // update tail ptr
    new_tail_entry := cur_tail_entry // default
    new_tail_entry.tail := tail_ptr_next
    tail_table(index) := new_tail_entry
  }

  def perform_deq(index: UInt) = {
    do_deq := true.B
    // lookup the current head of the FIFO for this context
    cur_head_entry := head_table(index)
    val head_ptr = cur_head_entry.head
    assert (cur_head_entry.valid, "Attempting to perform dequeue for an invalid contextID")

    // read the head word from the RAM
    val deq_fifo_word = Wire(new FIFOWord(ptrBits))
    deq_fifo_word := ram(head_ptr)
    // reverse bytes of outing data
    io.net_out.bits.data := reverse_bytes(deq_fifo_word.word, LNICConsts.NET_IF_BYTES)
    val head_ptr_next = deq_fifo_word.next

    // update head table
    new_head_entry := cur_head_entry // default
    new_head_entry.head := head_ptr_next
    // add current head to the free list
    freelist.io.enq.valid := true.B
    freelist.io.enq.bits := cur_head_entry.head
    assert (freelist.io.enq.ready, "Free list is full during dequeue!")
    head_table(index) := new_head_entry
  }
}

/**
 * LNIC Arbiter classes
 */
class ArbiterIO extends Bundle {
  val core_in = Flipped(Decoupled(new StreamChannel(LNICConsts.NET_DP_WIDTH)))
  val core_meta_in = Flipped(Valid(new PktizeMetaOut))
  val net_in = Flipped(Decoupled(new StreamChannel(LNICConsts.NET_DP_WIDTH)))
  val net_out = Decoupled(new StreamChannel(LNICConsts.NET_DP_WIDTH))
  val meta_out = Valid(new PISAMetaIO)

  override def cloneType = new ArbiterIO().asInstanceOf[this.type]
}

@chiselName
class LNICArbiter(implicit p: Parameters) extends Module {
  val io = IO(new ArbiterIO)

  val pktQueue_in = Wire(Decoupled(new StreamChannel(LNICConsts.NET_DP_WIDTH)))
  val metaQueue_in = Wire(Decoupled(new PISAMetaIO))
  val metaQueue_out = Wire(Flipped(Decoupled(new PISAMetaIO)))

  // Set up output queues
  io.net_out <> Queue(pktQueue_in, p(LNICKey).arbiterPktBufFlits)
  metaQueue_out <> Queue(metaQueue_in, p(LNICKey).arbiterMetaBufFlits)

  // state machine to arbitrate between core_in and net_in
  // TODO(sibanez): Eventually, I want this arbiter to drain the core_in @ up to line rate,
  //   which will prevent pkt drops further down the pipeline.
  //   The remaining bandwidth will be used to serve net_in.
  //   The datapath width should increase to 128 bits so both inputs can be served at line rate.

  val sInSelect :: sInWaitEnd :: Nil = Enum(2)
  val inState = RegInit(sInSelect)

  val reg_selected = RegInit(false.B) // true = CPU, false = network

  // default pktQueue_in
  pktQueue_in.valid := false.B
  pktQueue_in.bits := io.net_in.bits

  metaQueue_in.valid := false.B
  // defaults - used for pkts from network
  metaQueue_in.bits.ingress_id := false.B
  metaQueue_in.bits.msg_id := 0.U
  metaQueue_in.bits.offset := 0.U
  metaQueue_in.bits.lnic_src := 0.U
  // initialize unused metadata fields (driven by PISA module)
  metaQueue_in.bits.egress_id := 0.U;
  metaQueue_in.bits.lnic_dst := 0.U;
  metaQueue_in.bits.msg_len := 0.U;

  io.core_in.ready := false.B
  io.net_in.ready := false.B

  def selectCore() = {
    reg_selected := true.B
    pktQueue_in <> io.core_in
    metaQueue_in.valid := true.B
    metaQueue_in.bits.ingress_id := true.B
    metaQueue_in.bits.msg_id := io.core_meta_in.bits.msg_id
    metaQueue_in.bits.offset := io.core_meta_in.bits.offset
    metaQueue_in.bits.lnic_src := io.core_meta_in.bits.lnic_src
    when (pktQueue_in.valid && pktQueue_in.ready && pktQueue_in.bits.last) {
      inState := sInSelect // stay in same state
    } .otherwise {
      inState := sInWaitEnd
    }
  }

  def selectNet() = {
    reg_selected := false.B
    pktQueue_in <> io.net_in
    metaQueue_in.valid := true.B
    when (pktQueue_in.valid && pktQueue_in.ready && pktQueue_in.bits.last) {
      inState := sInSelect // stay in same state
    } .otherwise {
      inState := sInWaitEnd
    }
  }

  switch (inState) {
    is (sInSelect) {
      // select which input to read from
      // write to metaQueue and potentially pktQueue as well
      when (reg_selected && io.net_in.valid) {
        // selected CPU last time and net_in is valid
        selectNet()
      } .elsewhen (!reg_selected && io.core_in.valid) {
        // selected network last time and core_in is valid
        selectCore()
      } .elsewhen (reg_selected && io.core_in.valid) {
        // selected CPU last time and core_in is valid
        selectCore()
      } .elsewhen (!reg_selected && io.net_in.valid) {
        // selected network last time and net_in is valid
        selectNet()
      }
    }
    is (sInWaitEnd) {
      when (reg_selected) {
        // CPU input selected
        pktQueue_in <> io.core_in
      } .otherwise {
        // network input selected
        pktQueue_in <> io.net_in
      }
      // wait until end of selected pkt then transition back to sSelect
      when (pktQueue_in.valid && pktQueue_in.ready && pktQueue_in.bits.last) {
        inState := sInSelect
      }
    }
  }

  // state machine to drive metaQueue_out.ready
  val sOutWordOne :: sOutWaitEnd :: Nil = Enum(2)
  val outState = RegInit(sOutWordOne)

  // only read metaQueue when first word is transferred to PISA pipeline
  metaQueue_out.ready := (outState === sOutWordOne) && (io.net_out.valid && io.net_out.ready)
  io.meta_out <> metaQueue_out

  switch (outState) {
    is (sOutWordOne) {
      when (io.net_out.valid && io.net_out.ready && !io.net_out.bits.last) {
        outState := sOutWaitEnd
      }
    }
    is (sOutWaitEnd) {
      when (io.net_out.valid && io.net_out.ready && io.net_out.bits.last) {
        outState := sOutWordOne
      }
    }
  }
}

/**
 * LNIC Split classes.
 *
 * Tasks:
 *   - Combinationally split PISA pipeline output to network and core
 */
class SplitIO extends Bundle {
  val net_in = Flipped(Decoupled(new StreamChannel(LNICConsts.NET_DP_WIDTH)))
  val meta_in = Flipped(Valid(new PISAMetaIO))
  val net_out = Decoupled(new StreamChannel(LNICConsts.NET_DP_WIDTH))
  val core_out = Decoupled(new StreamChannel(LNICConsts.NET_DP_WIDTH))
  val core_meta_out = Valid(new PISAMetaIO)

  override def cloneType = new SplitIO().asInstanceOf[this.type]
}

@chiselName
class LNICSplit(implicit p: Parameters) extends Module {
  val io = IO(new SplitIO)

  // state machine to split output
  val sWordOne :: sWaitEnd :: Nil = Enum(2)
  val state = RegInit(sWordOne)

  val reg_egress_id = RegInit(false.B)

  io.core_out.bits := io.net_in.bits
  io.core_meta_out.bits := io.meta_in.bits
  io.net_out.bits := io.net_in.bits

  // default
  io.core_meta_out.valid := false.B

  switch (state) {
    is (sWordOne) {
      io.net_in.ready := Mux(io.meta_in.bits.egress_id, io.core_out.ready, io.net_out.ready)
      // connect core outputs
      io.core_out.valid := io.meta_in.bits.egress_id && io.net_in.valid
      io.core_meta_out.valid := io.core_out.valid
      // connect net outputs
      io.net_out.valid := !io.meta_in.bits.egress_id && io.net_in.valid
      when (io.net_in.valid && io.net_in.ready) {
        reg_egress_id := io.meta_in.bits.egress_id
        // next state logic
        when (!io.net_in.bits.last) {
          state := sWaitEnd
        }
      }
    }
    is (sWaitEnd) {
      io.net_in.ready := Mux(reg_egress_id, io.core_out.ready, io.net_out.ready)
      // connect core outputs
      io.core_out.valid := reg_egress_id && io.net_in.valid
      // connect net outputs
      io.net_out.valid := !reg_egress_id && io.net_in.valid
      when (io.net_in.valid && io.net_in.ready && io.net_in.bits.last) {
        state := sWordOne
      }
    }
  }
}

/**
 * LNIC Assemble classes.
 *
 * Tasks:
 *   - Reassemble pkts going to CPU into messages
 *   - Reverse byte order of data going to CPU
 *   - Enforce message length
 *   - For now, assume all msgs consist of a single pkt and there is only one thread running on the core
 */
class AssembleIO extends Bundle {
  val net_in = Flipped(Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH)))
  val meta_in = Flipped(Valid(new PISAMetaIO))
  val net_out = Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH))
  val meta_out = Valid(new AssembleMetaOut)

  override def cloneType = new AssembleIO().asInstanceOf[this.type]
}

class AssembleMetaOut extends Bundle {
  val lnic_dst = UInt(LNICConsts.LNIC_CONTEXT_BITS.W) // dst context ID

  override def cloneType = new AssembleMetaOut().asInstanceOf[this.type]
}

@chiselName
class LNICAssemble(implicit p: Parameters) extends Module {
  val io = IO(new AssembleIO)

  val pktQueue_in = Wire(Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH)))
  val metaQueue_in = Wire(Decoupled(new PISAMetaIO))
  val metaQueue_out = Wire(Flipped(Decoupled(new PISAMetaIO)))

  io.net_out <> Queue(pktQueue_in, p(LNICKey).assemblePktBufFlits)
  io.net_in.ready := pktQueue_in.ready
  metaQueue_out <> Queue(metaQueue_in, p(LNICKey).assembleMetaBufFlits)
  metaQueue_in.valid := false.B
  metaQueue_in.bits := io.meta_in.bits

  // default - connect net_in to pktQueue
  pktQueue_in <> io.net_in
  // reverse byte order of data going to CPU
  pktQueue_in.bits.data := reverse_bytes(io.net_in.bits.data, LNICConsts.NET_IF_BYTES)

  // state machine to compute message length using pktQueue_in
  val sStart :: sEnd :: Nil = Enum(2)
  val stateMsgLen = RegInit(sStart)

  val reg_msg_len = RegInit(0.U)

  switch (stateMsgLen) {
    is (sStart) {
      when (pktQueue_in.valid && pktQueue_in.ready) {
        reg_msg_len := io.meta_in.bits.msg_len
        metaQueue_in.valid := true.B
        assert (metaQueue_in.ready, "LNICAssemble: metaQueue is full!")
        when (!pktQueue_in.bits.last) {
          stateMsgLen := sEnd
        }
      }
    }
    is (sEnd) {
      when (pktQueue_in.valid && pktQueue_in.ready) {
        reg_msg_len := reg_msg_len - PopCount(pktQueue_in.bits.keep)
        when (pktQueue_in.bits.last) {
          stateMsgLen := sStart
        }
      }
    }
  }

  // state machine to enforce message length
  // NOTE: this will not work properly for single word messages (i.e. just app header)
  val sIdle :: sTruncate :: sPad :: Nil = Enum(3)
  val state = RegInit(sIdle)

  switch (state) {
    is (sIdle) {
      when (io.net_in.valid && io.net_in.ready && stateMsgLen =/= sStart) {
        when (reg_msg_len > LNICConsts.NET_IF_BYTES.U && io.net_in.bits.last) {
          // msg is too short - need to pad it
          state := sPad
          pktQueue_in.bits.last := false.B
          pktQueue_in.bits.keep := LNICConsts.NET_IF_FULL_KEEP
        } .elsewhen (reg_msg_len <= LNICConsts.NET_IF_BYTES.U && !io.net_in.bits.last) {
          // msg is too long - need to truncate it
          state := sTruncate
          pktQueue_in.bits.last := true.B
        }
      }
    }
    is (sTruncate) {
      pktQueue_in.valid := false.B
      when (io.net_in.valid && io.net_in.ready && io.net_in.bits.last) {
        state := sIdle
      }
    }
    is (sPad) {
      pktQueue_in.valid := true.B
      pktQueue_in.bits.data := 0.U
      when (pktQueue_in.ready && reg_msg_len <= LNICConsts.NET_IF_BYTES.U) {
        state := sIdle
        pktQueue_in.bits.last := true.B
        pktQueue_in.bits.keep := (1.U << reg_msg_len) - 1.U
      }
    }
  }

  // state machine to drive io.meta_out and read metaQueue
  val stateMetaOut = RegInit(sStart)

  switch (stateMetaOut) {
    is (sStart) {
        // io.meta_out is valid on the first word
        io.meta_out.valid := metaQueue_out.valid
        io.meta_out.bits.lnic_dst := metaQueue_out.bits.lnic_dst
        when (io.net_out.valid && io.net_out.ready) {
          metaQueue_out.ready := io.net_out.bits.last
          when (!io.net_out.bits.last) {
            stateMetaOut := sEnd
          }
        }
    }
    is (sEnd) {
      when (io.net_out.valid && io.net_out.ready && io.net_out.bits.last) {
        metaQueue_out.ready := true.B
        stateMetaOut := sStart
      }
    }
  }

}


/**
 * LNIC Per-Context FIFO queues and interrupt generation.
 *
 * Tasks:
 *   - Receive messages from the reassembly buffer.
 *   - The messages indicate which context they are for.
 *   - If the corresponding FIFO is full then drop the message.
 *   - Otherwise, write the message into the FIFO by reading from the free list and adding to the appropriate linked-list.
 *   - If the message is for a context that has a higher priority than the current_priority input signal (0 is highest priority)
 *     then generate an interrupt and update top_context and top_priority output signals
 *   - Dequeue words from the FIFO indicated by the current_context input signal
 *     - Remember that last 2 words that were dequeued for this context in case we need to roll back on pipeline flush
 *   - If the current_context_idle signal is asserted and top_context =/= current_context then generate an interrupt
 *   - Insert / remove the current context when told to do so
 */
class LNICRxQueuesIO(val entries: Int) extends Bundle {
  val net_in = Flipped(Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH)))
  val meta_in = Flipped(Valid(new NetToCoreMeta))
  val net_out = Decoupled(UInt(width = 64)) // words to the CPU

  val cur_context = Input(UInt(width = LNICConsts.LNIC_CONTEXT_BITS))
  val cur_priority = Input(UInt(width = LNICConsts.LNIC_PRIORITY_BITS))
  val insert = Input(Bool())
  // val remove = Input(Bool()) // we won't support remove for now since it would involve clearing the FIFO
  val idle = Input(Bool())

  val top_context = Output(UInt(width = LNICConsts.LNIC_CONTEXT_BITS))
  val top_priority = Output(UInt(width = LNICConsts.LNIC_PRIORITY_BITS))
  val interrupt = Output(Bool())

  // number of words for the current context
  val count = Output(UInt(log2Ceil(entries + 1).W))
  val unread = Flipped(Valid(UInt(width = 2)))

  override def cloneType = new LNICRxQueuesIO(entries).asInstanceOf[this.type]
}

/**
 * Rx Queues must support 2 word roll-back for pipeline flushes.
 */
class RxHeadTableEntry(ptrBits: Int) extends HeadTableEntry(ptrBits) {
  val head_minus1 = UInt(width = ptrBits)
  val head_minus2 = UInt(width = ptrBits)

  override def cloneType = new RxHeadTableEntry(ptrBits).asInstanceOf[this.type]
}

@chiselName
class LNICRxQueues(implicit p: Parameters) extends Module {
  val entries = p(LNICKey).rxBufFlits
  val num_contexts = p(LNICKey).maxNumContexts

  val io = IO(new LNICRxQueuesIO(entries))

  // divide the buffer space equally amongst contexts
  // NOTE: this may change in the future, but it's easy for now
  val max_qsize = entries/num_contexts

  val ptrBits = log2Ceil(entries)
  // create memory used to store msg words
  val ram = Mem(entries, new FIFOWord(ptrBits))
  // create free list for msg words
  val freelist = Module(new FreeList(entries))
  // create tables for indexing head/tail of FIFOs in the RAM
  // TODO(sibanez): currently just using context_id to directly index the tables.
  //   These should eventually turn into propper d-left lookup tables
  val head_table = Mem(num_contexts, new RxHeadTableEntry(ptrBits))
  val tail_table = Mem(num_contexts, new TailTableEntry(ptrBits))

  val cur_head_entry = Wire(new RxHeadTableEntry(ptrBits))
  val new_head_entry = Wire(new RxHeadTableEntry(ptrBits))
  val cur_tail_entry = Wire(new TailTableEntry(ptrBits))
  val new_tail_entry = Wire(new TailTableEntry(ptrBits))

  // create regs to store count for each context
  val counts = RegInit(VecInit(Seq.fill(num_contexts)(0.U(log2Ceil(max_qsize + 1).W))))
  val priorities = RegInit(VecInit(Seq.fill(num_contexts)(0.U(LNICConsts.LNIC_PRIORITY_BITS.W))))

  val cur_index = io.cur_context
  val enq_index = Wire(UInt())
  enq_index := io.meta_in.bits.lnic_dst // default

  val do_enq = Wire(Bool())
  val do_deq = Wire(Bool())
  do_enq := false.B
  do_deq := false.B

  // defaults
  // do not enq or deq from the free list
  freelist.io.enq.valid := false.B
  freelist.io.enq.bits := 0.U
  freelist.io.deq.ready := false.B
  // don't receive incomming messages
  io.net_in.ready := false.B

  io.count := counts(io.cur_context)

  io.net_out.valid := false.B
  io.net_out.bits := 0.U

  // Logic to insert new contexts.
  //   when insert is asserted, insert the cur_context into the head/tail_tables
  //   and set count to 0
  when (io.insert) {
    // do not perform an enqueue because we need to update tail_table
    io.net_in.ready := false.B
    // update head_table, try to read from free list
    freelist.io.deq.ready := true.B
    assert (freelist.io.deq.valid, "LNICRxQueues: freelist is empty during context insertion!")
    new_head_entry.valid := freelist.io.deq.valid
    new_head_entry.head := freelist.io.deq.bits
    new_head_entry.head_minus1 := freelist.io.deq.bits
    new_head_entry.head_minus2 := freelist.io.deq.bits
    head_table(cur_index) := new_head_entry
    // update tail_table
    new_tail_entry.valid := freelist.io.deq.valid
    new_tail_entry.tail := freelist.io.deq.bits
    tail_table(cur_index) := new_tail_entry
    // set priority
    priorities(cur_index) := io.cur_priority
    // initialize count
    counts(cur_index) := 0.U
  } .otherwise {

    // State machine to perform enqueue & drop operations
    // Drop message if:
    //   (1) the FIFO does not have enough space
    //   (2) the context ID is unallocated // TODO(sibanez): unused at the moment ...
    val sStart :: sEnqueue :: sDrop :: Nil = Enum(3)
    val enqState = RegInit(sStart)

    val reg_enq_index = RegInit(0.U(LNICConsts.LNIC_CONTEXT_BITS.W))

    // this module should not assert back pressure (except during context insertion)
    io.net_in.ready := true.B

    switch(enqState) {
      is (sStart) {
        when (io.net_in.valid) {
          val msg_len = io.net_in.bits.data(15, 0)
          val msg_words = (msg_len >> 3.U) + 1.U // NOTE: this is assuming 8B words
          when (msg_words < (max_qsize.U - counts(enq_index))) {
            // there is enough room, enqueue the msg
            perform_enq(enq_index)
            reg_enq_index := enq_index
            enqState := sEnqueue
          } .otherwise {
            // there is NOT enough room, drop the msg
            enqState := sDrop
          }
        }
      }
      is (sEnqueue) {
        when (io.net_in.valid) {
          enq_index := reg_enq_index
          perform_enq(enq_index)
          when (io.net_in.bits.last) {
            enqState := sStart
          }
        }
      }
      is (sDrop) {
        when (io.net_in.valid && io.net_in.bits.last) {
          enqState := sStart
        }
      }
    }

    // Logic to perform dequeue & unread operations.
    // Only add a ptr to the free list once head_minus2 no longer needs it.
    io.net_out.valid := (counts(cur_index) > 0.U)

    val unread_cnt = Wire(UInt())
    unread_cnt := 0.U

    // TODO(sibanez): what if there is an unread at the same time as an insert? Is that possible?
    when (io.unread.valid) {
      when (io.unread.bits.andR) { // roll back 2 words
        unread_cnt := 2.U
        cur_head_entry := head_table(cur_index)
        new_head_entry := cur_head_entry // default
        new_head_entry.head := cur_head_entry.head_minus2
        new_head_entry.head_minus1 := cur_head_entry.head_minus2
        head_table(cur_index) := new_head_entry
      } .elsewhen (io.unread.bits.orR) { // roll back 1 word
        unread_cnt := 1.U
        cur_head_entry := head_table(cur_index)
        new_head_entry := cur_head_entry // default
        new_head_entry.head := cur_head_entry.head_minus1
        head_table(cur_index) := new_head_entry
      }
    } .elsewhen (io.net_out.valid && io.net_out.ready) {
      // perform dequeue
      perform_deq(cur_index)
    }

    // Logic to count the size of each FIFO, updated on enq/deq/unread operations
    for (i <- 0 until counts.size) {
      val enq_inc = Mux(do_enq && (enq_index === i.U), 1.U, 0.U)
      val deq_dec = Mux(do_deq && (cur_index === i.U), 1.U, 0.U)
      val unread_inc = Mux(io.unread.valid && (cur_index === i.U), unread_cnt, 0.U)
      counts(i.U) := counts(i.U) + enq_inc - deq_dec + unread_inc
    }
  }

  // Update top_context and top_priority every time a word is enqueued, dequeued, or there is an unread operation.

  val reg_top_priority = RegInit(0.U(LNICConsts.LNIC_PRIORITY_BITS.W))
  val reg_top_context = RegInit(0.U(LNICConsts.LNIC_CONTEXT_BITS.W))

  val reg_do_enq = Reg(next = do_enq)
  val reg_do_deq = Reg(next = do_deq)
  val reg_do_unread = Reg(next = io.unread.valid)

  // Update top_priority & top_context on enq/deq/unread operations
  // Do this on the cycle after enq/deq/unread so that counts is updated
  when (reg_do_enq || reg_do_deq || reg_do_unread) {
    // Collect all context IDs (indicies) with count > 0 then select the one with the highest priority

    // TODO(sibanez): implement this ...
    val context_ids = Wire(Vec(num_contexts, UInt(width = LNICConsts.LNIC_CONTEXT_BITS)))
    for (i <- 0 until context_ids.size) {
      context_ids(i) := i.U
    }
    val result = priorities.zip(context_ids).reduce( (tuple1, tuple2) => {
      // tuple._1 = priority, tuple._2 = index (context)
      val top_prio = Wire(UInt())
      val top_context = Wire(UInt())
      when (counts(tuple1._2) > 0.U && counts(tuple2._2) > 0.U) {
        when (tuple1._1 < tuple2._1) {
          top_prio := tuple1._1
          top_context := tuple1._2
        } .otherwise {
          top_prio := tuple2._1
          top_context := tuple2._2
        }
      } .elsewhen (counts(tuple1._2) > 0.U) {
        top_prio := tuple1._1
        top_context := tuple1._2
      } .elsewhen (counts(tuple2._2) > 0.U) {
        top_prio := tuple2._1
        top_context := tuple2._2
      } .otherwise {
        top_prio := io.cur_priority
        top_context := io.cur_context
      }
      (top_prio, top_context)
    })

    reg_top_priority := result._1
    reg_top_context := result._2
  }

  io.top_priority := reg_top_priority
  io.top_context := reg_top_context

  // Generate an interrupt whenever:
  //   (1) an enqueue operation causes top_context =/= cur_context
  //   (2) io.idle is asserted and top_context =/= cur_context

  // default - do not fire interrupt
  io.interrupt := false.B

  val reg_reg_do_enq = Reg(next = reg_do_enq)
  val reg_idle = Reg(next = io.idle) // register io.idle to break combinational loop
  when (reg_reg_do_enq && (reg_top_context =/= io.cur_context)) {
    io.interrupt := true.B
  } .elsewhen (reg_idle && (reg_top_context =/= io.cur_context)) {
    io.interrupt := true.B
  }

  def perform_enq(index: UInt) = {
    do_enq := true.B
    // lookup current tail_ptr for this context
    cur_tail_entry := tail_table(index)
    val tail_ptr = cur_tail_entry.tail
    assert (cur_tail_entry.valid, "Attempting to perform an enqueue for an invalid contextID")

    // read free list
    val tail_ptr_next = freelist.io.deq.bits
    freelist.io.deq.ready := true.B
    assert (freelist.io.deq.valid, "Free list is empty during msg enqueue!")

    // write new word to RAM
    val enq_fifo_word = Wire(new FIFOWord(ptrBits))
    enq_fifo_word.word := io.net_in.bits.data
    enq_fifo_word.next := tail_ptr_next
    ram(tail_ptr) := enq_fifo_word

    // update tail ptr
    new_tail_entry := cur_tail_entry // default
    new_tail_entry.tail := tail_ptr_next
    tail_table(index) := new_tail_entry
  }

  def perform_deq(index: UInt) = {
    do_deq := true.B
    // lookup the current head of the FIFO for this context
    cur_head_entry := head_table(index)
    val head_ptr = cur_head_entry.head
    assert (cur_head_entry.valid, "Attempting to perform dequeue for an invalid contextID")

    // read the head word from the RAM
    val deq_fifo_word = Wire(new FIFOWord(ptrBits))
    deq_fifo_word := ram(head_ptr)
    io.net_out.bits := deq_fifo_word.word
    val head_ptr_next = deq_fifo_word.next

    // update head table
    new_head_entry := cur_head_entry // default
    new_head_entry.head := head_ptr_next
    when (cur_head_entry.head_minus1 =/= head_ptr) {
      new_head_entry.head_minus1 := head_ptr
      when (cur_head_entry.head_minus2 =/= cur_head_entry.head_minus1) {
        new_head_entry.head_minus2 := cur_head_entry.head_minus1
        // add head_minus2 to the free list
        freelist.io.enq.valid := true.B
        freelist.io.enq.bits := cur_head_entry.head_minus2
        assert (freelist.io.enq.ready, "Free list is full during dequeue!")
      }
    }
    head_table(index) := new_head_entry
  }

}

class FreeList(val entries: Int) extends Module {
  val io = IO(new Bundle {
    // enq ptrs into this module
    val enq = Flipped(Decoupled(UInt(width = log2Ceil(entries))))
    // deq ptrs from this module
    val deq = Decoupled(UInt(width = log2Ceil(entries)))
  })

  val ptrBits = log2Ceil(entries)

  // create FIFO queue to store pointers
  val queue_in = Wire(Decoupled(UInt(width = ptrBits)))
  val queue_out = Queue(queue_in, entries)

  val sReset :: sIdle :: Nil = Enum(2)
  val state = RegInit(sReset)

  val reg_ptr = RegInit(0.U(ptrBits.W))

  // default - connect queue to enq/deq interfaces
  queue_in <> io.enq
  io.deq <> queue_out

  switch(state) {
    is (sReset) {
      // disconnect queue from enq/deq interfaces
      io.enq.ready := false.B
      io.deq.valid := false.B
      queue_out.ready := false.B
      // insert 0 - (entries-1) into the queue on reset
      queue_in.valid := true.B
      queue_in.bits := reg_ptr
      when (queue_in.ready) {
        reg_ptr := reg_ptr + 1.U
      }
      when (reg_ptr === (entries - 1).U) {
        state := sIdle
      }
    }
    is (sIdle) {
      queue_in <> io.enq
      io.deq <> queue_out
    }
  }

}

