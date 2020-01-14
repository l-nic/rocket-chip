
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
 */
class PktizeMetaOut extends Bundle {
  val msg_id = UInt(16.W)
  val offset = UInt(16.W) // pkt offset within msg
  val lnic_src = UInt(16.W) // src context ID

  override def cloneType = new PktizeMetaOut().asInstanceOf[this.type]
}

class PktizeIO extends Bundle {
  val net_in = Flipped(Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH)))
  val meta_in = Flipped(Valid (new CoreNetMeta))
  val net_out = Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH))
  val meta_out = Valid(new PktizeMetaOut)

  override def cloneType = new PktizeIO().asInstanceOf[this.type]
}

@chiselName
class LNICPktize(implicit p: Parameters) extends Module {
  val io = IO(new PktizeIO)

  val pktQueue_in = Wire(Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH)))
  pktQueue_in <> io.net_in
  // reverse byte order of words coming from CPU
  pktQueue_in.bits.data := reverse_bytes(io.net_in.bits.data, LNICConsts.NET_IF_BYTES)

  io.net_out <> Queue(pktQueue_in, p(LNICKey).pktizePktBufFlits)

  // state machine to drive io.meta_out
  val sWordOne :: sWaitEnd :: Nil = Enum(2)
  val state = RegInit(sWordOne)

  val reg_msg_id = RegInit(0.U(16.W))

  switch (state) {
    is (sWordOne) {
      when (io.net_out.valid && io.net_out.ready) {
        io.meta_out.valid := true.B
        // TODO(sibanez): these are set assuming single pkt messages
        io.meta_out.bits.msg_id := reg_msg_id
        io.meta_out.bits.offset := 0.U
        // TODO(sibanez): this should really be queued up and sent out, but thi hack should usually work
        io.meta_out.bits.lnic_src := io.meta_in.bits.context_id
        when (!io.net_out.bits.last) {
          state := sWaitEnd
        }
        reg_msg_id := reg_msg_id + 1.U
      }
    }
    is (sWaitEnd) {
      when (io.net_out.valid && io.net_out.ready && io.net_out.bits.last) {
        state := sWordOne
      }
    }
  }

}

/**
 * LNIC Arbiter classes
 */
class ArbiterIO extends Bundle {
  val core_in = Flipped(Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH)))
  val core_meta_in = Flipped(Valid(new PktizeMetaOut))
  val net_in = Flipped(Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH)))
  val net_out = Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH))
  val meta_out = Valid(new PISAMetaIn)

  override def cloneType = new ArbiterIO().asInstanceOf[this.type]
}

@chiselName
class LNICArbiter(implicit p: Parameters) extends Module {
  val io = IO(new ArbiterIO)

  val pktQueue_in = Wire(Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH)))
  val metaQueue_in = Wire(Decoupled(new PISAMetaIn))
  val metaQueue_out = Wire(Flipped(Decoupled(new PISAMetaIn)))

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
  val net_in = Flipped(Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH)))
  val meta_in = Flipped(Valid(new PISAMetaOut))
  val net_out = Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH))
  val core_out = Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH))
  val core_meta_out = Valid(new PISAMetaOut)

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
  val meta_in = Flipped(Valid(new PISAMetaOut))
  val net_out = Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH))
  val meta_out = Valid(new CoreNetMeta)

  override def cloneType = new AssembleIO().asInstanceOf[this.type]
}

@chiselName
class LNICAssemble(implicit p: Parameters) extends Module {
  val io = IO(new AssembleIO)

  val pktQueue_in = Wire(Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH)))
  val metaQueue_in = Wire(Decoupled(new PISAMetaOut))
  val metaQueue_out = Wire(Flipped(Decoupled(new PISAMetaOut)))

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
          pktQueue_in.bits.keep := LNICConsts.NET_FULL_KEEP
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
        io.meta_out.bits.context_id := metaQueue_out.bits.lnic_dst
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
  val meta_in = Flipped(Valid(new CoreNetMeta))
  val net_out = Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH))

  val cur_context = Input(UInt(width = LNICConsts.LNIC_CONTEXT_BITS))
  val insert = Input(Bool())
  // val remove = Input(Bool()) // we won't support remove for now since it would involve clearing the FIFO

  // number of words for the current context
  val count = Output(UInt(log2Ceil(entries + 1).W))
  val unread = Flipped(Valid(UInt(width = 2)))

  override def cloneType = new LNICRxQueuesIO(entries).asInstanceOf[this.type]
}

class FIFOWord(val ptrBits: Int) extends Bundle {
  val word = new StreamChannel(LNICConsts.NET_IF_WIDTH)
  val next = UInt(width = ptrBits)

  override def cloneType = new FIFOWord(ptrBits).asInstanceOf[this.type]
}

class HeadTableEntry(val ptrBits: Int) extends Bundle {
  val valid = Bool()
  val head = UInt(width = ptrBits)
  val head_minus1 = UInt(width = ptrBits)
  val head_minus2 = UInt(width = ptrBits)

  override def cloneType = new HeadTableEntry(ptrBits).asInstanceOf[this.type]
}

class TailTableEntry(val ptrBits: Int) extends Bundle {
  val valid = Bool()
  val tail = UInt(width = ptrBits)

  override def cloneType = new TailTableEntry(ptrBits).asInstanceOf[this.type]
}

@chiselName
class LNICRxQueues(val entries: Int, val num_contexts: Int)(implicit p: Parameters) extends Module {
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
  val head_table = Mem(num_contexts, new HeadTableEntry(ptrBits))
  val tail_table = Mem(num_contexts, new TailTableEntry(ptrBits))

  val cur_head_entry = Wire(new HeadTableEntry(ptrBits))
  val new_head_entry = Wire(new HeadTableEntry(ptrBits))
  val cur_tail_entry = Wire(new TailTableEntry(ptrBits))
  val new_tail_entry = Wire(new TailTableEntry(ptrBits))

  // create regs to store count for each context
  val counts = RegInit(VecInit(Seq.fill(num_contexts)(0.U(log2Ceil(max_qsize + 1).W))))

  val cur_index = io.cur_context
  val enq_index = Wire(UInt())
  enq_index := io.meta_in.bits.context_id // default

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
  io.net_out.bits.data := 0.U
  io.net_out.bits.keep := 0.U
  io.net_out.bits.last := false.B

  // Logic to insert new contexts.
  //   when insert is asserted, insert the cur_context into the head/tail_tables
  //   and set count to 0
  when (io.insert) {
    // do not perform an enqueue because we need to update tail_table
    io.net_in.ready := false.B
    // update head_table, try to read from free list
    freelist.io.deq.ready := true.B
    new_head_entry.valid := freelist.io.deq.valid
    new_head_entry.head := freelist.io.deq.bits
    new_head_entry.head_minus1 := freelist.io.deq.bits
    new_head_entry.head_minus2 := freelist.io.deq.bits
    head_table(cur_index) := new_head_entry
    // update tail_table
    new_tail_entry.valid := freelist.io.deq.valid
    new_tail_entry.tail := freelist.io.deq.bits
    tail_table(cur_index) := new_tail_entry
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

