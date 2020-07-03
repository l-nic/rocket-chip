
package freechips.rocketchip.rocket

import Chisel._

import chisel3.{VecInit}
import chisel3.experimental._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.tile._
import LNICRocketConsts._

// the first word of every msg sent by application
class TxAppHdr extends Bundle {
  val dst_ip = UInt(32.W)
  val dst_context = UInt(LNIC_CONTEXT_BITS.W)
  val msg_len = UInt(MSG_LEN_BITS.W)
}

/**
 * The Core's IO to the L-NIC module.
 */
class CoreLNICIO extends Bundle {
  val net_in = Flipped(Decoupled(new StreamChannel(XLEN)))
  val meta_in = Flipped(Valid(new LNICRxMsgMeta))
  val net_out = Decoupled(new LNICTxMsgWord)
  // Out-of-band coordination with NIC for load balancing
  val add_context = Valid(UInt(LNIC_CONTEXT_BITS.W))
  val get_next_msg = Valid(UInt(LNIC_CONTEXT_BITS.W))
}

class LNICTxMsgWord extends Bundle {
  val data = UInt(XLEN.W)
  val src_context = UInt(LNIC_CONTEXT_BITS.W)
}

class LNICRxMsgMeta extends Bundle {
  val dst_context = UInt(LNIC_CONTEXT_BITS.W)
}

/* LNIC TX Queue:
 * Lives in the Rocket Core CSR File.
 *
 * Tasks:
 *   - Store and schedule msg words for delivery to the LNIC
 */
class LNICTxQueueIO extends Bundle {
  val net_in = Flipped(Valid(UInt(XLEN.W))) // words written from the CPU
  val net_out = Decoupled(new LNICTxMsgWord)

  val cur_context = Input(UInt(width = LNIC_CONTEXT_BITS))
  // TODO(sibanez): no need to insert/remove contexts for now
  // val insert = Input(Bool())
  // val remove = Input(Bool())
}

@chiselName
class LNICTxQueue(implicit p: Parameters) extends Module {
  val num_contexts = p(LNICRocketKey).get.maxNumContexts

  val io = IO(new LNICTxQueueIO)

  // find max msg buffer size (in terms of 8B words) 
  val max_msg_words = MAX_MSG_SIZE_BYTES/XBYTES

  val tx_queue_enq = Wire(Decoupled(new LNICTxMsgWord))
  io.net_out <> Queue(tx_queue_enq, max_msg_words*num_contexts)

  /* Enqueue state machine
   *   - Check if entire msg fits in the tx queue before writing the first word
   */
  val sStartEnq :: sFinishEnq :: Nil = Enum(2)
  // need one state per context
  val enqStates = RegInit(VecInit(Seq.fill(num_contexts)(sStartEnq)))

  val rem_bytes_reg = RegInit(VecInit(Seq.fill(num_contexts)(0.U(MSG_LEN_BITS.W))))

  // defaults
  tx_queue_enq.valid := io.net_in.valid
  tx_queue_enq.bits.data := io.net_in.bits
  tx_queue_enq.bits.src_context := io.cur_context

  switch (enqStates(io.cur_context)) {
    is (sStartEnq) {
      when (io.net_in.valid) {
        assert (tx_queue_enq.ready, "tx_queue is full during enqueue!")
        // record msg len
        val tx_app_hdr = (new TxAppHdr).fromBits(io.net_in.bits)
        rem_bytes_reg(io.cur_context) := tx_app_hdr.msg_len
        assert (tx_app_hdr.msg_len > 0.U, "Cannot send a 0 length msg!")
        // TODO(sibanez): if there is not enough room to store the whole msg then fire an interrupt
        // state transition
        enqStates(io.cur_context) := sFinishEnq
      }
    }
    is (sFinishEnq) {
      when (io.net_in.valid) {
        assert (tx_queue_enq.ready, "tx_queue is full during enqueue!")
        when (rem_bytes_reg(io.cur_context) <= XBYTES.U) {
          // last word
          enqStates(io.cur_context) := sStartEnq
        }
        // update remaining bytes for the msg
        rem_bytes_reg(io.cur_context) := rem_bytes_reg(io.cur_context) - XBYTES.U
      }
    }
  }

}

/**
 * LNIC Per-Context RX FIFO queues and interrupt generation.
 *
 * Tasks:
 *   - Receive messages from the reassembly buffer.
 *   - The messages indicate which context they are for.
 *   - If the corresponding FIFO is full then exert back-pressure.
 *   - Otherwise, write the message into the FIFO by reading from the free list and adding to the appropriate linked-list.
 *   - If the message is for a context that has a higher priority than the current_priority input signal (0 is highest priority)
 *     then generate an interrupt and update top_context and top_priority output signals
 *   - Dequeue words from the FIFO indicated by the current_context input signal
 *     - Remember that last 2 words that were dequeued for this context in case we need to roll back on pipeline flush
 *   - msg_done signal is asserted when the current thread finishes processing a msg.
 *   - msgs for threads at the same priority are processed in FIFO order
 *   - Priority 0 is for gauanteed service apps. This module will track msg processing time for these apps.
 *     - If the msg processing time exceeds Xus (e.g. 1us) then this module will reduce priority and schedule a higher
 *       priority active thread if there is one. 
 */
class LNICRxQueuesIO(val num_entries: Int) extends Bundle {
  val net_in = Flipped(Decoupled(new StreamChannel(XLEN)))
  val meta_in = Flipped(Valid(new LNICRxMsgMeta))
  val net_out = Decoupled(UInt(width = XLEN)) // words to the CPU

  val cur_context = Input(UInt(width = LNIC_CONTEXT_BITS))
  val cur_priority = Input(UInt(width = LNIC_PRIORITY_BITS))
  val insert = Input(Bool())
  // val remove = Input(Bool()) // we won't support remove for now since it would involve clearing the FIFO
  val start_timer = Input(Bool()) // asserted at the end of a context switch

  // application indicates that msg processing is complete
  val msg_done = Input(Bool())
  // app indicates that it is idle (set during polling loop)
  val idle = Input(Bool())

  // top_context and top_priority correspond to the highest priority active context and its priority, respectively
  val top_context = Output(UInt(width = LNIC_CONTEXT_BITS))
  val top_priority = Output(UInt(width = LNIC_PRIORITY_BITS))
  val interrupt = Output(Bool())

  // number of words for the current context
  val count = Output(UInt(log2Ceil(num_entries + 1).W))
  val unread = Flipped(Valid(UInt(width = 2)))

  // Indicate that the NIC should transmit another msg to this core
  val get_next_msg = Valid(UInt(width = LNIC_CONTEXT_BITS))

  override def cloneType = new LNICRxQueuesIO(num_entries).asInstanceOf[this.type]
}

class TsFIFOWord(val wordBits: Int, val ptrBits: Int) extends Bundle {
  val word = UInt(width = wordBits)
  val next = UInt(width = ptrBits)
  val timestamp = UInt(64.W) // time at which the first word of the msg arrived, measured in cycles

  override def cloneType = new TsFIFOWord(wordBits, ptrBits).asInstanceOf[this.type]
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
  val num_contexts = p(LNICRocketKey).get.maxNumContexts
  val num_entries = (MAX_MSG_SIZE_BYTES/XBYTES)*num_contexts*MAX_OUTSTANDING_MSGS

  val io = IO(new LNICRxQueuesIO(num_entries))

  // divide the buffer space equally amongst contexts
  // NOTE: this may change in the future, but it's easy for now
  val max_qsize = num_entries/num_contexts

  val ptrBits = log2Ceil(num_entries)
  // create memory used to store msg words
  val ram = Mem(num_entries, new TsFIFOWord(XLEN, ptrBits))
  // create free list for msg words
  val entries = for (i <- 0 until num_entries) yield i.U(log2Up(num_entries).W)
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

  // create regs to store state for each context
  val counts = RegInit(VecInit(Seq.fill(num_contexts)(0.U(log2Ceil(max_qsize + 1).W))))
  val priorities = RegInit(VecInit(Seq.fill(num_contexts)(0.U(LNIC_PRIORITY_BITS.W))))
  // each context is either idle of active.
  //   active means the context is either currently processing a msg or has msgs to process
  val sIdle :: sActive :: Nil = Enum(2)
  val ctx_state = RegInit(VecInit(Seq.fill(num_contexts)(sIdle)))
  // regs to store the timestamp of the head msg for each context (updated on msg_done)
  val ctx_timestamp = RegInit(VecInit(Seq.fill(num_contexts)(0.U(64.W))))

  // timer used to timestamp enqueued msgs
  val enq_timer = RegInit(0.U(64.W))
  enq_timer := enq_timer + 1.U

  val cur_index = io.cur_context
  val enq_index = Wire(UInt())
  enq_index := io.meta_in.bits.dst_context // default

  val do_enq = Wire(Bool())
  val do_deq = Wire(Bool())
  do_enq := false.B
  do_deq := false.B

  val get_next_msg = Reg(Valid(UInt(width = LNIC_CONTEXT_BITS)))
  when (reset.toBool) {
    get_next_msg.valid := false.B
    get_next_msg.bits := 0.U
  } .otherwise {
    get_next_msg.valid := io.msg_done
    get_next_msg.bits := io.cur_context
  }
  io.get_next_msg := get_next_msg

  // defaults
  // do not enq or deq from the free list
  freelist.io.enq.valid := false.B
  freelist.io.enq.bits := 0.U
  freelist.io.deq.ready := false.B

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

    // State machine to perform enqueue operations
    val sStart :: sEnqueue :: Nil = Enum(2)
    val enqState = RegInit(sStart)
    val enq_timestamp = RegInit(0.U(64.W))

    val reg_enq_index = RegInit(0.U(LNIC_CONTEXT_BITS.W))

    // assert backpressure if the current context is consuming too much space
    io.net_in.ready := counts(enq_index) < max_qsize.U

    switch(enqState) {
      is (sStart) {
        when (io.net_in.valid && io.net_in.ready) {
          // there is enough room, enqueue the msg
          perform_enq(enq_index, enq_timer)
          enq_timestamp := enq_timer
          reg_enq_index := enq_index
          enqState := sEnqueue
        }
      }
      is (sEnqueue) {
        enq_index := reg_enq_index
        when (io.net_in.valid && io.net_in.ready) {
          perform_enq(enq_index, enq_timestamp)
          when (io.net_in.bits.last) {
            enqState := sStart
          }
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

  // Update context state:
  //   - Transition to idle when msg_done or idle is asserted and there are no msgs for the context
  //   - Transition to active when a msg arrives (may already be active)
  when ( (io.msg_done || io.idle) && counts(cur_index) === 0.U) {
    ctx_state(cur_index) := sIdle
  }
  when (do_enq) {
    ctx_state(enq_index) := sActive
  }
  
  // Update top_context and top_priority
  // Compute the highest priority active context with the smallest msg timestamp

  val reg_top_priority = RegInit(0.U(LNIC_PRIORITY_BITS.W))
  val reg_top_context = RegInit(0.U(LNIC_CONTEXT_BITS.W))

  // Wire up timestamp of the head msg for each context
  // NOTE: this forces the tables / queues to be mapped to registers. Hopefully this still meets timing.
  val context_timestamps = Wire(Vec(num_contexts, UInt(64.W)))
  for (i <- 0 until num_contexts) {
    val head = Wire(new RxHeadTableEntry(ptrBits))
    head := head_table(i)
    // read the head word from the RAM
    val fifo_word = Wire(new TsFIFOWord(XLEN, ptrBits))
    fifo_word := ram(head.head)
    // NOTE: if the queue for this context is empty then use max value for timestamp
    context_timestamps(i) := Mux(counts(i) > 0.U, fifo_word.timestamp, ~0.U(64.W))
  }

  val context_ids = Wire(Vec(num_contexts, UInt(width = LNIC_CONTEXT_BITS)))
  for (i <- 0 until context_ids.size) {
    context_ids(i) := i.U
  }
  val result = priorities.zip(context_ids).reduce( (tuple1, tuple2) => {
    // tuple._1 = priority, tuple._2 = index (context)
    val prio1 = tuple1._1
    val ctx1 = tuple1._2
    val prio2 = tuple2._1
    val ctx2 = tuple2._2
    val top_prio = Wire(UInt())
    val top_context = Wire(UInt())
    when (ctx_state(ctx1) === sActive && ctx_state(ctx2) === sActive) {
      when (prio1 < prio2) {
        top_prio := prio1
        top_context := ctx1
      } .elsewhen(prio2 < prio1) {
        top_prio := prio2
        top_context := ctx2
      } .otherwise {
        // Break ties using timestamp of head msg
        when (context_timestamps(ctx1) < context_timestamps(ctx2)) {
          top_prio := prio1
          top_context := ctx1
        } .otherwise {
          top_prio := prio2
          top_context := ctx2
        }
      }
    } .elsewhen (ctx_state(ctx1) === sActive) {
      top_prio := prio1
      top_context := ctx1
    } .elsewhen (ctx_state(ctx2) === sActive) {
      top_prio := prio2
      top_context := ctx2
    } .otherwise {
      top_prio := io.cur_priority
      top_context := io.cur_context
    }
    (top_prio, top_context)
  })

  reg_top_priority := result._1
  reg_top_context := result._2

  io.top_priority := reg_top_priority
  io.top_context := reg_top_context

  // Timer to track msg processing time
  val msg_timer = RegInit(0.U(64.W))

  // Timer is restarted whenever:
  //   (1) application asserts msg_done - in a real implementation, this module should
  //       check to make sure a msg is actually processed before asserting msg_done
  //   (2) nanokernel asserts start_timer at the end of a context switch
  when (io.msg_done || io.idle || io.start_timer) {
    msg_timer := 0.U
  } .otherwise {
    msg_timer := msg_timer + 1.U
  } 

  // lower priority of the current context if it exceeds msg processing time
  val priority_lowered = Wire(Bool())
  priority_lowered := false.B // default
  when ((msg_timer >= MSG_PROC_MAX_CYCLES.U) && (io.cur_priority === 0.U)) {
    priority_lowered := true.B
    priorities(io.cur_context) := 1.U
  }

  // Generate an interrupt whenever:
  //   (1) an enqueue operation causes top_priority =/= cur_priority -- priority constraint
  //   (2) msg_done or idle is asserted and top_context =/= cur_context -- FIFO ordering constraint & work conserving constraint
  //   (3) msg_timer expires, cur_priority === 0, cur_context is active (do I need this condition?),
  //       and top_context =/= cur_context (also lower priority of cur_context) -- bounded processing time constraint

  // default - do not fire interrupt
  io.interrupt := false.B

  val reg_do_enq = Reg(next = do_enq)
  val reg_reg_do_enq = Reg(next = reg_do_enq) // wait for do_enq to update reg_top_context and reg_top_priority
  val check = Wire(Bool())
  check := io.msg_done || io.idle
  val reg_check = Reg(next = check)
  val reg_reg_check = Reg(next = reg_check) // wait for msg_done/idle to update reg_top_context and reg_top_priority
  val reg_priority_lowered = Reg(next = priority_lowered)
  when (reg_reg_do_enq && (reg_top_priority < io.cur_priority)) {
    assert(io.cur_context =/= reg_top_context, "Priorities don't match, but context IDs do?")
    io.interrupt := true.B
  } .elsewhen (reg_reg_check && (reg_top_context =/= io.cur_context)) {
    io.interrupt := true.B
  } .elsewhen (reg_priority_lowered && (reg_top_context =/= io.cur_context)) {
    io.interrupt := true.B
    msg_timer := 0.U // reset timer
  }

  def perform_enq(index: UInt, timestamp: UInt) = {
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
    val enq_fifo_word = Wire(new TsFIFOWord(XLEN, ptrBits))
    enq_fifo_word.word := io.net_in.bits.data
    enq_fifo_word.timestamp := timestamp
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
    val deq_fifo_word = Wire(new TsFIFOWord(XLEN, ptrBits))
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

