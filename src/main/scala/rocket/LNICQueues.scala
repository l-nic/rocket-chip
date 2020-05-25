
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
 *   - If the current_context_idle signal is asserted and top_context =/= current_context then generate an interrupt
 *   - Insert / remove the current context when told to do so
 */
class LNICRxQueuesIO(val num_entries: Int) extends Bundle {
  val net_in = Flipped(Decoupled(new StreamChannel(XLEN)))
  val meta_in = Flipped(Valid(new LNICRxMsgMeta))
  val net_out = Decoupled(UInt(width = XLEN)) // words to the CPU

  val cur_context = Input(UInt(width = LNIC_CONTEXT_BITS))
  val cur_priority = Input(UInt(width = LNIC_PRIORITY_BITS))
  val insert = Input(Bool())
  // val remove = Input(Bool()) // we won't support remove for now since it would involve clearing the FIFO
  val idle = Input(Bool())

  val top_context = Output(UInt(width = LNIC_CONTEXT_BITS))
  val top_priority = Output(UInt(width = LNIC_PRIORITY_BITS))
  val interrupt = Output(Bool())

  // number of words for the current context
  val count = Output(UInt(log2Ceil(num_entries + 1).W))
  val unread = Flipped(Valid(UInt(width = 2)))

  override def cloneType = new LNICRxQueuesIO(num_entries).asInstanceOf[this.type]
}

class FIFOWord(val ptrBits: Int) extends Bundle {
  val word = UInt(width = XLEN)
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
  val num_entries = (MAX_MSG_SIZE_BYTES/XBYTES)*num_contexts

  val io = IO(new LNICRxQueuesIO(num_entries))

  // divide the buffer space equally amongst contexts
  // NOTE: this may change in the future, but it's easy for now
  val max_qsize = num_entries/num_contexts

  val ptrBits = log2Ceil(num_entries)
  // create memory used to store msg words
  val ram = Mem(num_entries, new FIFOWord(ptrBits))
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

  // create regs to store count for each context
  val counts = RegInit(VecInit(Seq.fill(num_contexts)(0.U(log2Ceil(max_qsize + 1).W))))
  val priorities = RegInit(VecInit(Seq.fill(num_contexts)(0.U(LNIC_PRIORITY_BITS.W))))

  val cur_index = io.cur_context
  val enq_index = Wire(UInt())
  enq_index := io.meta_in.bits.dst_context // default

  val do_enq = Wire(Bool())
  val do_deq = Wire(Bool())
  do_enq := false.B
  do_deq := false.B

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

    val reg_enq_index = RegInit(0.U(LNIC_CONTEXT_BITS.W))

    // assert backpressure if the current context is consuming too much space
    io.net_in.ready := counts(enq_index) < max_qsize.U

    switch(enqState) {
      is (sStart) {
        when (io.net_in.valid && io.net_in.ready) {
          // there is enough room, enqueue the msg
          perform_enq(enq_index)
          reg_enq_index := enq_index
          enqState := sEnqueue
        }
      }
      is (sEnqueue) {
        enq_index := reg_enq_index
        when (io.net_in.valid && io.net_in.ready) {
          perform_enq(enq_index)
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

  // Update top_context and top_priority every time a word is enqueued, dequeued, or there is an unread operation.

  val reg_top_priority = RegInit(0.U(LNIC_PRIORITY_BITS.W))
  val reg_top_context = RegInit(0.U(LNIC_CONTEXT_BITS.W))

  val reg_do_enq = Reg(next = do_enq)
  val reg_do_deq = Reg(next = do_deq)
  val reg_do_unread = Reg(next = io.unread.valid)

  // Update top_priority & top_context on enq/deq/unread operations
  // Do this on the cycle after enq/deq/unread so that counts is updated
  when (reg_do_enq || reg_do_deq || reg_do_unread) {
    // Collect all context IDs (indicies) with count > 0 then select the one with the highest priority

    val context_ids = Wire(Vec(num_contexts, UInt(width = LNIC_CONTEXT_BITS)))
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

