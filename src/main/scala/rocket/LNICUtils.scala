
package freechips.rocketchip.rocket

import Chisel._

import chisel3.{chiselTypeOf}
import chisel3.experimental._
import freechips.rocketchip.config._

object LNICRocketConsts {
  val XLEN = 64
  val XBYTES = XLEN/8
  val LWRITE_ADDR = 31.U
  val LREAD_ADDR = 30.U
  val LNIC_CONTEXT_BITS = 16
  val LNIC_PRIORITY_BITS = 8
  val MSG_LEN_BITS = 16
  val MAX_NUM_CONTEXTS = 2
  val MAX_MSG_SIZE_BYTES = 8192
}

case class LNICRocketParams(
  maxNumContexts:  Int = LNICRocketConsts.MAX_NUM_CONTEXTS
)

case object LNICRocketKey extends Field[Option[LNICRocketParams]](None)

/**
 * NOTE: Copied StreamChannel and StreamIO here to avoid dependency on testchipip.
 */

class StreamChannel(val w: Int) extends Bundle {
  val data = UInt(w.W)
  val keep = UInt((w/8).W)
  val last = Bool()

  override def cloneType = new StreamChannel(w).asInstanceOf[this.type]
}

class StreamIO(w: Int) extends Bundle {
  val in = Flipped(Decoupled(new StreamChannel(w)))
  val out = Decoupled(new StreamChannel(w))

  def flipConnect(other: StreamIO) {
    in <> other.out
    other.in <> out
  }

  override def cloneType = new StreamIO(w).asInstanceOf[this.type]
}

object NetworkHelpers {
  def reverse_bytes(a: UInt, n: Int) = {
    val bytes = (0 until n).map(i => a((i + 1) * 8 - 1, i * 8))
    Cat(bytes)
  }
}

// entries is a Seq of UInt values with which to initialize the freelist.
class FreeList(val entries: Seq[UInt]) extends Module {
  val io = IO(new Bundle {
    // enq ptrs into this module
    val enq = Flipped(Decoupled(chiselTypeOf(entries(0))))
    // deq ptrs from this module
    val deq = Decoupled(chiselTypeOf(entries(0)))
  })

  // create FIFO queue to store pointers
  val queue_in = Wire(Decoupled(chiselTypeOf(entries(0))))
  val queue_out = Queue(queue_in, entries.size)

  val sReset :: sIdle :: Nil = Enum(2)
  val state = RegInit(sReset)

  val entries_vec = Vec(entries)
  val index = RegInit(0.U(log2Ceil(entries.size).W))

  // default - connect queue to enq/deq interfaces
  queue_in <> io.enq
  io.deq <> queue_out

  switch(state) {
    is (sReset) {
      // disconnect queue from enq/deq interfaces
      io.enq.ready := false.B
      io.deq.valid := false.B
      queue_out.ready := false.B
      // insert entries into the queue on reset
      queue_in.valid := true.B
      queue_in.bits := entries_vec(index)
      when (queue_in.ready) {
        index := index + 1.U
      }
      when (index === (entries.size - 1).U) {
        state := sIdle
      }
    }
    is (sIdle) {
      queue_in <> io.enq
      io.deq <> queue_out
    }
  }

}

