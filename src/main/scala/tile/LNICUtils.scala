
package freechips.rocketchip.tile

import Chisel._
import chisel3.{chiselTypeOf}

object NetworkHelpers {
  def reverse_bytes(a: UInt, n: Int) = {
    val bytes = (0 until n).map(i => a((i + 1) * 8 - 1, i * 8))
    Cat(bytes)
  }
}

/**
 * NOTE: Copied StreamChannel and StreamIO here to remove dependency on testchipip.
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

class StreamNarrower[T <: Data](inW: Int, outW: Int, metaType: T) extends Module {
  require(inW > outW)
  require(inW % outW == 0)

  val io = IO(new Bundle {
    val in = Flipped(Decoupled(new StreamChannel(inW)))
    val meta_in = Flipped(Valid(metaType.cloneType))
    val out = Decoupled(new StreamChannel(outW))
    val meta_out = Valid(metaType.cloneType)
  })

  val outBytes = outW / 8
  val outBeats = inW / outW

  val bits = Reg(new StreamChannel(inW))
  val meta_valid = Reg(Bool())
  val meta = Reg(metaType.cloneType)
  val count = Reg(UInt(log2Ceil(outBeats).W))

  val s_recv :: s_send_first :: s_send_finish :: Nil = Enum(3)
  val state = RegInit(s_recv)

  val nextData = bits.data >> outW.U
  val nextKeep = bits.keep >> outBytes.U

  io.in.ready := state === s_recv
  io.out.valid := (state === s_send_first) || (state === s_send_finish)
  io.out.bits.data := bits.data(outW - 1, 0)
  io.out.bits.keep := bits.keep(outBytes - 1, 0)
  io.out.bits.last := bits.last && !nextKeep.orR
  io.meta_out.bits := meta
  io.meta_out.valid := meta_valid && (state === s_send_first)

  when (io.in.fire()) {
    count := (outBeats - 1).U
    bits := io.in.bits
    meta_valid := io.meta_in.valid
    meta := io.meta_in.bits
    state := s_send_first
  }

  when (io.out.fire()) {
    count := count - 1.U
    bits.data := nextData
    bits.keep := nextKeep
    state := s_send_finish
    when (io.out.bits.last || count === 0.U) {
      state := s_recv
    }
  }
}

class StreamWidener[T <: Data](inW: Int, outW: Int, metaType: T) extends Module {
  require(outW > inW)
  require(outW % inW == 0)

  val io = IO(new Bundle {
    val in = Flipped(Decoupled(new StreamChannel(inW)))
    val meta_in = Flipped(Valid(metaType.cloneType))
    val out = Decoupled(new StreamChannel(outW))
    val meta_out = Valid(metaType.cloneType)
  })

  val inBytes = inW / 8
  val inBeats = outW / inW

  val data = Reg(Vec(inBeats, UInt(inW.W)))
  val keep = RegInit(Vec(Seq.fill(inBeats)(0.U(inBytes.W))))
  val last = Reg(Bool())
  val meta = Reg(metaType.cloneType)
  val meta_valid = Reg(Bool())

  val idx = RegInit(0.U(log2Ceil(inBeats).W))

  val s_recv_first :: s_recv_finish :: s_send :: Nil = Enum(3)
  val state = RegInit(s_recv_first)

  io.in.ready := (state === s_recv_first) || (state === s_recv_finish)
  io.out.valid := state === s_send
  io.out.bits.data := data.asUInt
  io.out.bits.keep := keep.asUInt
  io.out.bits.last := last
  io.meta_out.bits := meta
  io.meta_out.valid := meta_valid

  when (io.in.fire()) {
    idx := idx + 1.U
    data(idx) := io.in.bits.data
    keep(idx) := io.in.bits.keep
    state := s_recv_finish
    when (state === s_recv_first) {
      meta := io.meta_in.bits
      meta_valid := io.meta_in.valid
    }
    when (io.in.bits.last || idx === (inBeats - 1).U) {
      last := io.in.bits.last
      state := s_send
    }
  }

  when (io.out.fire()) {
    idx := 0.U
    keep.foreach(_ := 0.U)
    state := s_recv_first
  }
}

object StreamWidthAdapter {
  def apply[T <: Data](out: DecoupledIO[StreamChannel], meta_out: T, in: DecoupledIO[StreamChannel], meta_in: T) {
    if (out.bits.w > in.bits.w) {
      val widener = Module(new StreamWidener(in.bits.w, out.bits.w, chiselTypeOf(meta_out)))
      widener.io.in <> in
      widener.io.meta_in := meta_in
      out <> widener.io.out
      meta_out := widener.io.meta_out
    } else if (out.bits.w < in.bits.w) {
      val narrower = Module(new StreamNarrower(in.bits.w, out.bits.w, chiselTypeOf(meta_out)))
      narrower.io.in <> in
      narrower.io.meta_in := meta_in
      out <> narrower.io.out
      meta_out := narrower.io.meta_out
    } else {
      out <> in
      meta_out := meta_in
    }
  }

  def apply(out: DecoupledIO[StreamChannel], in: DecoupledIO[StreamChannel]) {
    if (out.bits.w > in.bits.w) {
      val widener = Module(new StreamWidener(in.bits.w, out.bits.w, Bool()))
      widener.io.in <> in
      out <> widener.io.out
    } else if (out.bits.w < in.bits.w) {
      val narrower = Module(new StreamNarrower(in.bits.w, out.bits.w, Bool()))
      narrower.io.in <> in
      out <> narrower.io.out
    } else {
      out <> in
    }
  }

}

