
package freechips.rocketchip.tile

import Chisel._

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
      when (io.net_in.valid && io.net_in.ready) {
        io.meta_out.valid := true.B
        // TODO(sibanez): these are set assuming single pkt messages
        io.meta_out.bits.msg_id := reg_msg_id
        io.meta_out.bits.offset := 0.U
        io.meta_out.bits.lnic_src := 0.U
        when (!io.net_in.bits.last) {
          state := sWaitEnd
        }
        reg_msg_id := reg_msg_id + 1.U
      }
    }
    is (sWaitEnd) {
      when (io.net_in.valid && io.net_in.ready && io.net_in.bits.last) {
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

  override def cloneType = new AssembleIO().asInstanceOf[this.type]
}

@chiselName
class LNICAssemble(implicit p: Parameters) extends Module {
  val io = IO(new AssembleIO)

  val pktQueue_in = Wire(Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH)))

  io.net_out <> Queue(pktQueue_in, p(LNICKey).assemblePktBufFlits)
  io.net_in.ready := pktQueue_in.ready

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
      when (io.net_in.valid && io.net_in.ready && reg_msg_len <= LNICConsts.NET_IF_BYTES.U) {
        state := sIdle
        pktQueue_in.bits.last := true.B
        pktQueue_in.bits.keep := (1.U << reg_msg_len) - 1.U
      }
    }
  }

}


