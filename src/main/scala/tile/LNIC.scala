
package freechips.rocketchip.tile

import Chisel._

import freechips.rocketchip.config._
import freechips.rocketchip.subsystem._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.rocket._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._

object LNICConsts {
  val NET_IF_WIDTH = 64
  val NET_IF_BYTES = NET_IF_WIDTH/8
  val NET_LEN_BITS = 16

  val ETH_MAX_BYTES = 1520
  val ETH_HEAD_BYTES = 16
  val ETH_MAC_BITS = 48
  val ETH_TYPE_BITS = 16
  val ETH_PAD_BITS = 16

  val IPV4_HEAD_BYTES = 20

  def NET_FULL_KEEP = ~0.U(NET_IF_BYTES.W)
  def ETH_BCAST_MAC = ~0.U(ETH_MAC_BITS.W)

  val LWRITE_ADDR = 31.U
  val LREAD_ADDR = 30.U
}

case class LNICParams(
  usingLNIC: Boolean = false,
  usingGPRs: Boolean = false,
  inBufFlits: Int  = 2 * LNICConsts.ETH_MAX_BYTES / LNICConsts.NET_IF_BYTES,
  outBufFlits: Int = 2 * LNICConsts.ETH_MAX_BYTES / LNICConsts.NET_IF_BYTES
)

case object LNICKey extends Field[LNICParams]

/**
 * NOTE: Copied StreamChannel and StreamIO here to remove dependency on testchipip
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

/**
 * This is intended to be the IO to the core.
 */
class LNICCoreIO extends StreamIO(LNICConsts.NET_IF_WIDTH) {
  override def cloneType = (new LNICCoreIO).asInstanceOf[this.type]
}

/**
 * This is intended to be the IO to the external network.
 */
class LNICNetIO extends StreamIO(LNICConsts.NET_IF_WIDTH) {
  override def cloneType = (new LNICNetIO).asInstanceOf[this.type]
}

/**
 * All IO for the LNIC module.
 */
class LNICIO(implicit p: Parameters) extends CoreBundle()(p) {
  val core = new LNICCoreIO()
  val net = new LNICNetIO()
}

/**
 * Diplomatic LNIC module.
 */
class LNIC(implicit p: Parameters) extends LazyModule {
  lazy val module = new LNICModuleImp(this)
}

/**
 * LNIC module implementation.
 */
class LNICModuleImp(outer: LNIC)(implicit p: Parameters) extends LazyModuleImp(outer) {
  val io = IO(new LNICIO)
  // Connect io.core to io.net with FIFOs for now
  io.core.out <> Queue(io.net.in, p(LNICKey).inBufFlits)
  io.net.out <> Queue(io.core.in, p(LNICKey).outBufFlits)
}

/** Tile-level mixins for including LNIC **/

trait CanHaveLNIC { this: RocketTile =>
  val lnic = if (usingLNIC) Some(LazyModule(new LNIC)) else None
}

trait CanHaveLNICModule { this: RocketTileModuleImp =>
  val net = if (usingLNIC) Some(IO(new LNICNetIO)) else None
  def connectLNIC() {
    require(net.isDefined, "[CanHaveLNICModule] net is not defined.")
    require(outer.lnic.isDefined, "[CanHaveLNICModule] outer.lnic is not defined.")
    require(core.io.net.isDefined, "[CanHaveLNICModule] core.io.net is not defined.")
    // Connect network IO to LNIC module
    net.get <> outer.lnic.get.module.io.net
    // Connect LNIC module to RocketCore
    core.io.net.get.in <> outer.lnic.get.module.io.core.out
    outer.lnic.get.module.io.core.in <> core.io.net.get.out
  }
}

/** Top-level mixins for including LNIC **/

trait HasLNIC { this: RocketSubsystem =>
  val lnicTiles = tiles
}

trait HasLNICModuleImp extends LazyModuleImp with HasTileParameters {
  val outer: HasLNIC

  // Create one network IO port for each tile
  val netPorts = IO(Vec(outer.lnicTiles.size, new LNICNetIO))

  // Connect tile net IO to top-level net IO
  outer.lnicTiles.zip(netPorts).foreach { case (tile, net) =>
    net <> tile.module.net.get
  }

  private val packetWords = LNICConsts.ETH_MAX_BYTES / LNICConsts.NET_IF_BYTES

  def connectNicLoopback(qDepth: Int = 4 * packetWords, latency: Int = 10) {
    // Connect all network interfaces as loopback
    netPorts.foreach { net =>
      net.in <> Queue(LatencyPipe(net.out, latency), qDepth)
    }
  }

  def connectPktGen(pktLen: Int = 64) {
    netPorts.foreach { net =>
      val pktGen = Module(new PktGen(pktLen))
      pktGen.io.net.in <> net.out
      net.in <> pktGen.io.net.out
    }
  }

}

/* Test Modules */

class PktGen (pktLen: Int = 64) extends Module {
  val io = IO(new Bundle {
    val net = new LNICNetIO
  })

  /* A simple module that generates a 8 word pkt (plus one word msg length) once every 100 cycles */

  val pktDelay = RegInit(0.U(64.W))
  val wordCnt = RegInit(0.U(64.W))

  val sWaitStart :: sWriteWords :: sWaitResp :: Nil = Enum(3)
  val state = RegInit(sWaitStart)

  val numWords = pktLen/LNICConsts.NET_IF_BYTES + 1 // one for msg length as the first word

  // default io
  io.net.in.ready := true.B
  io.net.out.valid := false.B
  io.net.out.bits.data := 0.U
  io.net.out.bits.keep := LNICConsts.NET_FULL_KEEP
  io.net.out.bits.last := 0.U

  switch (state) {
    is (sWaitStart) {
      when (pktDelay >= 100.U) {
        state := sWriteWords
        pktDelay := 0.U
      }.otherwise {
        pktDelay := pktDelay + 1.U
      }
    }
    is (sWriteWords) {
      // drive outputs
      io.net.out.valid := true.B
      io.net.out.bits.data := wordCnt
      when (wordCnt === 0.U) {
        io.net.out.bits.data := pktLen.asUInt
      }
      when (wordCnt === (numWords - 1).asUInt) {
        io.net.out.bits.last := true.B
      }
      // next state logic
      when (wordCnt === (numWords - 1).asUInt && io.net.out.ready) {
        state := sWaitResp
        wordCnt := 0.U
      } .otherwise {
        wordCnt := wordCnt + 1.U
      }
    }
    is (sWaitResp) {
      when (io.net.in.valid) {
        state := sWaitStart
      }
    }
  }


}


