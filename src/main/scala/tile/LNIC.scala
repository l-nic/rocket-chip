
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

  def NET_FULL_KEEP = ~0.U(NET_IF_BYTES.W) // 0xFF.U
  def ETH_BCAST_MAC = ~0.U(ETH_MAC_BITS.W)
}

case class LNICParams(
  usingLNIC: Boolean = false,
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
    core.io.net.get <> outer.lnic.get.module.io.core
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
}

