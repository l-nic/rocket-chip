
package freechips.rocketchip.tile

import Chisel._

import chisel3.{chiselTypeOf, WireDefault}
import chisel3.util.{EnqIO, DeqIO, HasBlackBoxResource}
import chisel3.experimental._
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
  val ETH_MIN_BYTES = 64
  val ETH_HEAD_BYTES = 16
  val ETH_MAC_BITS = 48
  val ETH_TYPE_BITS = 16
  val ETH_PAD_BITS = 16 // TODO(sibanez): what is this?? remove?

  val ETH_MAX_FLITS = ETH_MAX_BYTES/NET_IF_BYTES
  val ETH_MIN_FLITS = ETH_MIN_BYTES/NET_IF_BYTES

  val IPV4_HEAD_BYTES = 20

  def NET_FULL_KEEP = ~0.U(NET_IF_BYTES.W)
  def ETH_BCAST_MAC = ~0.U(ETH_MAC_BITS.W)

  val LWRITE_ADDR = 31.U
  val LREAD_ADDR = 30.U

  val IP_TYPE = 0x800.U(16.W)
  val LNIC_PROTO = 0x99.U(8.W)
  val LNIC_HDR_BYTES = 14

  val TEST_CONTEXT_ID = 0x1234.U(16.W)

  val SWITCH_MAC_ADDR = "h085566778808".U
  val NIC_MAC_ADDR = "h081122334408".U
  val NIC_IP_ADDR = "h0A000001".U // 10.0.0.1
}

object NetworkHelpers {
  def reverse_bytes(a: UInt, n: Int) = {
    val bytes = (0 until n).map(i => a((i + 1) * 8 - 1, i * 8))
    Cat(bytes)
  }
}

case class LNICParams(
  usingLNIC: Boolean = false,
  usingGPRs: Boolean = false,
  rxQueueFlits: Int = 16,
  txQueueFlits: Int = 16,
  pktizePktBufFlits: Int = 2 * LNICConsts.ETH_MAX_FLITS,
  arbiterPktBufFlits: Int = 2 * LNICConsts.ETH_MAX_FLITS,
  arbiterMetaBufFlits: Int = 2 * LNICConsts.ETH_MAX_FLITS,
  assemblePktBufFlits: Int = 2 * LNICConsts.ETH_MAX_FLITS,
  parserPktBufFlits: Int = 2 * LNICConsts.ETH_MAX_FLITS,
  parserMetaBufFlits: Int = 2 * LNICConsts.ETH_MAX_FLITS,
  maPktBufFlits: Int = 2 * LNICConsts.ETH_MAX_FLITS,
  maMetaBufFlits: Int = 2 * LNICConsts.ETH_MAX_FLITS,
  deparserPktBufFlits: Int = 2 * LNICConsts.ETH_MAX_FLITS,
  deparserMetaBufFlits: Int = 2 * LNICConsts.ETH_MAX_FLITS
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

//  // Connect io.core to io.net with FIFOs for now
//  io.core.out <> Queue(io.net.in, p(LNICKey).inBufFlits)
//  io.net.out <> Queue(io.core.in, p(LNICKey).outBufFlits)

  // NIC datapath
  val pktize = Module(new LNICPktize)
  val arbiter = Module(new LNICArbiter)
  val pisa = Module(new LNICPISA)
  val split = Module(new LNICSplit)
  val assemble = Module(new LNICAssemble)

  pktize.io.net_in <> io.core.in
  arbiter.io.core_in <> pktize.io.net_out
  arbiter.io.core_meta_in <> pktize.io.meta_out
  arbiter.io.net_in <> io.net.in
  pisa.io.net_in <> arbiter.io.net_out
  pisa.io.meta_in <> arbiter.io.meta_out
  split.io.net_in <> pisa.io.net_out
  split.io.meta_in <> pisa.io.meta_out
  io.net.out <> split.io.net_out
  assemble.io.net_in <> split.io.core_out
  assemble.io.meta_in <> split.io.core_meta_out
  io.core.out <> assemble.io.net_out
}

/** An I/O Bundle for LNIC RxQueue.
 *  Based on QueueIO in: chisel3/src/main/scala/chisel3/util/Decoupled.scala
  * @param gen The type of data to queue
  * @param entries The max number of entries in the queue.
  */
class LNICRxQueueIO[T <: Data](private val gen: T, val entries: Int) extends Bundle
{ // See github.com/freechipsproject/chisel3/issues/765 for why gen is a private val and proposed replacement APIs.

  /* These may look inverted, because the names (enq/deq) are from the perspective of the client,
   *  but internally, the queue implementation itself sits on the other side
   *  of the interface so uses the flipped instance.
   */
  /** I/O to enqueue data (client is producer, and Queue object is consumer), is [[Chisel.DecoupledIO]] flipped. */
  val enq = Flipped(EnqIO(gen))
  /** I/O to dequeue data (client is consumer and Queue object is producer), is [[Chisel.DecoupledIO]]*/
  val deq = Flipped(DeqIO(gen))
  /** The current amount of data in the queue */
  val count = Output(UInt(log2Ceil(entries + 1).W))
  /** Command to "unread" either the last one or two words that were previously dequeued */
  val unread = Flipped(Valid(UInt(width = 2)))
}

/**
 * LNIC RxQueue implementation.
 * This module resides in the Rocket core's CSRFile. It is used in place of a simple FIFO
 * when using LNIC w/ GPRs. It is basically a normal FIFO that supports an "unread" operation
 * for up to two words.
 * That is, the FIFO retains the last two words that were read out of it so that it can roll
 * back the read pointer when the unread cmd is asserted.
 * Based on the Queue class in: chisel3/src/main/scala/chisel3/util/Decoupled.scala
 * @param gen The type of data to queue
 * @param entries The max number of entries in the queue
 *
 * @example {{{
 * val q = Module(new LNICRxQueue(UInt(), 16))
 * q.io.enq <> producer.io.out
 * consumer.io.in <> q.io.deq
 * }}}
 */
@chiselName
class LNICRxQueue[T <: Data](gen: T,
                             val entries: Int)
                            (implicit compileOptions: chisel3.core.CompileOptions)
    extends Module() {
  @deprecated("Module constructor with override _reset deprecated, use withReset", "chisel3")
  def this(gen: T, entries: Int, override_reset: Option[Bool]) = {
    this(gen, entries)
    this.override_reset = override_reset
  }
  @deprecated("Module constructor with override _reset deprecated, use withReset", "chisel3")
  def this(gen: T, entries: Int, _reset: Bool) = {
    this(gen, entries)
    this.override_reset = Some(_reset)
  }

  val genType = if (compileOptions.declaredTypeMustBeUnbound) {
    requireIsChiselType(gen)
    gen
  } else {
    if (DataMirror.internal.isSynthesizable(gen)) {
      chiselTypeOf(gen)
    } else {
      gen
    }
  }

  require(isPow2(entries), "LNICRxQueue requires a power of 2 number of entries!")
  val io = IO(new LNICRxQueueIO(genType, entries))

  val ram = Mem(entries, genType)
  val enq_ptr = RegInit(2.U(log2Ceil(entries).W))
  val deq_ptr = RegInit(2.U(log2Ceil(entries).W))
  val deq_ptr_minus1 = RegInit(1.U(log2Ceil(entries).W))
  val deq_ptr_minus2 = RegInit(0.U(log2Ceil(entries).W))

  // if enq_ptr === deq_ptr_minus2 then the queue is full no questions asked
  val ptr_match_full = enq_ptr === deq_ptr_minus2
  // if enq_ptr === deq_ptr then the queue may be empty (deq_ptr caught up to enq_ptr)
  //   or the queue was full and received 2 undo operations
  val ptr_match_empty = enq_ptr === deq_ptr
  val full = ptr_match_full
  val empty = ptr_match_empty && !ptr_match_full
  val do_enq = WireDefault(io.enq.fire())
  val do_deq = WireDefault(io.deq.fire())

  when (do_enq) {
    ram(enq_ptr) := io.enq.bits
    enq_ptr := enq_ptr + 1.U
  }

  when (do_deq && !io.unread.valid) {
    // This is the common case: performing a dequeue without performing an unread
    deq_ptr := deq_ptr + 1.U
    when (deq_ptr_minus1 =/= deq_ptr) {
      deq_ptr_minus1 := deq_ptr_minus1 + 1.U
      when (deq_ptr_minus2 =/= deq_ptr_minus1) {
        deq_ptr_minus2 := deq_ptr_minus2 + 1.U
      }
    }
  } .elsewhen (io.unread.valid) {
    // perform an unread operation (up to two words)
    when (io.unread.bits === 2.U) {
      deq_ptr := deq_ptr_minus2
      deq_ptr_minus1 := deq_ptr_minus2
    } .elsewhen (io.unread.bits === 1.U) {
      deq_ptr := deq_ptr_minus1
    }
  }

  io.deq.valid := !empty
  io.enq.ready := !full
  io.deq.bits := ram(deq_ptr)

  // TODO(sibanez): verify that count is actually driven correctly
  io.count := Mux(empty,
                  0.U,
                  Mux(full,
                      Mux(deq_ptr_minus2 > deq_ptr,
                          entries.U - (deq_ptr_minus2 - deq_ptr),
                          entries.U - (deq_ptr - deq_ptr_minus2)),
                      Mux(deq_ptr > enq_ptr,
                          entries.U - (deq_ptr - enq_ptr),
                          enq_ptr - deq_ptr)))
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

  // Connect core to simulated network.
  // TODO: maybe insert a module between the core and network that inserts
  //   timestamps into application payload to compute latency?
  //   Maybe if the LNIC src context ID == 0 then insert timestamp at the
  //   specified word offset. But then it's only measuring latency from
  //   first word to first word.
  def connectSimNetwork(clock: Clock, reset: Bool) {
    netPorts.foreach { net =>
      val sim = Module(new SimNetwork)
      val latency = Module(new LatencyModule)
      sim.io.clock := clock
      sim.io.reset := reset
      sim.io.net <> latency.io.net
      latency.io.nic.in <> net.out
      net.in <> latency.io.nic.out
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

class SimNetwork extends BlackBox with HasBlackBoxResource {
  val io = IO(new Bundle {
    val clock = Input(Clock())
    val reset = Input(Bool())
    val net = Flipped(new LNICNetIO)
  })

  addResource("/vsrc/SimNetwork.v")
  addResource("/csrc/SimNetwork.cc")
  addResource("/csrc/switch.h")
  addResource("/csrc/switch.cc")
  addResource("/csrc/device.h")
  addResource("/csrc/device.cc")
  addResource("/csrc/packet.h")
}

/**
 * Parse Eth/IP/LNIC headers.
 * Check if LNIC src/dst port indicates test_app then insert timestamp/latency measurement
 * If yes:
 *   For pkts going to core, record timestamp of the first word in the last 8 bytes of msg.
 *   For pkts coming from core, record timestamp - now in the last 8 bytes of msg.
 * TODO(sibanez): current implementation assumes 8-byte aligned msgs.
 */
class LatencyModule extends Module {
  val io = IO(new Bundle {
    val net = new LNICNetIO 
    val nic = new LNICNetIO 
  })

  val nic_ts = Module(new Timestamp(to_nic = true))
  nic_ts.io.net.in <> io.net.in
  io.nic.out <> nic_ts.io.net.out

  val net_ts = Module(new Timestamp(to_nic = false))
  net_ts.io.net.in <> io.nic.in
  io.net.out <> net_ts.io.net.out
}

class Timestamp(to_nic: Boolean = true) extends Module {
  val io = IO(new Bundle {
   val net = new LNICNetIO 
  })

  // state machine to parse headers
  val sWordOne :: sWordTwo :: sWordThree :: sWordFour :: sWordFive :: sWordSix :: sWaitEnd :: Nil = Enum(7)
  val state = RegInit(sWordOne)

  val reg_now = RegInit(0.U(64.W))
  reg_now := reg_now + 1.U

  val reg_ts_start = RegInit(0.U)
  val reg_eth_type = RegInit(0.U)
  val reg_ip_proto = RegInit(0.U)
  val reg_lnic_src = RegInit(0.U)
  val reg_lnic_dst = RegInit(0.U)

  // default - connect input to output
  io.net.out <> io.net.in

  switch (state) {
    is (sWordOne) {
      reg_ts_start := reg_now
      transition(sWordTwo)
    }
    is (sWordTwo) {
      reg_eth_type := NetworkHelpers.reverse_bytes(io.net.in.bits.data(47, 32), 2)
      transition(sWordThree)
    }
    is (sWordThree) {
      reg_ip_proto := io.net.in.bits.data(63, 56)
      transition(sWordFour)
    }
    is (sWordFour) {
      transition(sWordFive)
    }
    is (sWordFive) {
      reg_lnic_src := NetworkHelpers.reverse_bytes(io.net.in.bits.data(31, 16), 2)
      reg_lnic_dst := NetworkHelpers.reverse_bytes(io.net.in.bits.data(47, 32), 2)
      transition(sWordSix)
    }
    is (sWordSix) {
      transition(sWaitEnd)
    }
    is (sWaitEnd) {
      when (io.net.in.valid && io.net.in.ready && io.net.in.bits.last) {
        state := sWordOne
        // overwrite last bytes with timestamp / latency
        when (reg_eth_type === LNICConsts.IP_TYPE && reg_ip_proto === LNICConsts.LNIC_PROTO) {
          when (to_nic.B && (reg_lnic_src === LNICConsts.TEST_CONTEXT_ID)) {
            io.net.out.bits.data := NetworkHelpers.reverse_bytes(reg_ts_start, 8)
          } .elsewhen (!to_nic.B && (reg_lnic_dst === LNICConsts.TEST_CONTEXT_ID)) {
            val pkt_ts = NetworkHelpers.reverse_bytes(io.net.in.bits.data, 8)
            io.net.out.bits.data := NetworkHelpers.reverse_bytes(reg_now - pkt_ts, 8)
          }
        }
      }
    }
  }

  def transition(next_state: UInt) = {
    when (io.net.in.valid && io.net.in.ready) {
      when (!io.net.in.bits.last) {
        state := next_state
      } .otherwise {
        state := sWordOne
      }
    }
  }

}

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


