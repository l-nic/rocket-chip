
package freechips.rocketchip.tile

import Chisel._

import chisel3.experimental._
import chisel3.util.{HasBlackBoxResource}
import freechips.rocketchip.config._
import freechips.rocketchip.subsystem._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.rocket._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._
import NetworkHelpers._
import LNICConsts._

class PISAIngressMetaOut extends Bundle {
  // metadata for pkts going to CPU
  val src_ip       = UInt(32.W)
  val src_context  = UInt(LNIC_CONTEXT_BITS.W)
  val msg_len      = UInt(MSG_LEN_BITS.W)
  val pkt_offset   = UInt(PKT_OFFSET_BITS.W)
  val dst_context  = UInt(LNIC_CONTEXT_BITS.W)
  val rx_msg_id    = UInt(LNIC_MSG_ID_BITS.W)

  override def cloneType = new PISAIngressMetaIO().asInstanceOf[this.type]
}

class PISAEgressMetaIn extends Bundle {
  // metadata for pkts coming from CPU
  val dst_ip         = UInt(32.W)
  val dst_context    = UInt(LNIC_CONTEXT_BITS.W)
  val msg_len        = UInt(MSG_LEN_BITS.W)
  val pkt_offset     = UInt(PKT_OFFSET_BITS.W)
  val src_context    = UInt(LNIC_CONTEXT_BITS.W)
  val tx_msg_id      = UInt(LNIC_MSG_ID_BITS.W)
  val buf_ptr        = UInt(BUF_PTR_BITS.W)
  val buf_size_class = UInt(SIZE_CLASS_BITS.W)

  override def cloneType = new PISAEgressMetaIO().asInstanceOf[this.type]
}

/**
 * All IO for the LNIC PISA Ingress module.
 */
class LNICPISAIngressIO extends Bundle {
  val net_in = Flipped(Decoupled(new StreamChannel(NET_DP_BITS)))
  val net_out = Decoupled(new StreamChannel(NET_DP_BITS))
  val meta_out = Valid(new PISAIngressMetaOut)
  val get_rx_msg_info = new GetRxMsgInfoIO

  override def cloneType = new LNICPISAIngressIO().asInstanceOf[this.type]
}

/**
 * All IO for the LNIC PISA Egress module.
 */
class LNICPISAEgressIO extends Bundle {
  val net_in = Flipped(Decoupled(new StreamChannel(NET_DP_BITS)))
  val meta_in = Valid(new PISAEgressMetaIn)
  val net_out = Decoupled(new StreamChannel(NET_DP_BITS))

  override def cloneType = new LNICPISAEgressIO().asInstanceOf[this.type]
}

/* IO for Ingress extern call */
class GetRxMsgInfoIO extends Bundle {
  val req = Valid(new GetRxMsgInfoReq)
  val resp = Flipped(Valid(new GetRxMsgInfoResp))
}

class GetRxMsgInfoReq extends Bundle {
  val src_ip = UInt(32.W)
  val src_context = UInt(LNIC_CONTEXT_BITS.W)
  val tx_msg_id = UInt(LNIC_MSG_ID_BITS.W)
  val msg_len = UInt(MSG_LEN_BITS.W)
}

class GetRxMsgInfoResp extends Bundle {
  val fail = Bool()
  val rx_msg_id = UInt(LNIC_MSG_ID_BITS.W)
  // TODO(sibanez): add additional fields for transport processing
}

/* Ingress Pipeline Blackbox */
class SDNetIngressWrapper extends BlackBox with HasBlackBoxResource {
  val io = IO(new Bundle {
    val clock = Input(Clock())
    val reset = Input(Bool())
    val net = new LNICPISAIngressIO
  })
}

/* Egress Pipeline Blackbox */
class SDNetEgressWrapper extends BlackBox with HasBlackBoxResource {
  val io = IO(new Bundle {
    val clock = Input(Clock())
    val reset = Input(Bool())
    val net = new LNICPISAEgressIO
  })
}

