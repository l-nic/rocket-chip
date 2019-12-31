
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

class PISAMetaIn extends Bundle {
  val ingress_id = Bool() // 0 = network, 1 = CPU

  /* metadata for pkts from CPU only (set by pktization buffer) */
  val msg_id = UInt(16.W)
  val offset = UInt(16.W) // pkt offset within msg
  val lnic_src = UInt(16.W) // src context ID

  override def cloneType = new PISAMetaIn().asInstanceOf[this.type]
}

class PISAMetaOut extends Bundle {
  val egress_id = Bool() // 0 = network, 1 = CPU
  /* metadata for pkt going to CPU */
  val lnic_dst = UInt(16.W) // dst context ID from LNIC hdr
  val msg_len = UInt(16.W)

  override def cloneType = new PISAMetaOut().asInstanceOf[this.type]
}

/**
 * All IO for the LNIC PISA module.
 */
class LNICPISAIO extends Bundle {
  val net_in = Flipped(Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH)))
  val meta_in = Flipped(Valid (new PISAMetaIn))
  val net_out = Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH))
  val meta_out = Valid(new PISAMetaOut)

  override def cloneType = new LNICPISAIO().asInstanceOf[this.type]
}

/**
 * LNIC PISA module.
 * Temporary placeholder for SDNet module.
 */
class LNICPISA(implicit p: Parameters) extends Module {
  val io = IO(new LNICPISAIO)
  val parser = Module(new PISAParser)
  val ma = Module(new PISAMA)
  val deparser = Module(new PISADeparser)
  parser.io.net_in <> io.net_in
  parser.io.meta_in <> io.meta_in
  ma.io.net_in <> parser.io.net_out
  ma.io.meta_in <> parser.io.meta_out
  deparser.io.net_in <> ma.io.net_out
  deparser.io.meta_in <> ma.io.meta_out
  io.net_out <> deparser.io.net_out
  io.meta_out <> deparser.io.meta_out
}

/**
 * LNIC PISA Parser.
 *
 * Tasks:
 *   - If pkt from network:
 *     - remove Ethernet, IP, and LNIC headers and fill out PHV
 *     - mark pkt for drop if no LNIC header
 *   - Else:
 *     - remove app hdr and fill out PHV
 *   - NOTE: all pkts from the network that don't have an Eth/IP/LNIC hdr are dropped in the parser
 *
 * // standard Ethernet header (14 bytes = 112 bits)
 * header Ethernet_h {
 *     EthAddr_t dstAddr;
 *     EthAddr_t srcAddr;
 *     bit<16> etherType;
 * }
 * 
 * // IPv4 header without options (20 bytes = 160 bits)
 * header IPv4_h {
 *     bit<4> version;
 *     bit<4> ihl;
 *     bit<8> tos;
 *     bit<16> totalLen;
 *     bit<16> identification;
 *     bit<3> flags;
 *     bit<13> fragOffset;
 *     bit<8> ttl;
 *     bit<8> protocol;
 *     bit<16> hdrChecksum;
 *     IPv4Addr_t srcAddr;
 *     IPv4Addr_t dstAddr;
 * }
 * 
 * // L-NIC transport header (14 bytes = 112 bits)
 * header LNIC_h {
 *     bit<16> src_context;
 *     bit<16> dst_context;
 *     bit<16> msg_id;
 *     bit<16> msg_len;
 *     bit<16> offset;
 *     bit<32> padding; // used to make Ethernet, IP, LNIC hdrs 64-bit aligned for an easy parser implementation
 * }
 */
class ParserMeta extends Bundle {
  /* metadata for pkts from network only */
  val eth_dst = UInt(48.W)
  val eth_src = UInt(48.W)
  val ip_src = UInt(32.W)
  /* metadata for both pkts from CPU and network */
  val ip_dst = UInt(32.W)
  val lnic_src = UInt(16.W)
  val lnic_dst = UInt(16.W)
  val msg_id = UInt(16.W)
  val msg_len = UInt(16.W)
  val offset = UInt(16.W)
  val drop = Bool()
  val ingress_id = Bool()

  override def cloneType = new ParserMeta().asInstanceOf[this.type]
}

class PISAParserIO extends Bundle {
  val net_in = Flipped(Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH)))
  val meta_in = Flipped(Valid (new PISAMetaIn))
  val net_out = Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH))
  val meta_out = Decoupled(new ParserMeta)

  override def cloneType = new PISAParserIO().asInstanceOf[this.type]
}

@chiselName
class PISAParser(implicit p: Parameters) extends Module {
  val io = IO(new PISAParserIO)

  val metaQueue_in = Wire(Decoupled(new ParserMeta))
  val pktQueue_in = Wire(Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH)))

  // wire up queues 
  io.meta_out <> Queue(metaQueue_in, p(LNICKey).parserMetaBufFlits)
  io.net_out <> Queue(pktQueue_in, p(LNICKey).parserPktBufFlits)
  io.net_in.ready := pktQueue_in.ready

  // Regs to hold parsed values
  val reg_eth_dst = RegInit(0.U(48.W))
  val reg_eth_src = RegInit(0.U(48.W))
  val reg_ip_src = RegInit(0.U(32.W))
  /* metadata for both pkts from CPU and network */
  val reg_ip_dst = RegInit(0.U(32.W))
  val reg_lnic_src = RegInit(0.U(16.W))
  val reg_lnic_dst = RegInit(0.U(16.W))
  val reg_msg_id = RegInit(0.U(16.W))
  val reg_msg_len = RegInit(0.U(16.W))
  val reg_offset = RegInit(0.U(16.W))
  val reg_drop = RegInit(false.B)
  val reg_ingress_id = RegInit(false.B)

  // state machine to parse pkt
  val sWordOne :: sWordTwo :: sWordThree :: sWordFour :: sWordFive :: sWordSix :: sWriteMeta :: sWaitEnd :: Nil = Enum(8)
  val state = RegInit(sWordOne)

  // default queue write logic
  metaQueue_in.valid := false.B
  metaQueue_in.bits.eth_dst := reg_eth_dst
  metaQueue_in.bits.eth_src := reg_eth_src
  metaQueue_in.bits.ip_src := reg_ip_src
  metaQueue_in.bits.ip_dst := reg_ip_dst
  metaQueue_in.bits.lnic_src := reg_lnic_src
  metaQueue_in.bits.lnic_dst := reg_lnic_dst
  metaQueue_in.bits.msg_id := reg_msg_id
  metaQueue_in.bits.msg_len := reg_msg_len
  metaQueue_in.bits.offset := reg_offset
  metaQueue_in.bits.drop := reg_drop
  metaQueue_in.bits.ingress_id := reg_ingress_id

  pktQueue_in.valid := false.B
  pktQueue_in.bits := io.net_in.bits

  switch (state) {
    is (sWordOne) {
      // parsing logic
      // NOTE: assuming SDNet style signaling for meta_in so meta_in should be valid on the first word of the pkt
      when (io.net_in.valid && io.net_in.ready) {
        reg_ingress_id := io.meta_in.bits.ingress_id
        when (io.meta_in.bits.ingress_id) {
          // pkt from CPU
          reg_ip_dst := reverse_bytes(io.net_in.bits.data(31, 0), 4)
          reg_lnic_src := io.meta_in.bits.lnic_src
          reg_lnic_dst := reverse_bytes(io.net_in.bits.data(47, 32), 2)
          reg_msg_id := io.meta_in.bits.msg_id
          reg_msg_len := reverse_bytes(io.net_in.bits.data(63, 48), 2)
          reg_offset := io.meta_in.bits.offset
        } .otherwise {
          // pkt from network
          reg_eth_dst := reverse_bytes(io.net_in.bits.data(47, 0), 6)
          reg_eth_src := reverse_bytes(io.net_in.bits.data(63, 48), 2) << 32
        }
      }

      // next state logic
      when (io.net_in.valid && io.net_in.ready) {
        when (io.net_in.bits.last) {
          clear_regs()
          state := sWordOne
        } .elsewhen (io.meta_in.bits.ingress_id) {
          // pkt from CPU
          state := sWriteMeta
        } .otherwise {
          state := sWordTwo
        }
      }
    }
    is (sWordTwo) {
      // parsing logic
      val eth_type = reverse_bytes(io.net_in.bits.data(47, 32), 2)
      when (io.net_in.valid && io.net_in.ready) {
        reg_eth_src := reg_eth_src | reverse_bytes(io.net_in.bits.data(31, 0), 4)
      }

      // next state logic
      when (io.net_in.valid && io.net_in.ready) {
        when (io.net_in.bits.last) {
          clear_regs()
          state := sWordOne
        } .elsewhen (eth_type =/= LNICConsts.IP_TYPE) {
          reg_drop := true.B
          state := sWriteMeta
        } .otherwise {
          state := sWordThree
        }
      }
    }
    is (sWordThree) {
      // parsing logic
      val ip_proto = io.net_in.bits.data(63, 56)

      // next state logic
      when (io.net_in.valid && io.net_in.ready) {
        when (io.net_in.bits.last) {
          clear_regs()
          state := sWordOne
        } .elsewhen (ip_proto =/= LNICConsts.LNIC_PROTO) {
          reg_drop := true.B
          state := sWriteMeta
        } .otherwise {
          state := sWordFour
        }       
      }
    }
    is (sWordFour) {
      // parsing logic
      when (io.net_in.valid && io.net_in.ready) {
        reg_ip_src := reverse_bytes(io.net_in.bits.data(47, 16), 4)
        reg_ip_dst := reverse_bytes(io.net_in.bits.data(63, 48), 2) << 16
      }

      // next state logic
      when (io.net_in.valid && io.net_in.ready) {
        when (io.net_in.bits.last) {
          clear_regs()
          state := sWordOne
        } .otherwise {
          state := sWordFive
        }
      }
    }
    is (sWordFive) {
      // parsing logic
      when (io.net_in.valid && io.net_in.ready) {
        reg_ip_dst := reg_ip_dst | reverse_bytes(io.net_in.bits.data(15, 0), 2)
        reg_lnic_src := reverse_bytes(io.net_in.bits.data(31, 16), 2)
        reg_lnic_dst := reverse_bytes(io.net_in.bits.data(47, 32), 2)
        reg_msg_id := reverse_bytes(io.net_in.bits.data(63, 48), 2)
      }

      // next state logic
      when (io.net_in.valid && io.net_in.ready) {
        when (io.net_in.bits.last) {
          clear_regs()
          state := sWordOne
        } .otherwise {
          state := sWordSix
        }
      }
    }
    is (sWordSix) {
      // parsing logic
      when (io.net_in.valid && io.net_in.ready) {
        reg_msg_len := reverse_bytes(io.net_in.bits.data(15, 0), 2)
        reg_offset := reverse_bytes(io.net_in.bits.data(31, 16), 2)
      }

      // next state logic
      when (io.net_in.valid && io.net_in.ready) {
        when (io.net_in.bits.last) {
          clear_regs()
          state := sWordOne
        } .otherwise {
          state := sWriteMeta
        }
      }
    }
    is (sWriteMeta) {
      // write metaQueue
      metaQueue_in.valid := io.net_in.valid && io.net_in.ready // only write metadata on first word of pkt
      
      // wire up pktQueue
      pktQueue_in.valid := io.net_in.valid
      pktQueue_in.bits := io.net_in.bits

      // next state logic - stay in this state until the first word of the pkt is written
      when (io.net_in.valid && io.net_in.ready) {
        when (io.net_in.bits.last) {
          clear_regs()
          state := sWordOne
        } .otherwise {
          state := sWaitEnd
        }
      }
    }
    is (sWaitEnd) {
      // wire up pktQueue
      pktQueue_in.valid := io.net_in.valid
      pktQueue_in.bits := io.net_in.bits
      when (io.net_in.valid && io.net_in.ready && io.net_in.bits.last) {
        clear_regs()
        state := sWordOne
      }
    }
  }

  def clear_regs() = {
    reg_eth_dst := 0.U
    reg_eth_src := 0.U
    reg_ip_src := 0.U
    reg_ip_dst := 0.U
    reg_lnic_src := 0.U
    reg_lnic_dst := 0.U
    reg_msg_id := 0.U
    reg_msg_len := 0.U
    reg_offset := 0.U
    reg_drop := false.B
    reg_ingress_id := false.B
  }

}

/**
 * LNIC PISA M/A module.
 *
 * Tasks:
 *   - Stream incomming pkt into pktQueue
 *   - Stream incmming metadata into M/A pipeline
 *   - Synchronize at the end of the M/A pipeline and the pktQueue
 *     (i.e. metadata should be written on the first word to the deparser)
 */
class MAMeta extends ParserMeta {
  val egress_id = Bool()
  val ip_chksum = UInt(16.W)

  override def cloneType = new MAMeta().asInstanceOf[this.type]
}

class PISAMAIO extends Bundle {
  val net_in = Flipped(Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH)))
  val meta_in = Flipped(Decoupled(new ParserMeta))
  val net_out = Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH))
  val meta_out = Decoupled(new MAMeta)

  override def cloneType = new PISAMAIO().asInstanceOf[this.type]
}

@chiselName
class PISAMA(implicit p: Parameters) extends Module {
  val io = IO(new PISAMAIO)

  val pktQueue_out = Wire(Flipped(Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH))))
  val metaQueue_in = Wire(Decoupled(new MAMeta))
  val metaQueue_out = Wire(Flipped(Decoupled(new MAMeta)))

  // wire up queues
  // NOTE: can add more metadata queues for more sophisticated M/A processing
  // just need to remember to increase pktQueue depth to ensure it can hold all the pkts
  pktQueue_out <> Queue(io.net_in, p(LNICKey).maPktBufFlits)
  metaQueue_out <> Queue(metaQueue_in, p(LNICKey).maMetaBufFlits)

  metaQueue_in.valid := io.meta_in.valid
  io.meta_in.ready := true.B // no backpressure on metadata bus, TODO(sibanez): make sure metadata is never dropped here
  when (io.meta_in.bits.ingress_id) {
    // pkt came from CPU
    metaQueue_in.bits.eth_dst    := LNICConsts.SWITCH_MAC_ADDR
    metaQueue_in.bits.eth_src    := LNICConsts.NIC_MAC_ADDR
    metaQueue_in.bits.ip_src     := LNICConsts.NIC_IP_ADDR
  } .otherwise {
    // pkt came from network
    metaQueue_in.bits.eth_dst    := io.meta_in.bits.eth_dst
    metaQueue_in.bits.eth_src    := io.meta_in.bits.eth_src
    metaQueue_in.bits.ip_src     := io.meta_in.bits.ip_src
  }
  metaQueue_in.bits.ip_dst     := io.meta_in.bits.ip_dst
  metaQueue_in.bits.lnic_src   := io.meta_in.bits.lnic_src
  metaQueue_in.bits.lnic_dst   := io.meta_in.bits.lnic_dst
  metaQueue_in.bits.msg_id     := io.meta_in.bits.msg_id
  metaQueue_in.bits.msg_len    := io.meta_in.bits.msg_len
  metaQueue_in.bits.offset     := io.meta_in.bits.offset
  metaQueue_in.bits.drop       := io.meta_in.bits.drop
  metaQueue_in.bits.ingress_id := io.meta_in.bits.ingress_id
  metaQueue_in.bits.egress_id  := !io.meta_in.bits.ingress_id // network goes to CPU and CPU goes to network
  metaQueue_in.bits.ip_chksum  := 0.U(16.W) // TODO(sibanez): implement this ...

  // state machine to synchronize metadata and pkts and drive outputs
  val sWaitPktAndMeta :: sWaitStart :: sWaitEnd :: Nil = Enum(3)
  val state = RegInit(sWaitPktAndMeta)

  // default - disconnect queues and outputs
  io.net_out.valid := false.B
  pktQueue_out.ready := false.B
  io.net_out.bits := pktQueue_out.bits
  io.meta_out.valid := false.B
  metaQueue_out.ready := false.B
  io.meta_out.bits := metaQueue_out.bits

  switch (state) {
    is (sWaitPktAndMeta) {
      when (pktQueue_out.valid && metaQueue_out.valid) {
        io.net_out.valid := true.B
        pktQueue_out.ready := io.net_out.ready
        io.meta_out.valid := true.B
        metaQueue_out.ready := io.net_out.ready // only read metaQueue if net_out is ready
        when (!pktQueue_out.bits.last) {
          when (io.net_out.ready) {
            state := sWaitEnd
          } .otherwise {
            state := sWaitStart
          }
        }
      }
    }
    is (sWaitStart) {
      // wait for first word to be transferred
      io.net_out.valid := pktQueue_out.valid
      pktQueue_out.ready := io.net_out.ready
      io.meta_out.valid := metaQueue_out.valid
      metaQueue_out.ready := io.net_out.ready // only read metaQueue if net_out is ready
      when (pktQueue_out.valid && io.net_out.ready) {
        when (pktQueue_out.bits.last) {
          state := sWaitPktAndMeta
        } .otherwise {
          state := sWaitEnd
        }
      }
    }
    is (sWaitEnd) {
      // connect pktQueue to net_out
      io.net_out.valid := pktQueue_out.valid
      pktQueue_out.ready := io.net_out.ready
      when (pktQueue_out.valid && pktQueue_out.ready && pktQueue_out.bits.last) {
        state := sWaitPktAndMeta
      }
    }
  }
}

/**
 * LNIC PISA Deparser module.
 *
 * Tasks:
 *   - drop pkt if drop bit is set
 *   - if pkt is going to CPU:
 *     - insert app hdr
 *   - if pkt is going to network:
 *     - insert Ethernet, IP, LNIC hdrs
 *   - set output metadata
 */
class PISADeparserIO extends Bundle {
  val net_in = Flipped(Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH)))
  val meta_in = Flipped(Decoupled(new MAMeta))
  val net_out = Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH))
  val meta_out = Valid(new PISAMetaOut)

  override def cloneType = new PISADeparserIO().asInstanceOf[this.type]
}

@chiselName
class PISADeparser(implicit p: Parameters) extends Module {
  val io = IO(new PISADeparserIO)

  // NOTE: assumes SDNet style metadata signaling on both sides
  // (i.e. metadata valid on first word of pkt)

  val pktQueue_out = Wire(Flipped(Decoupled(new StreamChannel(LNICConsts.NET_IF_WIDTH))))
  val metaQueue_out = Wire(Flipped(Decoupled(new MAMeta)))

  // wire up queues
  metaQueue_out <> Queue(io.meta_in, p(LNICKey).deparserMetaBufFlits)
  pktQueue_out <> Queue(io.net_in, p(LNICKey).deparserPktBufFlits)

  // state machine to drive outputs
  val sWordOne :: sWordTwo :: sWordThree :: sWordFour :: sWordFive :: sWordSix :: sWritePkt :: sDrop :: sHoldRegs :: Nil = Enum(9)
  val state = RegInit(sWordOne)

  // default - don't read queues, don't write outputs
  pktQueue_out.ready := false.B

  io.net_out.valid := false.B
  io.net_out.bits := pktQueue_out.bits
  io.meta_out.valid := false.B
  io.meta_out.bits.egress_id := metaQueue_out.bits.egress_id
  io.meta_out.bits.lnic_dst := metaQueue_out.bits.lnic_dst
  io.meta_out.bits.msg_len := metaQueue_out.bits.msg_len

  val reg_data = RegInit(0.U(LNICConsts.NET_IF_WIDTH))
  val reg_meta_out_valid = RegInit(false.B)
  val reg_egress_id = RegInit(false.B)
  val reg_lnic_dst = RegInit(0.U(16.W))
  val reg_msg_len = RegInit(0.U(16.W))
  val reg_next_state = RegInit(sWordOne) // only used for sHoldRegs state (i.e. back pressure)

  switch (state) {
    is (sWordOne) {
      // deparsing logic
      // Wait for pktQueue and metaQueue outputs to become valid
      // Don't write anything if drop bit set, read queues, if last bit is set stay in same state, otherwise transfer to sDrop state
      // Otherwise, if the pkt is going to the CPU - insert app hdr,
      //   transfer to sWritePkt state if net_out.ready is asserted, otherwise transfer to sHoldRegs
      // If the pkt is going to the network - insert eth_dst and eth_src,
      //   transfer to sWordTwo if net_out.ready is asserted, otherwise transfer to sHoldRegs

      when (pktQueue_out.valid && metaQueue_out.valid) {
        // Wait for pktQueue and metaQueue outputs to become valid
        when (metaQueue_out.bits.drop) {
          // Drop bit is set, don't write anything, but read the pktQueue
          pktQueue_out.ready := true.B
          // If last bit is set stay in same state, otherwise transfer to sDrop state
          when (!pktQueue_out.bits.last) {
            state := sDrop
          }
        } .otherwise {
          // Pkt is either going to CPU or network, write first word, do not read pktQueue
          io.net_out.valid := true.B
          io.net_out.bits.keep := LNICConsts.NET_FULL_KEEP
          io.net_out.bits.last := false.B
          io.meta_out.valid := true.B
          io.meta_out.bits.egress_id := metaQueue_out.bits.egress_id
          io.meta_out.bits.lnic_dst := metaQueue_out.bits.lnic_dst
          io.meta_out.bits.msg_len := metaQueue_out.bits.msg_len
          io.net_out.bits.data := Mux (metaQueue_out.bits.egress_id,
            // pkt going to CPU
            Cat(reverse_bytes(metaQueue_out.bits.msg_len, 2), reverse_bytes(metaQueue_out.bits.lnic_src, 2), reverse_bytes(metaQueue_out.bits.ip_src, 4)),
            // pkt going to network
            Cat(reverse_bytes(metaQueue_out.bits.eth_src(47, 32), 2), reverse_bytes(metaQueue_out.bits.eth_dst, 6)))
          // If net_out.ready is not set, fill out regs and transfer to sHoldRegs, otherwise transfer to either sWritePkt or sWordTwo
          when (!io.net_out.ready) {
            state := sHoldRegs
            reg_data       := io.net_out.bits.data
            reg_meta_out_valid := io.meta_out.valid
            reg_egress_id  := io.meta_out.bits.egress_id
            reg_lnic_dst   := io.meta_out.bits.lnic_dst
            reg_msg_len    := io.meta_out.bits.msg_len
            reg_next_state := Mux (metaQueue_out.bits.egress_id, sWritePkt, sWordTwo)
          } .otherwise {
            state := Mux (metaQueue_out.bits.egress_id, sWritePkt, sWordTwo)
          }
        }
      }
    }
    is (sWordTwo) {
      // eth_src ++ eth_type ++ ip_version ++ ip_ihl ++ ip_tos
      val eth_type = LNICConsts.IP_TYPE
      val ip_version = 4.U(4.W)
      val ip_ihl = 5.U(4.W)
      val ip_tos = 0.U(8.W)
      write_hdr_word(Cat(metaQueue_out.bits.eth_src(31, 0), eth_type, ip_version, ip_ihl, ip_tos), sWordThree)
    }
    is (sWordThree) {
      // ip_len (2B) ++ ip_id (2B) ++ ip_flags (3 bits) ++ ip_frag (13 bits) ++ ip_ttl (1B) ++ ip_proto (1B)
      val msg_len_plus_hdr = metaQueue_out.bits.msg_len + 20.U + LNICConsts.LNIC_HDR_BYTES.U
      val ip_len = Mux(msg_len_plus_hdr < LNICConsts.ETH_MAX_BYTES.U, msg_len_plus_hdr, (LNICConsts.ETH_MAX_BYTES.U - 20.U - LNICConsts.LNIC_HDR_BYTES.U))
      val ip_id = 1.U(16.W)
      val ip_flags = 0.U(3.W)
      val ip_frag = 0.U(13.W)
      val ip_ttl = 64.U(8.W)
      val ip_proto = LNICConsts.LNIC_PROTO
      write_hdr_word(Cat(ip_len, ip_id, ip_flags, ip_frag, ip_ttl, ip_proto), sWordFour)
    }
    is (sWordFour) {
      // ip_chksum (2B) ++ ip_src (4B) ++ ip_dst (2B)
      write_hdr_word(Cat(metaQueue_out.bits.ip_chksum, metaQueue_out.bits.ip_src, metaQueue_out.bits.ip_dst(31, 16)), sWordFive)
    }
    is (sWordFive) {
      // ip_dst (2B) ++ lnic_src (2B) ++ lnic_dst (2B) ++ lnic_msg_id (2B)
      write_hdr_word(Cat(metaQueue_out.bits.ip_dst(15, 0),
                         metaQueue_out.bits.lnic_src,
                         metaQueue_out.bits.lnic_dst,
                         metaQueue_out.bits.msg_id), sWordSix)
    }
    is (sWordSix) {
      // lnic_msg_len (2B) ++ lnic_offset (2B) ++ lnic_padding (4B)
      write_hdr_word(Cat(metaQueue_out.bits.msg_len, metaQueue_out.bits.offset, 0.U(32.W)), sWritePkt)
    }
    is (sWritePkt) {
      // Connect net_out to pktQueue_out and wait until last word is transferred, then transfer to sWordOne
      io.net_out <> pktQueue_out
      when (pktQueue_out.valid && pktQueue_out.ready && pktQueue_out.bits.last) {
        state := sWordOne
      }
    }
    is (sDrop) {
      // Keep reading pktQueue until last bit is set then transfer to sWordOne
      pktQueue_out.ready := true.B
      when (pktQueue_out.valid && pktQueue_out.bits.last) {
        state := sWordOne
      }
    }
    is (sHoldRegs) {
      // Hold outputs as the registered values to deal with back pressure
      // This state does not read any queues
      io.net_out.valid := true.B
      io.net_out.bits.data := reg_data
      io.net_out.bits.keep := LNICConsts.NET_FULL_KEEP
      io.net_out.bits.last := false.B
      io.meta_out.valid := reg_meta_out_valid
      io.meta_out.bits.egress_id := reg_egress_id
      io.meta_out.bits.lnic_dst := reg_lnic_dst
      io.meta_out.bits.msg_len := reg_msg_len
      when (io.net_out.ready) {
        clear_regs()
        state := reg_next_state
      }
    }
  }

  // read metaQueue on the last word of pkt
  metaQueue_out.ready := pktQueue_out.valid && pktQueue_out.ready && pktQueue_out.bits.last

  // helper method to write a word
  // NOTE: byte order is reverse when it's put on the wire.
  // So most significant byte of pktData input should be on the left
  def write_hdr_word (pktData: UInt, nextState: UInt) = {
    val net_data = pktData
    val net_data_rvs = reverse_bytes(pktData, LNICConsts.NET_IF_BYTES)
    dontTouch(net_data)
    dontTouch(net_data_rvs)
    when (pktQueue_out.valid && metaQueue_out.valid) {
      io.net_out.valid := true.B
      io.net_out.bits.data := net_data_rvs
      io.net_out.bits.keep := LNICConsts.NET_FULL_KEEP
      io.net_out.bits.last := false.B
      when (!io.net_out.ready) {
        state := sHoldRegs
        reg_data       := io.net_out.bits.data
        reg_next_state := nextState
      } .otherwise {
        state := nextState
      }
    }
  }

  def clear_regs() = {
    reg_data       := 0.U
    reg_egress_id  := false.B
    reg_lnic_dst   := 0.U
    reg_msg_len    := 0.U
    reg_next_state := sWordOne
    reg_meta_out_valid := false.B
  }

}


