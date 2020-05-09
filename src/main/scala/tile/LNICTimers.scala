
package freechips.rocketchip.tile

import Chisel._

import chisel3.{SyncReadMem, VecInit, chiselTypeOf}
import chisel3.experimental._
import freechips.rocketchip.config._
import freechips.rocketchip.subsystem._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.rocket._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._
import LNICConsts._

class TimerMeta extends Bundle {
  // max pkt offset transmitted at the last timeout event
  val rtx_offset = UInt(PKT_OFFSET_BITS.W)
  val msg_desc = new TxMsgDescriptor
}

class ScheduleEvent extends Bundle {
  val msg_id = UInt(LNIC_MSG_ID_BITS.W)
  val delay = UInt(TIMER_BITS.W)
  val metadata = new TimerMeta
}

class CancelEvent extends Bundle {
  val msg_id = UInt(LNIC_MSG_ID_BITS.W)
}

class TimeoutEvent extends Bundle {
  val msg_id = UInt(LNIC_MSG_ID_BITS.W)
  val metadata = new TimerMeta
}

class TimerEntry extends Bundle {
  val valid = Bool()
  val timeout_val = UInt(TIMER_BITS.W)
  val metadata = new TimerMeta
}

/* LNIC Timers:
 * Maintain N timers that support schedule, reschedule, cancel, and timeout events
 */
class LNICTimersIO extends Bundle {
  val schedule = Flipped(Valid(new ScheduleEvent))
  val reschedule = Flipped(Valid(new ScheduleEvent))
  val cancel = Flipped(Valid(new CancelEvent))
  val timeout = Valid(new TimeoutEvent)
}

@chiselName
class LNICTimers(implicit p: Parameters) extends Module {
  val io = IO(new LNICTimersIO)

  // Timer state
  val timer_mem = SyncReadMem(NUM_MSG_BUFFERS, new TimerEntry)

  // Cycle counter to track time
  val now = RegInit(0.U(TIMER_BITS.W))
  now := now + 1.U

  // port used for initialization and Schedule/Reschedule/Timeout Events
  val timer_id = Wire(UInt(LNIC_MSG_ID_BITS.W))
  val timer_rw_port = timer_mem(timer_id)

  // initialize timer_mem so all entries are invalid
  val init_done_reg = RegInit(false.B)
  MemHelpers.memory_init(timer_rw_port, timer_id, NUM_MSG_BUFFERS, (new TimerEntry).fromBits(0.U), init_done_reg)

  /* Logic to process Schedule/Reschedule/Timeout Events */

  // Pipeline regs
  val schedule_reg = RegNext(io.schedule)
  val reschedule_reg = RegNext(io.reschedule)

  val new_timer_entry = Wire(new TimerEntry)

  // state used to search for timeouts
  val cur_timer_id = RegInit(0.U(LNIC_MSG_ID_BITS.W))
  val sRead :: sCheck :: Nil = Enum(2)
  val stateTimeout = RegInit(sRead)

  // defaults
  io.timeout.valid := false.B

  when (!reset.toBool) {
    when (schedule_reg.valid) {
      stateTimeout := sRead // reset timeout state machine
      // process schedule event
      new_timer_entry.valid := true.B
      new_timer_entry.timeout_val := now + schedule_reg.bits.delay
      new_timer_entry.metadata := schedule_reg.bits.metadata
      timer_id := schedule_reg.bits.msg_id
      timer_rw_port := new_timer_entry
    } .elsewhen (reschedule_reg.valid) {
      stateTimeout := sRead // reset timeout state machine
      // process reschedule event
      new_timer_entry.valid := true.B
      new_timer_entry.timeout_val := now + reschedule_reg.bits.delay
      new_timer_entry.metadata := reschedule_reg.bits.metadata
      timer_id := reschedule_reg.bits.msg_id
      timer_rw_port := new_timer_entry
    } .elsewhen (init_done_reg) {
      // state machine to search for timeouts

      timer_id := cur_timer_id

      switch (stateTimeout) {
        is (sRead) {
          // start reading
          // state transition
          stateTimeout := sCheck
        }
        is (sCheck) {
          // get read result
          val cur_timer_entry = Wire(new TimerEntry)
          cur_timer_entry := timer_rw_port
          when (cur_timer_entry.valid && now >= cur_timer_entry.timeout_val) {
            // fire timeout event
            io.timeout.valid := true.B
            io.timeout.bits.msg_id := cur_timer_id
            io.timeout.bits.metadata := cur_timer_entry.metadata
            // do not fire the timer again
            val update_timer_entry = Wire(new TimerEntry)
            update_timer_entry := cur_timer_entry
            update_timer_entry.valid := false.B
            timer_rw_port := update_timer_entry
          }
          // move to next timer entry
          cur_timer_id := Mux(cur_timer_id === (NUM_MSG_BUFFERS-1).U,
                              0.U,
                              cur_timer_id + 1.U)
          // state transition
          stateTimeout := sRead
        }
      }
    }
  }

  /* Logic to process CancelEvents */

  // Pipeline reg
  val cancel_reg = RegNext(io.cancel)

  when (cancel_reg.valid && !reset.toBool) {
    val cancel_entry = Wire(new TimerEntry)
    cancel_entry.valid := false.B
    cancel_entry.timeout_val := 0.U
    cancel_entry.metadata := (new TimerMeta).fromBits(0.U)
    timer_mem(cancel_reg.bits.msg_id) := cancel_entry
  }

}

