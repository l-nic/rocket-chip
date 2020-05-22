
package freechips.rocketchip.tile

import Chisel._
import freechips.rocketchip.rocket.CoreLNICIO

/** Tile-level mixins for including LNIC **/

trait CanHaveLNIC { this: RocketTile =>
  // Nothing to do ...
}

trait CanHaveLNICModule { this: RocketTileModuleImp =>
  val net = p(LNICKey).map { params =>
    // create tile network IO and connect to core network IO
    val netio = IO(new CoreLNICIO)
    netio <> core.io.net.get
    netio
  }
}

