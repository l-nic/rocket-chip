
import "DPI-C" function void network_tick (
    input string devname,

    input  bit     tx_valid,
    output bit     tx_ready,
    input  longint tx_data,
    input  byte    tx_keep,
    input  bit     tx_last,

    output bit     rx_valid,
    input  bit     rx_ready,
    output longint rx_data,
    output byte    rx_keep,
    output bit     rx_last
);

import "DPI-C" function void network_init (
    input string devname
);

module SimNetwork #(
  parameter DEVNAME = "tap0"
)
(
    input             clock,
    input             reset,

    // TX packets: HW --> tap iface
    input             net_out_valid,
    output reg        net_out_ready,
    input  [63:0]     net_out_bits_data,
    input  [7:0]      net_out_bits_keep,
    input             net_out_bits_last,

    // RX packets: tap iface --> HW
    output reg        net_in_valid,
    input             net_in_ready,
    output reg [63:0] net_in_bits_data,
    output reg [7:0]  net_in_bits_keep,
    output reg        net_in_bits_last
);

    string devname = DEVNAME;

    initial begin
        network_init(devname);
    end

    always@(posedge clock) begin
        if (reset) begin
            net_out_ready <= 0;
            net_in_valid <= 0;
            net_in_bits_data <= 0;
            net_in_bits_keep <= 0;
            net_in_bits_last <= 0;
        end
        else begin
            network_tick(
                devname,

                net_out_valid,
                net_out_ready,
                net_out_bits_data,
                net_out_bits_keep,
                net_out_bits_last,
    
                net_in_valid,
                net_in_ready,
                net_in_bits_data,
                net_in_bits_keep,
                net_in_bits_last);
        end
    end

endmodule
