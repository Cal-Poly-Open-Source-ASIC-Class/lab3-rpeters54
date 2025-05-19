
`include "ram_defines.svh"
`timescale 1ns/1ps

// 2KB, 14-bit address
module ram_arbiter(
    input                           clk_i,
    input                           rst_n,
    input        [`ADDR_WIDTH-1:0]  pA_wb_addr_i,      
    input        [`ADDR_WIDTH-1:0]  pB_wb_addr_i,
    input                           pA_wb_stb_i,      
    input                           pB_wb_stb_i,      
    input        [3:0]              pA_wb_we_i,
    input        [3:0]              pB_wb_we_i,
    input        [`DATA_WIDTH-1:0]  pA_wb_data_i,
    input        [`DATA_WIDTH-1:0]  pB_wb_data_i,

    output logic                    pA_wb_ack_o,
    output logic                    pB_wb_ack_o,
    output logic                    pA_wb_stall_o,
    output logic                    pB_wb_stall_o,
    output logic [`DATA_WIDTH-1:0]  pA_wb_data_o,
    output logic [`DATA_WIDTH-1:0]  pB_wb_data_o
);

// combining ports into arrays to allow the generate statement to work
logic                    port_priority;
logic [`ADDR_WIDTH-1:0]  wb_addr_i   [0:1];  
logic                    wb_stb_i    [0:1];   
logic [3:0]              wb_we_i     [0:1];
logic [`DATA_WIDTH-1:0]  wb_data_i   [0:1];

logic                    wb_ack_o    [0:1];
logic                    wb_stall_o  [0:1];
logic [`DATA_WIDTH-1:0]  wb_data_o   [0:1];

logic                    prev_ram_selected   [0:1];  

assign wb_addr_i[0] = pA_wb_addr_i; assign wb_addr_i[1] = pB_wb_addr_i;
assign wb_stb_i[0]  = pA_wb_stb_i;  assign wb_stb_i[1]  = pB_wb_stb_i;
assign wb_we_i[0]   = pA_wb_we_i;   assign wb_we_i[1]   = pB_wb_we_i;
assign wb_data_i[0] = pA_wb_data_i; assign wb_data_i[1] = pB_wb_data_i;

assign pA_wb_ack_o   = wb_ack_o[0];   assign pB_wb_ack_o   = wb_ack_o[1];
assign pA_wb_data_o  = wb_data_o[0];  assign pB_wb_data_o  = wb_data_o[1];
assign pA_wb_stall_o = wb_stall_o[0]; assign pB_wb_stall_o = wb_stall_o[1];

// doing the same for the DFFRAM connections
logic [`DATA_WIDTH-1:0] dffram_data_out_i [0:1];
logic                   dffram_en_o       [0:1];
logic [3:0]             dffram_we_o       [0:1];
logic [`DATA_WIDTH-1:0] dffram_data_in_o  [0:1];
logic [`ADDR_WIDTH-2:0] dffram_addr_o     [0:1];


DFFRAM256x32 ram0 (
    .CLK(clk_i),
    .WE0(dffram_we_o[0]),
    .EN0(dffram_en_o[0]),
    .Di0(dffram_data_in_o[0]),
    .Do0(dffram_data_out_i[0]),
    .A0(dffram_addr_o[0])
);

DFFRAM256x32 ram1 (
    .CLK(clk_i),
    .WE0(dffram_we_o[1]),
    .EN0(dffram_en_o[1]),
    .Di0(dffram_data_in_o[1]),
    .Do0(dffram_data_out_i[1]),
    .A0(dffram_addr_o[1])
);

genvar port;
generate
    for (port = 0; port < 2; port++) begin

        // stall condition when both ports try to access same address, and the port does not have priority
        assign wb_stall_o[port] = wb_stb_i[port] && wb_stb_i[(1 - port)] 
            && wb_addr_i[port][`ADDR_WIDTH-1] == wb_addr_i[(1 - port)][`ADDR_WIDTH-1] 
            && port_priority == (1 - port);

        // data out is a mux connected whatever ram the ports address is currently looking at
        assign wb_data_o[port] = dffram_data_out_i[prev_ram_selected[port]];

        // logic for setting ack and priority
        always_ff @(posedge clk_i) begin
            if (!rst_n) begin
                prev_ram_selected[port] <= 0;
                wb_ack_o[port]          <= '0;
            end else begin
                prev_ram_selected[port] <= wb_addr_i[port][`ADDR_WIDTH-1];
                if (wb_stb_i[port] && !wb_stall_o[port]) begin
                    wb_ack_o[port] <= '1;
                end else begin
                    wb_ack_o[port] <= '0;
                end
            end
        end
    end
endgenerate


// port_priority reset (moved out of generator to avoid multi-driver issue)
always_ff @(posedge clk_i) begin
    if (!rst_n) begin
        port_priority  <= '0;
    end else if (wb_stall_o[0] || wb_stall_o[1]) begin
        port_priority <= ~port_priority;
    end
end


always_comb begin
    // combinational arbitration logic for each ram
    // provides access to ram for either port based on priority
    for (int ram = 0; ram < 2; ram++) begin
        case ({wb_stb_i[0] && wb_addr_i[0][`ADDR_WIDTH-1] == ram[0], wb_stb_i[1] && wb_addr_i[1][`ADDR_WIDTH-1] == ram[0]})
            2'b11 : begin
                dffram_we_o[ram]      = wb_we_i[port_priority];
                dffram_en_o[ram]      = wb_stb_i[port_priority];
                dffram_data_in_o[ram] = wb_data_i[port_priority];
                dffram_addr_o[ram]    = wb_addr_i[port_priority][`ADDR_WIDTH-2:0];
            end
            2'b10 : begin
                dffram_we_o[ram]      = wb_we_i[0];
                dffram_en_o[ram]      = wb_stb_i[0];
                dffram_data_in_o[ram] = wb_data_i[0];
                dffram_addr_o[ram]    = wb_addr_i[0][`ADDR_WIDTH-2:0];
            end
            2'b01 : begin
                dffram_we_o[ram]      = wb_we_i[1];
                dffram_en_o[ram]      = wb_stb_i[1];
                dffram_data_in_o[ram] = wb_data_i[1];
                dffram_addr_o[ram]    = wb_addr_i[1][`ADDR_WIDTH-2:0];
            end default begin 
                dffram_we_o[ram]      = 0;
                dffram_en_o[ram]      = 0;
                dffram_data_in_o[ram] = 0;
                dffram_addr_o[ram]    = 0;
            end
        endcase
    end
end


endmodule