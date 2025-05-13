
`include "ram_defines.svh"

// 2KB, 14-bit address
module ram_arbiter(
    input                           clk_i,
    input        [`ADDR_WIDTH-1:0]  pA_wb_addr_i,      
    input        [`ADDR_WIDTH-1:0]  pB_wb_addr_i,
    input                           pA_wb_stb_i,      
    input                           pB_wb_stb_i,      
    input        [3:0]              pA_wb_we_i,
    input        [3:0]              pB_wb_we_i,
    input        [`DATA_WIDTH-1:0]  pA_wb_data_i,
    input        [`DATA_WIDTH-1:0]  pB_wb_data_i,

    input        [`DATA_WIDTH-1:0]  p0_dffram_data_out_i,
    input        [`DATA_WIDTH-1:0]  p1_dffram_data_out_i,

    output logic                    pA_wb_ack_o,
    output logic                    pB_wb_ack_o,
    output logic                    pA_wb_stall_o,
    output logic                    pB_wb_stall_o,
    output logic [`DATA_WIDTH-1:0]  pA_wb_data_o,
    output logic [`DATA_WIDTH-1:0]  pB_wb_data_o,
    
    output logic [3:0]              p0_dffram_we_o,
    output logic [3:0]              p1_dffram_we_o,
    output logic                    p0_dffram_en_o,   
    output logic                    p1_dffram_en_o,   
    output logic [`DATA_WIDTH-1:0]  p0_dffram_data_in_o,
    output logic [`DATA_WIDTH-1:0]  p1_dffram_data_in_o,
    output logic [`ADDR_WIDTH-2:0]  p0_dffram_addr_o,
    output logic [`ADDR_WIDTH-2:0]  p1_dffram_addr_o
);

// combining ports into arrays to allow the generate statement to work
logic                    port_priority = 0;
logic [1:0]              port_warmed;
logic [`ADDR_WIDTH-1:0]  wb_addr_i   [0:1];  
logic                    wb_stb_i    [0:1];   
logic [3:0]              wb_we_i     [0:1];
logic [`DATA_WIDTH-1:0]  wb_data_i   [0:1];

logic                    wb_ack_o    [0:1];
logic                    wb_stall_o  [0:1];
logic [`DATA_WIDTH-1:0]  wb_data_o   [0:1];

assign wb_addr_i[0] = pA_wb_addr_i; assign wb_addr_i[1] = pB_wb_addr_i;
assign wb_stb_i[0]  = pA_wb_stb_i;  assign wb_stb_i[1]  = pB_wb_stb_i;
assign wb_we_i[0]   = pA_wb_we_i;   assign wb_we_i[1]   = pB_wb_we_i;
assign wb_data_i[0] = pA_wb_data_i; assign wb_data_i[1] = pB_wb_data_i;

assign pA_wb_ack_o   = wb_ack_o[0];   assign pB_wb_ack_o   = wb_ack_o[1];
assign pA_wb_data_o  = wb_data_o[0];  assign pB_wb_data_o  = wb_data_o[1];


// doing the same for the DFFRAM connections
logic [`DATA_WIDTH-1:0] dffram_data_out_i [0:1];
logic                   dffram_en_o       [0:1];
logic [3:0]             dffram_we_o       [0:1];
logic [`DATA_WIDTH-1:0] dffram_data_in_o  [0:1];
logic [`ADDR_WIDTH-2:0] dffram_addr_o     [0:1];

assign dffram_data_out_i[0] = p0_dffram_data_out_i; assign dffram_data_out_i[1] = p1_dffram_data_out_i;
assign p0_dffram_we_o = dffram_we_o[0];             assign p1_dffram_we_o = dffram_we_o[1];
assign p0_dffram_en_o = dffram_en_o[0];             assign p1_dffram_en_o = dffram_en_o[1];
assign p0_dffram_data_in_o = dffram_data_in_o[0];   assign p1_dffram_data_in_o = dffram_data_in_o[1];
assign p0_dffram_addr_o = dffram_addr_o[0];         assign p1_dffram_addr_o = dffram_addr_o[1]; 

assign pA_wb_stall_o = pB_wb_stb_i && pA_wb_addr_i[`ADDR_WIDTH-1] == pB_wb_addr_i[`ADDR_WIDTH-1] && port_priority == '1;
assign pB_wb_stall_o = pA_wb_stb_i && pB_wb_addr_i[`ADDR_WIDTH-1] == pA_wb_addr_i[`ADDR_WIDTH-1] && port_priority == '0;

genvar port;
generate
    for (port = 0; port < 2; port++) begin

        // assign wb_stall_o[port] = wb_stb_i[~port] && wb_addr_i[port][`ADDR_WIDTH-1] == wb_addr_i[~port][`ADDR_WIDTH-1] && port_priority == ~port;
        assign wb_data_o[port] = dffram_data_out_i[wb_addr_i[port][`ADDR_WIDTH-1]];

        always_ff @(posedge clk_i) begin
            if (wb_stb_i[port] && !wb_stall_o[port]) begin
                port_warmed[port] <= '1;
                dffram_we_o[wb_addr_i[port][`ADDR_WIDTH-1]]      <= wb_we_i[port];
                dffram_en_o[wb_addr_i[port][`ADDR_WIDTH-1]]      <= wb_stb_i[port];
                dffram_data_in_o[wb_addr_i[port][`ADDR_WIDTH-1]] <= wb_data_i[port];
                dffram_addr_o[wb_addr_i[port][`ADDR_WIDTH-1]]    <= wb_addr_i[port][`ADDR_WIDTH-2:0];
            end else begin
                port_warmed[port] <= '0;
                if (wb_stall_o[port]) begin
                    port_priority <= port;
                end
            end

            if (port_warmed[port]) begin
                wb_ack_o[port]  <= '1;
            end else begin
                wb_ack_o[port]  <= '0;
            end
        end
    end
endgenerate


// disable the ports of any unused ram
logic [1:0] ram_busy;
always_comb begin
    for (int ram_port = 0; ram_port < 2; ram_port++) begin
        ram_busy[ram_port] = 0;
        for (int in_port = 0; in_port < 2; in_port++) begin
            ram_busy[ram_port] |= (wb_addr_i[in_port][`ADDR_WIDTH-1] == ram_port[0]) && wb_stb_i[in_port];
        end
    end
end

always_ff @(posedge clk_i) begin
    for (int ram_port = 0; ram_port < 2; ram_port++) begin
        if (!ram_busy[ram_port]) begin
            dffram_en_o[ram_port] <= '0; 
        end
    end
end


endmodule