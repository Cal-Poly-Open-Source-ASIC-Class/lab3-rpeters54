
`timescale 1ns/1ps
`include "ram_defines.svh"

module tb_all;

`ifdef USE_POWER_PINS
    wire VPWR;
    wire VGND;
    assign VPWR=1;
    assign VGND=0;
`endif


logic                    clk_i;
logic                    rst_n;
logic [`ADDR_WIDTH-1:0]  pA_wb_addr_i;      
logic [`ADDR_WIDTH-1:0]  pB_wb_addr_i;
logic                    pA_wb_stb_i;      
logic                    pB_wb_stb_i;      
logic [3:0]              pA_wb_we_i;
logic [3:0]              pB_wb_we_i;
logic [`DATA_WIDTH-1:0]  pA_wb_data_i;
logic [`DATA_WIDTH-1:0]  pB_wb_data_i;

logic                    pA_wb_ack_o;
logic                    pB_wb_ack_o;
logic                    pA_wb_stall_o;
logic                    pB_wb_stall_o;
logic [`DATA_WIDTH-1:0]  pA_wb_data_o;
logic [`DATA_WIDTH-1:0]  pB_wb_data_o;

ram_arbiter arbiter(
    `ifdef USE_POWER_PINS
    .VPWR(VPWR),
    .VGND(VGND),
    `endif
    .*
);

// extra logic to allow for array access selection

logic [`ADDR_WIDTH-1:0]  wb_addr_i   [0:1];  
logic                    wb_stb_i    [0:1];   
logic [3:0]              wb_we_i     [0:1];
logic [`DATA_WIDTH-1:0]  wb_data_i   [0:1];

logic                    wb_ack_o    [0:1];
logic                    wb_stall_o  [0:1];
logic [`DATA_WIDTH-1:0]  wb_data_o   [0:1];

assign pA_wb_addr_i = wb_addr_i[0];
assign pB_wb_addr_i = wb_addr_i[1];
assign pA_wb_stb_i  = wb_stb_i[0];
assign pB_wb_stb_i  = wb_stb_i[1];
assign pA_wb_we_i   = wb_we_i[0];
assign pB_wb_we_i   = wb_we_i[1];
assign pA_wb_data_i = wb_data_i[0];
assign pB_wb_data_i = wb_data_i[1];
assign wb_ack_o     = '{pA_wb_ack_o, pB_wb_ack_o};
assign wb_stall_o   = '{pA_wb_stall_o, pB_wb_stall_o};
assign wb_data_o    = '{pA_wb_data_o, pB_wb_data_o};


// Sample to drive clock
localparam PERIOD = 50;
initial begin
clk_i = 0;
forever #(PERIOD/2) clk_i = ~clk_i;
end

// Necessary to create Waveform
initial begin
    // Name as needed

    $dumpfile("tb_all.vcd");
    $dumpvars(2, tb_all);
end


logic test_done;
initial begin
    test_done = 0;
    #1000000;
    if (!test_done) begin
        $error("Timeout: test did not finish in time");
        $finish;
    end
end

initial begin
    init();
    
    $display("Starting Single Port Tests...");
    test_simple_read_write(0, 10);
    test_simple_read_write(1, 10);

    $display("Testing Byte Addressability Tests...");
    test_masked_read_write(0, 32);
    test_masked_read_write(1, 32);
    
    $display("Starting Arbitration Tests...");
    test_arbitration_read_write(10, 0);
    test_arbitration_read_write(10, 1);
    
    $display("All tests finished.");
    $finish();
end

// initialize ram
task automatic init();
    rst_n = 0;
    foreach (wb_addr_i[i]) wb_addr_i[i] = '0;
    foreach (wb_stb_i[i])  wb_stb_i[i]  = 1'b0;
    foreach (wb_we_i[i])   wb_we_i[i]   = 4'b0000;
    foreach (wb_data_i[i]) wb_data_i[i] = '0;

    repeat (2) @(negedge clk_i);

    rst_n = 1;
    repeat (3) @(negedge clk_i);

endtask

task automatic test_simple_read_write(
    input                   port,
    input [`ADDR_WIDTH-1:0] num_read_writes
);

    logic [`ADDR_WIDTH-1:0] addr = 0;
    logic [`DATA_WIDTH-1:0] prev_data_i [$];

    $display("Starting test_simple_read_write for port %0d with %0d operations.", port, num_read_writes);

    // Check that ack is low and not stalling before we start
    assert (wb_ack_o[port] == 1'b0)
      else $error("Port %0d: ACK is unexpectedly high before starting operations.", port);
    assert (wb_stall_o[port] == 1'b0)
      else $error("Port %0d: STALL is unexpectedly high before starting operations.", port);


    $display("Port %0d: Starting Write Phase...", port);
    wb_stb_i[port] = 1'b1;
    for (addr = 0; addr < num_read_writes; addr++) begin
        wb_addr_i[port] = addr;
        wb_we_i[port]   = 4'hF; 
        wb_data_i[port] = $urandom;  
        prev_data_i.push_back(wb_data_i[port]);
        @(negedge clk_i);
        $display("Port %0d: Writing 0x%h to address 0x%h", port, wb_data_i[port], wb_addr_i[port]);
        assert (wb_ack_o[port] == '1)
            else $error("test_simple_read_write: output should be valid, failed at iteration: %d", addr);
        assert (wb_stall_o[port] == '0)
            else $error("test_simple_read_write: stalling without other writer, failed at iteration: %d", addr);   
    end

    wb_we_i[port] = 0;
    for (addr = 0; addr < num_read_writes; addr++) begin
        $display("Port %0d: Reading from address 0x%h", port, wb_addr_i[port]);
        wb_addr_i[port] = addr;
        @(negedge clk_i);

        assert (wb_ack_o[port] == 1'b1)
            else $error("Port %0d: Read ACK not received for addr 0x%h. wb_ack_o = %b", port, addr, wb_ack_o[port]);
        assert (wb_stall_o[port] == 1'b0)
            else $error("Port %0d: STALL asserted during read from addr 0x%h.", port, addr);


        assert (wb_data_o[port] == prev_data_i[addr])
            else $error("Port %0d: Read-write MISMATCH at addr 0x%h. Expected: 0x%h, Received: 0x%h", port, addr, prev_data_i[addr], wb_data_o[port]);
        
        $display("Port %0d: Read from addr 0x%h, Expected: 0x%h, Got: 0x%h. ACK: %b", port, addr, prev_data_i[addr], wb_data_o[port], wb_ack_o[port]);
    end

    wb_stb_i[port] = 1'b0;
    @(negedge clk_i);

endtask

task automatic test_masked_read_write(
    input                   port,
    input [`ADDR_WIDTH-1:0] num_read_writes
);

    logic [`DATA_WIDTH-1:0] read_data, saved_data;
    logic [`ADDR_WIDTH-1:0] addr = 0;
    logic [`DATA_WIDTH-1:0] prev_data_i [$];

    $display("Starting test_simple_read_write for port %0d with %0d operations.", port, num_read_writes);

    // Check that ack is low and not stalling before we start
    assert (wb_ack_o[port] == 1'b0)
      else $error("Port %0d: ACK is unexpectedly high before starting operations.", port);
    assert (wb_stall_o[port] == 1'b0)
      else $error("Port %0d: STALL is unexpectedly high before starting operations.", port);


    $display("Port %0d: Starting Write Phase...", port);
    wb_stb_i[port] = 1'b1;
    for (addr = 0; addr < num_read_writes; addr++) begin
        wb_addr_i[port] = addr;
        wb_we_i[port]   = addr[3:0];
        wb_data_i[port] = $urandom;  
        prev_data_i.push_back(wb_data_i[port]);
        @(negedge clk_i);
        $display("Port %0d: Writing 0x%h to address 0x%h, with write setting 0x%h", port, wb_data_i[port], wb_addr_i[port], wb_we_i[port]);
        assert (wb_ack_o[port] == '1)
            else $error("test_simple_read_write: output should be valid, failed at iteration: %d", addr);
        assert (wb_stall_o[port] == '0)
            else $error("test_simple_read_write: stalling without other writer, failed at iteration: %d", addr);   
    end

    wb_we_i[port] = 0;
    for (addr = 0; addr < num_read_writes; addr++) begin
        $display("Port %0d: Reading from address 0x%h", port, wb_addr_i[port]);
        wb_addr_i[port] = addr;
        @(negedge clk_i);

        assert (wb_ack_o[port] == 1'b1)
            else $error("Port %0d: Read ACK not received for addr 0x%h. wb_ack_o = %b", port, addr, wb_ack_o[port]);
        assert (wb_stall_o[port] == 1'b0)
            else $error("Port %0d: STALL asserted during read from addr 0x%h.", port, addr);

        // mask output based on written bits
        read_data[7:0]   = {8{addr[0]}} & wb_data_o[port][7:0];
        read_data[15:8]  = {8{addr[1]}} & wb_data_o[port][15:8];
        read_data[23:16] = {8{addr[2]}} & wb_data_o[port][23:16];
        read_data[31:24] = {8{addr[3]}} & wb_data_o[port][31:24];
        
        saved_data        = prev_data_i[addr];
        saved_data[7:0]   = {8{addr[0]}} & saved_data[7:0];
        saved_data[15:8]  = {8{addr[1]}} & saved_data[15:8];
        saved_data[23:16] = {8{addr[2]}} & saved_data[23:16];
        saved_data[31:24] = {8{addr[3]}} & saved_data[31:24];

        assert (read_data == saved_data)
            else $error("Port %0d: Read-write MISMATCH at addr 0x%h. Expected: 0x%h, Received: 0x%h", port, addr, saved_data, read_data);
        
        $display("Port %0d: Read from addr 0x%h, Expected: 0x%h, Got: 0x%h. ACK: %b", port, addr, saved_data, read_data, wb_ack_o[port]);
    end

    wb_stb_i[port] = 1'b0;
    @(negedge clk_i);

endtask


// --- Arbitration Test Task and Helper ---
task automatic test_arbitration_read_write (
    input int num_ops_per_port,
    input logic target_ram_idx
);

    logic [`DATA_WIDTH-1:0] portA_data_q[$];
    logic [`DATA_WIDTH-1:0] portB_data_q[$];

    // --- WRITE PHASE ---
    int portA_index = 0;
    int portB_index = 0;
    logic portA_wip = 0;
    logic portB_wip = 0;

    $display("Starting WRITE phase for both ports. RAM=%0d, Ops/port=%0d", target_ram_idx, num_ops_per_port);

    wb_stb_i = '{1'b1, 1'b1};
    wb_we_i = '{4'hF, 4'hF};

    while (portA_index < num_ops_per_port || portB_index < num_ops_per_port) begin

        // --- Port A Write ---
        if (portA_index < num_ops_per_port) begin
            logic [`ADDR_WIDTH-1:0] addr = {target_ram_idx, 1'b0, portA_index[(`ADDR_WIDTH-3):0]};
            wb_addr_i[0] = addr;

            if (!portA_wip) begin
                logic [`DATA_WIDTH-1:0] data = $urandom();
                wb_data_i[0] = data;
                portA_data_q.push_back(data);
                portA_wip = 1;
                $display("Port A: WRITE 0x%h to addr 0x%h", data, addr);
            end

            if (wb_ack_o[0]) begin
                portA_index++;
                portA_wip = 0;
            end
        end

        // --- Port B Write ---
        if (portB_index < num_ops_per_port) begin
            logic [`ADDR_WIDTH-1:0] addr = {target_ram_idx, 1'b1, portB_index[(`ADDR_WIDTH-3):0]};
            wb_addr_i[1] = addr;

            if (!portB_wip) begin
                logic [`DATA_WIDTH-1:0] data = $urandom();
                wb_data_i[1] = data;
                portB_data_q.push_back(data);
                portB_wip = 1;
                $display("Port B: WRITE 0x%h to addr 0x%h", data, addr);
            end

            if (wb_ack_o[1]) begin
                portB_index++;
                portB_wip = 0;
            end
        end

        @(negedge clk_i);
    end

    // Clean up write phase
    wb_stb_i = '{1'b0, 1'b0};
    wb_we_i  = '{4'h0, 4'h0};
    wb_data_i = '{'0, '0};
    wb_addr_i = '{'0, '0};
    @(negedge clk_i);

    // --- READ PHASE ---

    $display("Starting READ phase for both ports. RAM=%0d, Ops/port=%0d", target_ram_idx, num_ops_per_port);

    portA_index = 0;
    portB_index = 0;
    portA_wip = 0;
    portB_wip = 0;

    wb_stb_i = '{1'b1, 1'b1};
    wb_we_i = '{4'h0, 4'h0};
    wb_data_i = '{'0, '0};

    while (portA_index < num_ops_per_port || portB_index < num_ops_per_port) begin

        // --- Port A Read ---
        if (portA_index < num_ops_per_port) begin
            logic [`ADDR_WIDTH-1:0] addr = {target_ram_idx, 1'b0, portA_index[(`ADDR_WIDTH-3):0]};
            wb_addr_i[0] = addr;

            $display("Port A: READ request to addr 0x%h", addr);

            if (wb_ack_o[0]) begin
                assert(wb_data_o[0] == portA_data_q[portA_index])
                    else $error("Port A: READ MISMATCH at %0d: expected 0x%h, got 0x%h",
                                portA_index, portA_data_q[portA_index], wb_data_o[0]);
                portA_index++;
            end
        end

        // --- Port B Read ---
        if (portB_index < num_ops_per_port) begin
            logic [`ADDR_WIDTH-1:0] addr = {target_ram_idx, 1'b1, portB_index[(`ADDR_WIDTH-3):0]};
            wb_addr_i[1] = addr;

            $display("Port B: READ request to addr 0x%h", addr);

            if (wb_ack_o[1]) begin
                assert(wb_data_o[1] == portB_data_q[portB_index])
                    else $error("Port B: READ MISMATCH at %0d: expected 0x%h, got 0x%h",
                                portB_index, portB_data_q[portB_index], wb_data_o[1]);
                portB_index++;
            end
        end

        @(negedge clk_i);
    end

    // Final cleanup
    wb_stb_i = '{1'b0, 1'b0};
    wb_addr_i = '{'0, '0};
    wb_data_i = '{'0, '0};

    @(negedge clk_i);
    $display("Finished WRITE + READ test for RAM %0d", target_ram_idx);

endtask



endmodule