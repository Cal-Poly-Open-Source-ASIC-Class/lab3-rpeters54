
`include "ram_defines.svh"

module tb_all;

logic                    clk_i;
logic [`ADDR_WIDTH-1:0]  pA_wb_addr_i;      
logic [`ADDR_WIDTH-1:0]  pB_wb_addr_i;
logic                    pA_wb_stb_i;      
logic                    pB_wb_stb_i;      
logic [3:0]              pA_wb_we_i;
logic [3:0]              pB_wb_we_i;
logic [`DATA_WIDTH-1:0]  pA_wb_data_i;
logic [`DATA_WIDTH-1:0]  pB_wb_data_i;

logic [`DATA_WIDTH-1:0]  p0_dffram_data_out_i;
logic [`DATA_WIDTH-1:0]  p1_dffram_data_out_i;

logic                    pA_wb_ack_o;
logic                    pB_wb_ack_o;
logic                    pA_wb_stall_o;
logic                    pB_wb_stall_o;
logic [`DATA_WIDTH-1:0]  pA_wb_data_o;
logic [`DATA_WIDTH-1:0]  pB_wb_data_o;

logic [3:0]              p0_dffram_we_o;
logic [3:0]              p1_dffram_we_o;
logic                    p0_dffram_en_o;   
logic                    p1_dffram_en_o;   
logic [`DATA_WIDTH-1:0]  p0_dffram_data_in_o;
logic [`DATA_WIDTH-1:0]  p1_dffram_data_in_o;
logic [`ADDR_WIDTH-2:0]  p0_dffram_addr_o;
logic [`ADDR_WIDTH-2:0]  p1_dffram_addr_o;

ram_arbiter arbiter(
    .*
);

DFFRAM256x32 ram0 (
        clk_i,
        p0_dffram_we_o,
        p0_dffram_en_o,
        p0_dffram_data_in_o,
        p0_dffram_data_out_i,
        p0_dffram_addr_o
);

DFFRAM256x32 ram1 (
        clk_i,
        p1_dffram_we_o,
        p1_dffram_en_o,
        p1_dffram_data_in_o,
        p1_dffram_data_out_i,
        p1_dffram_addr_o
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

// doing the same for the DFFRAM connections
logic [`DATA_WIDTH-1:0] dffram_data_out_i [0:1];
logic                   dffram_en_o       [0:1];
logic [3:0]             dffram_we_o       [0:1];
logic [`DATA_WIDTH-1:0] dffram_data_in_o  [0:1];
logic [`ADDR_WIDTH-2:0] dffram_addr_o     [0:1];

assign dffram_data_out_i    = '{p0_dffram_data_out_i, p1_dffram_data_out_i};
assign dffram_we_o          = '{p0_dffram_we_o, p1_dffram_we_o};
assign dffram_en_o          = '{p0_dffram_en_o, p1_dffram_en_o};
assign dffram_data_in_o     = '{p0_dffram_data_in_o, p1_dffram_data_in_o};
assign dffram_addr_o        = '{p0_dffram_addr_o, p1_dffram_addr_o};


// Sample to drive clock
localparam PERIOD = 10;
initial begin
clk_i = 0;
forever #(PERIOD/2) clk_i = ~clk_i;
end

// Necessary to create Waveform
initial begin
    // Name as needed
    $dumpfile("tb_all.vcd");
    $dumpvars(0);
end

initial begin
    init();
    
    $display("[%0t] Starting Single Port Tests...", $time);
    test_simple_read_write(0, 10);
    test_simple_read_write(1, 10);
    
    $display("[%0t] Starting Arbitration Tests...", $time);
    test_arbitration_read_write(10, 0); // 10 ops per port, contending for RAM0
    test_arbitration_read_write(10, 1); // 10 ops per port, contending for RAM1
    
    $display("[%0t] All tests finished.", $time);
    $finish();
end

// initialize ram
task automatic init();
    foreach (wb_addr_i[i]) wb_addr_i[i] = '0;
    foreach (wb_stb_i[i])  wb_stb_i[i]  = 1'b0;
    foreach (wb_we_i[i])   wb_we_i[i]   = 4'b0000;
    foreach (wb_data_i[i]) wb_data_i[i] = '0;

    repeat (2) @(negedge clk_i);
endtask

task automatic test_simple_read_write(
    input                   port,
    input [`ADDR_WIDTH-1:0] num_read_writes
);

    logic [`ADDR_WIDTH-1:0] addr = 0;
    logic [`DATA_WIDTH-1:0]  prev_data_i [$];

    $display("[%0t] Starting test_simple_read_write for port %0d with %0d operations.", $time, port, num_read_writes);

    // Check that ack is low and not stalling before we start
    assert (wb_ack_o[port] == 1'b0)
      else $error("[%0t] Port %0d: ACK is unexpectedly high before starting operations.", $time, port);
    assert (wb_stall_o[port] == 1'b0)
      else $error("[%0t] Port %0d: STALL is unexpectedly high before starting operations.", $time, port);


    $display("[%0t] Port %0d: Starting Write Phase...", $time, port);
    wb_stb_i[port] = 1'b1;
    wb_addr_i[port] = addr;
    wb_we_i[port]   = 4'hF; 
    wb_data_i[port] = $urandom();  
    prev_data_i.push_back(wb_data_i[port]);
    @(negedge clk_i);
    $display("[%0t] Port %0d: Writing 0x%h to address 0x%h", $time, port, wb_data_i[port], wb_addr_i[port]);

    assert (wb_ack_o[port] == '0)
        else $error("test_simple_read_write: ack set before pipeline finished");
    assert (wb_stall_o[port] == '0)
        else $error("test_simple_read_write: stalling without other writer");    

    for (addr = 1; addr < num_read_writes; addr++) begin
        wb_addr_i[port] = addr;
        wb_we_i[port]   = 4'hF; 
        wb_data_i[port] = $urandom;  
        prev_data_i.push_back(wb_data_i[port]);
        @(negedge clk_i);
        $display("[%0t] Port %0d: Writing 0x%h to address 0x%h", $time, port, wb_data_i[port], wb_addr_i[port]);
        assert (wb_ack_o[port] == '1)
            else $error("test_simple_read_write: output should be valid, failed at iteration: %d", addr);
        assert (wb_stall_o[port] == '0)
            else $error("test_simple_read_write: stalling without other writer, failed at iteration: %d", addr);   
    end

    wb_we_i[port] = 0;
    wb_addr_i[port] = 0;
    @(negedge clk_i);

    for (addr = 1; addr < num_read_writes; addr++) begin
        $display("[%0t] Port %0d: Reading from address 0x%h", $time, port, wb_addr_i[port]);
        wb_addr_i[port] = addr;
        @(negedge clk_i);

        assert (wb_ack_o[port] == 1'b1)
            else $error("[%0t] Port %0d: Read ACK not received for addr 0x%h. wb_ack_o = %b", $time, port, addr-1, wb_ack_o[port]);
        assert (wb_stall_o[port] == 1'b0)
            else $error("[%0t] Port %0d: STALL asserted during read from addr 0x%h.", $time, port, addr-1);


        assert (wb_data_o[port] == prev_data_i[addr-1])
            else $error("[%0t] Port %0d: Read-write MISMATCH at addr 0x%h. Expected: 0x%h, Received: 0x%h", $time, port, addr-1, prev_data_i[addr-1], wb_data_o[port]);
        
        $display("[%0t] Port %0d: Read from addr 0x%h, Expected: 0x%h, Got: 0x%h. ACK: %b", $time, port, addr-1, prev_data_i[addr-1], wb_data_o[port], wb_ack_o[port]);
    end

    wb_stb_i[port] = 1'b0;
    @(negedge clk_i);

    assert (wb_ack_o[port] == 1'b1)
        else $error("[%0t] Port %0d: Read ACK not received for addr 0x%h. wb_ack_o = %b", $time, port, addr-1, wb_ack_o[port]);
    assert (wb_stall_o[port] == 1'b0)
        else $error("[%0t] Port %0d: STALL asserted during read from addr 0x%h.", $time, port, addr-1);


    assert (wb_data_o[port] == prev_data_i[addr-1])
        else $error("[%0t] Port %0d: Read-write MISMATCH at addr 0x%h. Expected: 0x%h, Received: 0x%h", $time, port, addr-1, prev_data_i[addr-1], wb_data_o[port]);
    
    $display("[%0t] Port %0d: Read from addr 0x%h, Expected: 0x%h, Got: 0x%h. ACK: %b", $time, port, addr-1, prev_data_i[addr-1], wb_data_o[port], wb_ack_o[port]);

endtask


  // --- Arbitration Test Task and Helper ---

  task automatic do_port_operations (
      input int port_id,                 // 0 for Port A, 1 for Port B
      input int num_ops,                 // Number of operations (writes or reads)
      input bit target_ram_idx,          // 0 for RAM0, 1 for RAM1
      input logic is_write,              // 1 for write phase, 0 for read phase
      inout int ops_completed_count,     // Counter for completed operations
      inout int stalled_cycles_count,    // Counter for stall cycles
      inout logic [`DATA_WIDTH-1:0] data_q[$] // Queue for data: write stores here, read verifies from here
  );
      logic [`ADDR_WIDTH-1:0] current_addr;
      logic [`DATA_WIDTH-1:0] current_data_val; // Renamed to avoid conflict with Verilog keyword
      logic [`DATA_WIDTH-1:0] read_data_val;    // Renamed
      logic ack_received_for_current_op;

      if (num_ops == 0) return;

      $display("[%0t] Port %0d: Starting %s phase for %0d ops to RAM %0d.",
                $time, port_id, (is_write ? "Write" : "Read"), num_ops, target_ram_idx);

      // Keep STB high for the duration of this phase's operations for this port
      wb_stb_i[port_id] = 1'b1;
      wb_we_i[port_id]  = is_write ? 4'hF : 4'b0;

      for (int i = 0; i < num_ops; i++) begin
          // Determine base offset for addresses within the target RAM block
          // Port 0 uses addresses 0 to num_ops-1
          // Port 1 uses addresses num_ops to 2*num_ops-1
          // This ensures they target different memory locations within the same RAM block.
          int base_offset_in_ram_block = (port_id == 0) ? 0 : num_ops;
          logic [`ADDR_WIDTH-2:0] addr_in_ram_block = base_offset_in_ram_block[7:0] + i[7:0];

          // Construct the full address
          current_addr = {target_ram_idx,{(`ADDR_WIDTH - 1){1'b0}}} | {1'b0, addr_in_ram_block};

          if (is_write) begin
              current_data_val = $urandom();
              if (i >= data_q.size()) data_q.push_back(current_data_val); // Store data written
              else data_q[i] = current_data_val; // Overwrite if re-using (shouldn't happen with push_back logic for writes)
              wb_data_i[port_id] = current_data_val;
              $display("[%0t] Port %0d (Write): Op %0d/%0d, Addr=0x%h, Data=0x%h",
                        $time, port_id, i+1, num_ops, current_addr, current_data_val);
          end else begin // is_read
              current_data_val = data_q[i]; // Expected data from the write phase
              $display("[%0t] Port %0d (Read): Op %0d/%0d, Addr=0x%h, Expect=0x%h",
                        $time, port_id, i+1, num_ops, current_addr, current_data_val);
          end
          
          wb_addr_i[port_id] = current_addr;
          // STB and WE are already set for the phase.

          // 1. Wait one cycle for the request to be presented to the arbiter.
          @(negedge clk_i);

          // 2. Loop until ACK is received for the current operation.
          ack_received_for_current_op = 1'b0;
          while (!ack_received_for_current_op) begin
              if (wb_stall_o[port_id]) begin
                  stalled_cycles_count++;
                  $display("[%0t] Port %0d: STALLED. Op %0d/%0d, Addr=0x%h", $time, port_id, i+1, num_ops, current_addr);
                  // Keep request asserted during stall
                  wb_stb_i[port_id] = 1'b1; // Re-assert to be safe
                  wb_we_i[port_id]  = is_write ? 4'hF : 4'b0; // Re-assert WE
                  wb_addr_i[port_id] = current_addr; // Re-assert address
                  if(is_write) wb_data_i[port_id] = current_data_val; // Re-assert data

                  @(negedge clk_i); // Consume cycle while stalled
              end else begin
                  // Not stalled. Request was accepted by arbiter on the *previous* clock edge.
                  // ACK should be asserted on *this current* clock edge.
                  if (wb_ack_o[port_id]) begin
                      $display("[%0t] Port %0d: ACKED. Op %0d/%0d, Addr=0x%h", $time, port_id, i+1, num_ops, current_addr);
                      if (!is_write) begin
                          read_data_val = wb_data_o[port_id];
                          assert(read_data_val == current_data_val)
                              else $error("[%0t] Port %0d: Read MISMATCH Op %0d/%0d, Addr=0x%h. Expected=0x%h, Got=0x%h",
                                          $time, port_id, i+1, num_ops, current_addr, current_data_val, read_data_val);
                      end
                      ops_completed_count++;
                      ack_received_for_current_op = 1'b1;
                      // ACK received, loop will terminate.
                      // The next @(negedge clk_i) will be at the start of the next iteration of the outer 'for' loop,
                      // or after the 'for' loop if this was the last operation.
                  end else begin
                      // Not stalled, STB is high, but no ACK. This is an error condition.
                      $error("[%0t] Port %0d: NOT STALLED but NO ACK. Op %0d/%0d, Addr=0x%h. wb_stb=%b, wb_ack=%b",
                                $time, port_id, i+1, num_ops, current_addr, wb_stb_i[port_id], wb_ack_o[port_id]);
                      ack_received_for_current_op = 1'b1; // Break loop on error to prevent infinite simulation
                  end
                  // If ACK was not received (error), we still need to advance time if we were to continue,
                  // but since we break, the outer loop's @(negedge clk_i) will handle it.
                  // If ACK was received, we also break and let outer loop handle.
                  // No @(negedge clk_i) here.
              end
          end // end while !ack_received_for_current_op
      end // end for (int i = 0; i < num_ops; i++)

      // After all operations for this phase for this port are done:
      wb_stb_i[port_id] = 1'b0;
      // wb_we_i[port_id] is effectively don't care now that STB is low, but good to clear
      wb_we_i[port_id]  = 4'b0;
      // Allow one cycle for de-assertion to propagate if the other thread is still running.
      @(negedge clk_i);
      $display("[%0t] Port %0d: Finished %s phase.", $time, port_id, (is_write ? "Write" : "Read"));
  endtask


  task automatic test_arbitration_read_write(
      input int num_ops_per_port,
      input bit target_ram_idx // 0 for RAM0, 1 for RAM1
  );
      int portA_writes_done = 0;
      int portA_reads_done = 0;
      int portA_stalled_cycles = 0;
      logic [`DATA_WIDTH-1:0] portA_written_data[$];

      int portB_writes_done = 0;
      int portB_reads_done = 0;
      int portB_stalled_cycles = 0;
      logic [`DATA_WIDTH-1:0] portB_written_data[$];

      $display("[%0t] Starting test_arbitration_read_write: %0d ops/port to RAM %0d", $time, num_ops_per_port, target_ram_idx);

      // Ensure ports start with STB low before forking threads
      wb_stb_i[0] = 1'b0;
      wb_stb_i[1] = 1'b0;
      @(negedge clk_i);

      fork
          begin : portA_thread
              // Port A Write Phase
              do_port_operations(0, num_ops_per_port, target_ram_idx, 1'b1, portA_writes_done, portA_stalled_cycles, portA_written_data);
              // Port A Read Phase
              do_port_operations(0, num_ops_per_port, target_ram_idx, 1'b0, portA_reads_done,  portA_stalled_cycles, portA_written_data);
          end
          begin : portB_thread
              // Port B Write Phase
              do_port_operations(1, num_ops_per_port, target_ram_idx, 1'b1, portB_writes_done, portB_stalled_cycles, portB_written_data);
              // Port B Read Phase
              do_port_operations(1, num_ops_per_port, target_ram_idx, 1'b0, portB_reads_done,  portB_stalled_cycles, portB_written_data);
          end
      join

      // Final Assertions
      assert(portA_writes_done == num_ops_per_port)
          else $error("Port A: Write ops mismatch. Expected %0d, Got %0d", num_ops_per_port, portA_writes_done);
      assert(portA_reads_done  == num_ops_per_port)
          else $error("Port A: Read ops mismatch. Expected %0d, Got %0d", num_ops_per_port, portA_reads_done);
      assert(portB_writes_done == num_ops_per_port)
          else $error("Port B: Write ops mismatch. Expected %0d, Got %0d", num_ops_per_port, portB_writes_done);
      assert(portB_reads_done  == num_ops_per_port)
          else $error("Port B: Read ops mismatch. Expected %0d, Got %0d", num_ops_per_port, portB_reads_done);

      $display("[%0t] Arbitration Test (RAM %0d) Results:", $time, target_ram_idx);
      $display("  Port A: Writes=%0d, Reads=%0d, Total Ops=%0d, Stalls=%0d",
                portA_writes_done, portA_reads_done, portA_writes_done + portA_reads_done, portA_stalled_cycles);
      $display("  Port B: Writes=%0d, Reads=%0d, Total Ops=%0d, Stalls=%0d",
                portB_writes_done, portB_reads_done, portB_writes_done + portB_reads_done, portB_stalled_cycles);

      if (num_ops_per_port > 1) begin // More likely to see stalls if more ops and true contention
          // This is a heuristic. If one port finishes very quickly before the other starts its contentious phase,
          // stalls might be low for one. True equity means both get through and stalls are distributed if they overlap.
          if (portA_stalled_cycles == 0 && portB_stalled_cycles == 0 && num_ops_per_port > 2) begin // Be more lenient for few ops
             $warning("[%0t] Arbitration Test (RAM %0d): No stalls recorded for either port with %0d ops/port. Check for true contention.",
                       $time, target_ram_idx, num_ops_per_port);
          end
      end
      $display("[%0t] test_arbitration_read_write (RAM %0d) finished.", $time, target_ram_idx);
  endtask


endmodule