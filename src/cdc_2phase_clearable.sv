// Copyright 2018 ETH Zurich and University of Bologna.
//
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Fabian Schuiki <fschuiki@iis.ee.ethz.ch>
// Manuel Eggimann <meggiman@iis.ee.ethz.ch>

/// A two-phase clock domain crossing.
///
/// CONSTRAINT: Requires max_delay of min_period(src_clk_i, dst_clk_i) through
/// the paths async_req, async_ack, async_data.
/* verilator lint_off DECLFILENAME */
module cdc_2phase_clearable #(
  parameter type T = logic,
  parameter int unsigned SYNC_STAGES = 2
)(
  input  logic src_rst_ni,
  input  logic src_clk_i,
  input  logic src_clear_i,
  input  T     src_data_i,
  input  logic src_valid_i,
  output logic src_ready_o,

  input  logic dst_rst_ni,
  input  logic dst_clk_i,
  input  logic dst_clear_i,
  output T     dst_data_o,
  output logic dst_valid_o,
  input  logic dst_ready_i
);



  // Asynchronous handshake signals.
  (* dont_touch = "true" *) logic async_req;
  (* dont_touch = "true" *) logic async_ack;
  (* dont_touch = "true" *) T async_data;

  // The sender in the source domain.
  cdc_2phase_src_clearable #(
    .T           ( T           ),
    .SYNC_STAGES ( SYNC_STAGES )
  ) i_src (
    .rst_ni       ( src_rst_ni  ),
    .clk_i        ( src_clk_i   ),
    .clear_i      ( src_clear_i ),
    .data_i       ( src_data_i  ),
    .valid_i      ( src_valid_i ),
    .ready_o      ( src_ready_o ),
    .async_req_o  ( async_req   ),
    .async_ack_i  ( async_ack   ),
    .async_data_o ( async_data  )
  );

  // The receiver in the destination domain.
  cdc_2phase_dst_clearable #(
    .T           ( T           ),
    .SYNC_STAGES ( SYNC_STAGES )
  ) i_dst (
    .rst_ni       ( dst_rst_ni  ),
    .clk_i        ( dst_clk_i   ),
    .clear_i      ( dst_clear_i ),
    .data_o       ( dst_data_o  ),
    .valid_o      ( dst_valid_o ),
    .ready_i      ( dst_ready_i ),
    .async_req_i  ( async_req   ),
    .async_ack_o  ( async_ack   ),
    .async_data_i ( async_data  )
  );

endmodule


/// Half of the two-phase clock domain crossing located in the source domain.
module cdc_2phase_src_clearable #(
  parameter type T = logic,
  parameter int unsigned SYNC_STAGES = 2
) (
  input  logic rst_ni,
  input  logic clk_i,
  input  logic clear_i,
  input  T     data_i,
  input  logic valid_i,
  output logic ready_o,
  output logic async_req_o,
  input  logic async_ack_i,
  output T     async_data_o
);

  (* dont_touch = "true" *)
  logic  req_src_d, req_src_q, ack_synced;
  (* dont_touch = "true" *)
  T data_src_d, data_src_q;

  // Synchronize the async ACK
  sync #(
    .STAGES(SYNC_STAGES)
  ) i_sync(
    .clk_i,
    .rst_ni,
    .serial_i( async_ack_i ),
    .serial_o( ack_synced  )
  );

  // If we receive the clear signal clear the content of the request flip-flop
  // and the data register
  always_comb begin
    data_src_d = data_src_q;
    req_src_d  = req_src_q;
    if (clear_i) begin
      req_src_d  = 1'b0;
      data_src_d = '0;
    // The req_src and data_src registers change when a new data item is accepted.
    end else if (valid_i && ready_o) begin
      req_src_d  = ~req_src_q;
      data_src_d = data_i;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      req_src_q  <= 0;
      data_src_q <= '0;
    end else begin
      req_src_q  <= req_src_d;
      data_src_q <= data_src_d;
    end
  end

  // Output assignments.
  assign ready_o = (req_src_q == ack_synced);
  assign async_req_o = req_src_q;
  assign async_data_o = data_src_q;

// Assertions
// pragma translate_off
`ifndef VERILATOR
  no_clear_and_request: assume property (
     @(posedge clk_i) disable iff(~rst_ni) (clear_i |-> ~valid_i))
    else $fatal(1, "No request allowed while clear_i is asserted.");
`endif

endmodule


/// Half of the two-phase clock domain crossing located in the destination
/// domain.
module cdc_2phase_dst_clearable #(
  parameter type T = logic,
  parameter int unsigned SYNC_STAGES = 2
)(
  input  logic rst_ni,
  input  logic clk_i,
  input  logic clear_i,
  output T     data_o,
  output logic valid_o,
  input  logic ready_i,
  input  logic async_req_i,
  output logic async_ack_o,
  input  T     async_data_i
);

  (* dont_touch = "true" *)
  (* async_reg = "true" *)
 logic ack_dst_d, ack_dst_q, req_synced, req_synced_q1;
  (* dont_touch = "true" *)
  T data_dst_d, data_dst_q;


  //Synchronize the request
  sync #(
    .STAGES(SYNC_STAGES)
  ) i_sync(
    .clk_i,
    .rst_ni,
    .serial_i( async_req_i ),
    .serial_o( req_synced  )
  );

  // The ack_dst register changes when a new data item is accepted.
  always_comb begin
    ack_dst_d = ack_dst_q;
    if (clear_i) begin
      ack_dst_d = 1'b0;
    end else if (valid_o && ready_i) begin
      ack_dst_d = ~ack_dst_q;
    end
  end

  // The data_dst register samples when a new data item is presented. This is
  // indicated by a transition in the req_synced line.
  always_comb begin
    data_dst_d = data_dst_q;
    if (clear_i) begin
      data_dst_d = '0;
    end else if (req_synced != req_synced_q1 && !valid_o) begin
      data_dst_d = async_data_i;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ack_dst_q     <= 0;
      data_dst_q    <= '0;
      req_synced_q1 <= 1'b0;
    end else begin
      ack_dst_q     <= ack_dst_d;
      data_dst_q    <= data_dst_d;
      // The req_synced_q1 is the delayed version of the synchronized req_synced
      // used to detect transitions in the request.
      req_synced_q1 <= req_synced;
    end
  end

  // Output assignments.
  assign valid_o = (ack_dst_q != req_synced_q1);
  assign data_o = data_dst_q;
  assign async_ack_o = ack_dst_q;

endmodule
/* verilator lint_on DECLFILENAME */
