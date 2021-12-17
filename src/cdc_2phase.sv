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

/// A two-phase clock domain crossing. Wrapper around module with synchronous
/// clear. This wrapper exists only for backward compatibility.
///
/// CONSTRAINT: Requires max_delay of min_period(src_clk_i, dst_clk_i) through
/// the paths async_req, async_ack, async_data.
/* verilator lint_off DECLFILENAME */
module cdc_2phase #(
  parameter type T = logic,
  parameter int unsigned SYNC_STAGES = 2
)(
  input  logic src_rst_ni,
  input  logic src_clk_i,
  input  T     src_data_i,
  input  logic src_valid_i,
  output logic src_ready_o,

  input  logic dst_rst_ni,
  input  logic dst_clk_i,
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
    .clear_i      ( 1'b0        ),
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
    .clear_i      ( 1'b0        ),
    .data_o       ( dst_data_o  ),
    .valid_o      ( dst_valid_o ),
    .ready_i      ( dst_ready_i ),
    .async_req_i  ( async_req   ),
    .async_ack_o  ( async_ack   ),
    .async_data_i ( async_data  )
  );

endmodule


/// Half of the two-phase clock domain crossing located in the source domain.
module cdc_2phase_src #(
  parameter type T = logic,
  parameter int unsigned SYNC_STAGES = 2
)(
  input  logic rst_ni,
  input  logic clk_i,
  input  T     data_i,
  input  logic valid_i,
  output logic ready_o,
  output logic async_req_o,
  input  logic async_ack_i,
  output T     async_data_o
);

  cdc_2phase_src_clearable #(
    .T           ( T           ),
    .SYNC_STAGES ( SYNC_STAGES )
  ) i_src_clearable (
    .rst_ni,
    .clk_i,
    .clear_i (1'b0),
    .data_i,
    .valid_i,
    .ready_o,
    .async_req_o,
    .async_ack_i,
    .async_data_o
  );

endmodule


/// Half of the two-phase clock domain crossing located in the destination
/// domain.
module cdc_2phase_dst #(
  parameter type T = logic,
  parameter int unsigned SYNC_STAGES = 2
)(
  input  logic rst_ni,
  input  logic clk_i,
  output T     data_o,
  output logic valid_o,
  input  logic ready_i,
  input  logic async_req_i,
  output logic async_ack_o,
  input  T     async_data_i
);

  cdc_2phase_dst_clearable #(
    .T           ( T           ),
    .SYNC_STAGES ( SYNC_STAGES )
  ) i_dst_clearable (
    .rst_ni,
    .clk_i,
    .clear_i ( 1'b0 ),
    .data_o,
    .valid_o,
    .ready_i,
    .async_req_i,
    .async_ack_o,
    .async_data_i
  );
endmodule
/* verilator lint_on DECLFILENAME */
