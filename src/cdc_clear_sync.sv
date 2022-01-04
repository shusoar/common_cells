//-----------------------------------------------------------------------------
// Title         : CDC Clear Signaling Synchronization
//-----------------------------------------------------------------------------
// File          : cdc_clear_propagator.sv
// Author        : Manuel Eggimann  <meggimann@iis.ee.ethz.ch>
// Created       : 22.12.2021
//-----------------------------------------------------------------------------
// Description :
//
// This module is mainly used internally to synchronize the clear requests
// between both sides of a CDC module. It aims to solve the problem of
// initiating a CDC clear, reset one-sidedly without running into
// reset-domain-crossing issues and breaking CDC protocol assumption.
//
// Problem Formulation:
//
// CDC implementations usually face the issue that one side of the CDC must not
// be cleared without clearing the other side. E.g. clearing the write-pointer
// without clearing the read-pointer in a gray-counting CDC FIFO results in an
// invalid fill-state an may cause spurious transactions of invalid data to be
// propagated accross the CDC. A similar effect is caused in 2-phase CDC
// implementations.
//
// A naive mitigation technique would be to reset both domains asynchronously
// with the same reset signal. This will cause intra-clock domain RDC issues
// since the asynchronous clear event (assertion of the reset signal) might
// happen close to the active edge of the CDC's periphery and thus might induce
// metastability. A better, but still flawed approach would be to naively
// synchronize assertion AND deassertion (the usual rst sync only synchronize
// deassertion) of the resets into the respective other domain. However, this
// might cause the classic issue of fast-to-slow clock domain crossing where the
// clear signal is not asserted long enough to be captured by the receiving
// side. The common mitigation strategy is to use a feedback acknowledge signal
// to handshake the reset request into the other domain. One even more peculiar
// corner case this approach might suffer is the scenario where the synchronized
// clear signal arrives at the other side of the CDC within or even worse after
// the same clock cylce that the other domain crossing signals (e.g. read/write
// pointers) are cleared. In this scenario, multiple signals change within the
// same clock cycle and due to metastability we cannot be sure, that the other
// side of the CDC sees the reset assertion before the first bits of e.g. the
// write/read pointer start to swith to their reset state.
//
// How this Module Works
//
// This module implements the task of bi-directiona synchronization of all
// synchronous clear or asynchronous reset requests from one side into the
// respective other side of a CDC. It uses the above mentioned request
// acknowledge handshaking for the clock domain crossing. However, it ensures
// that the clear request arrives at the other side of the CDC (given that max
// path delay of the crossing is constrained to min(t_src, t_dst)) before any
// other signals changes state.
//
// How to Use It
//
// Instantiate the module within your CDC and connect src/dst_clk_i, the
// asyncrhonous src/dst_rst_ni and the synchronous src/dst_clear_i signals.
// PARAMETRIZE the number of synchronization stages (for metastability
// resoultion) to be equal or less than the latency of the CDC. E.g. if your CDC
// uses 3 Sync Stages, parametrize this module with SYNC_STAGES <= 3! Your CDC
// must implement a src/dst_clear_i port that SYNCHRONOUSLY clears all FFs on
// the respective side. Connect the CDC's clear port to this module's
// src/dst_clear_o port. Ensure, that neither side of the CDC accepts new
// requests while the respective src/dst_clear_o signal is asserted. I.e. gate
// the src_ready_o signal of your CDC with the src_clear_o signal from this
// module.
//
// -----------------------------------------------------------------------------
// Copyright (C) 2021 ETH Zurich, University of Bologna Copyright and related
// rights are licensed under the Solderpad Hardware License, Version 0.51 (the
// "License"); you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law or
// agreed to in writing, software, hardware and materials distributed under this
// License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS
// OF ANY KIND, either express or implied. See the License for the specific
// language governing permissions and limitations under the License.
// SPDX-License-Identifier: SHL-0.51
// -----------------------------------------------------------------------------

module cdc_clear_sync #(
  /// The number of synchronization stages to use for the
  //clear signal request/acknowledge. Must be less or
  //equal to the number of sync stages used in the CDC
  parameter int unsigned SYNC_STAGES = 3,
  /// Whether an asynchronous reset shall cause a clear
  /// request to be sent to the other side.
   parameter logic CLEAR_ON_ASYNC_RESET = 1'b1
)(
  // Side A (both sides are symmetric)
  input logic  a_clk_i,
  input logic  a_rst_ni,
  input logic  a_clear_i,
  output logic a_clear_o,
  // Side B (both sides are symmetric)
  input logic  b_clk_i,
  input logic  b_rst_ni,
  input logic  b_clear_i,
  output logic b_clear_o
);

  // Closed-loop synchronization of single-bit clear signal into respective
  // other clock domain. We first latch the clear signal in the sender domain
  // into a register to filter out any glitches. Then we send the clear request
  // to the other domain using an N-stage flip-flop synchronizer to resolve
  // metastability. Finally we send an ack back to ensure the clear request
  // was sampled by the other side.
  logic        s_a2b_clear_req_d, s_a2b_clear_req_q;
  logic        s_a2b_clear_req_synced2b;
  logic        s_b2a_clear_ack_q;
  logic        s_b2a_clear_ack_synced2a;

  logic        s_b2a_clear_req_d, s_b2a_clear_req_q;
  logic        s_b2a_clear_req_synced2a;
  logic        s_a2b_clear_ack_q;
  logic        s_a2b_clear_ack_synced2b;

  //-------------------------- Side A --------------------------

  // Assert the clear request signal if there is a new (we are not already
  // clearing) clear event or if a pending clear request was not yet
  // acknowledged.
  assign s_a2b_clear_req_d = (a_clear_i & !a_clear_o) | (s_a2b_clear_req_q & !s_b2a_clear_ack_synced2a);

  // Latch the clear request signal before sending it to the other domain
  always_ff @(posedge a_clk_i, negedge a_rst_ni) begin
    if (!a_rst_ni) begin
      s_a2b_clear_req_q <= CLEAR_ON_ASYNC_RESET;
    end else begin
      s_a2b_clear_req_q <= s_a2b_clear_req_d;
    end
  end
  // Synchronize the request signal into the other domain
  sync #(.STAGES(SYNC_STAGES)) i_a2b_clear_req_synchronizer(
    .clk_i(b_clk_i),
    .rst_ni(b_rst_ni),
    // Send the glitch filtered clear signal or the async (and thus hopefully
    // glitch-free asynchronous reset) to the other domain
    .serial_i(s_a2b_clear_req_q),
    .serial_o(s_a2b_clear_req_synced2b)
  );

  // Delay the acknowledge by one more cycle to ensure we cleared our side
  // before we send the ack back.
  always_ff @(posedge b_clk_i, negedge b_rst_ni) begin
    if (!b_rst_ni) begin
      s_b2a_clear_ack_q <= 1'b0;
    end else begin
      s_b2a_clear_ack_q <= s_a2b_clear_req_synced2b;
    end
  end

  // Synchronize the ack signal back into the sending domain
  sync #(.STAGES(SYNC_STAGES)) i_b2a_clear_ack_synchronizer(
    .clk_i(a_clk_i),
    .rst_ni(a_rst_ni),
    .serial_i(s_b2a_clear_ack_q),
    .serial_o(s_b2a_clear_ack_synced2a)
  );

  // Clear the source if there is a clear request from either the source side or
  // the destination side and keep it asserted until the acknowledge signal is deasserted.
  assign a_clear_o = s_a2b_clear_req_q | s_b2a_clear_req_synced2a | s_b2a_clear_ack_synced2a;


  //-------------------------- Side B --------------------------

  // Assert the clear request signal if there is a new clear event (we are not
  // already clearing) or if a pending clear request was not yet acknowledged.
  assign s_b2a_clear_req_d = (b_clear_i & !b_clear_o) | (s_b2a_clear_req_q & !s_a2b_clear_ack_synced2b);

  // Latch the clear request signal before sending it to the other domain
  always_ff @(posedge b_clk_i, negedge b_rst_ni) begin
    if (!b_rst_ni) begin
      s_b2a_clear_req_q <= 1'b1;  // Start a request when we received an async reset.
    end else begin
      s_b2a_clear_req_q <= s_b2a_clear_req_d;
    end
  end
  // Synchronize the request signal into the other domain
  sync #(.STAGES(SYNC_STAGES)) i_b2a_clear_req_synchronizer(
    .clk_i(a_clk_i),
    .rst_ni(a_rst_ni),
    // Send the glitch filtered clear signal or the async (and thus hopefully
    // glitch-free asynchronous reset) to the other domain
    .serial_i(s_b2a_clear_req_q),
    .serial_o(s_b2a_clear_req_synced2a)
  );

  // Delay the acknowledge by one more cycle to ensure we cleared our side
  // before we send the ack back.
  always_ff @(posedge a_clk_i, negedge a_rst_ni) begin
    if (!a_rst_ni) begin
      s_a2b_clear_ack_q <= 1'b0;
    end else begin
      s_a2b_clear_ack_q <= s_b2a_clear_req_synced2a;
    end
  end

  sync #(.STAGES(SYNC_STAGES)) i_a2b_clear_ack_synchronizer(
    .clk_i(b_clk_i),
    .rst_ni(b_rst_ni),
    .serial_i(s_a2b_clear_ack_q),
    .serial_o(s_a2b_clear_ack_synced2b)
  );

  // Clear the source if there is a clear request from either the source side or
  // the destination side and keep it asserted until the acknowledge signal is deasserted.
  assign b_clear_o = s_b2a_clear_req_q | s_a2b_clear_req_synced2b | s_a2b_clear_ack_synced2b;

`ifndef VERILATOR
  sequence clear_o_eventually_asserts(clear_o);
    !clear_o [*] ##1 clear_o;
  endsequence

  // Assert that if the clear_i signal on side asserts, the clear_o of the same
  // side asserts eventually and stays asserted until the clear_o on the other
  // side is asserted.
  property clear_i_asserts_clear_o_until_other_clear_o_asserts(src_clk_i, src_clear_i, src_clear_o, dst_clk_i, dst_clear_o);
    @(posedge src_clk_i) $rose(src_clear_i) & !src_clear_o |=> src_clear_o ##1 @(posedge dst_clk_i) src_clear_o throughout clear_o_eventually_asserts(dst_clear_o)
  endproperty

  a2b_clear_sync: assert property (clear_i_asserts_clear_o_until_other_clear_o_asserts(a_clk_i, a_clear_i, a_clear_o, b_clk_i, b_clear_o));

  b2a_clear_sync: assert property (clear_i_asserts_clear_o_until_other_clear_o_asserts(b_clk_i, b_clear_i, b_clear_o, a_clk_i, a_clear_o));
`endif	
endmodule
