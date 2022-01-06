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

module cdc_clear_sync
  import cdc_clear_sync_pkg::*;
 #(
  /// The number of synchronization stages to use for the
  //clear signal request/acknowledge. Must be less or
  //equal to the number of sync stages used in the CDC
  parameter int unsigned SYNC_STAGES = 2,
  /// Whether an asynchronous reset shall cause a clear
  /// request to be sent to the other side.
  parameter logic CLEAR_ON_ASYNC_RESET = 1'b1
)(
  // Side A (both sides are symmetric)
  input logic  a_clk_i,
  input logic  a_rst_ni,
  input logic  a_clear_i,
  output logic a_clear_o,
  output logic a_isolate_o,
  input logic  a_isolate_ack_i,
  // Side B (both sides are symmetric)
  input logic  b_clk_i,
  input logic  b_rst_ni,
  input logic  b_clear_i,
  output logic b_clear_o,
  output logic b_isolate_o,
  input logic  b_isolate_ack_i
);

  (* dont_touch = "true" *)
  logic        async_a2b_req, async_b2a_ack;
  (* dont_touch = "true" *)
  clear_seq_phase_e async_a2b_next_phase;
  (* dont_touch = "true" *)
  logic        async_b2a_req, async_a2b_ack;
  (* dont_touch = "true" *)
  clear_seq_phase_e async_b2a_next_phase;

  cdc_clear_sync_half #(
    .SYNC_STAGES          ( SYNC_STAGES          ),
    .CLEAR_ON_ASYNC_RESET ( CLEAR_ON_ASYNC_RESET )
  ) i_clear_sync_half_a (
    .clk_i              ( a_clk_i              ),
    .rst_ni             ( a_rst_ni             ),
    .clear_i            ( a_clear_i            ),
    .clear_o            ( a_clear_o            ),
    .isolate_o          ( a_isolate_o          ),
    .isolate_ack_i      ( a_isolate_ack_i      ),
    (* async *) .async_next_phase_o ( async_a2b_next_phase ),
    (* async *) .async_req_o        ( async_a2b_req        ),
    (* async *) .async_ack_i        ( async_b2a_ack        ),
    (* async *) .async_next_phase_i ( async_b2a_next_phase ),
    (* async *) .async_req_i        ( async_b2a_req        ),
    (* async *) .async_ack_o        ( async_a2b_ack        )
  );

    cdc_clear_sync_half #(
    .SYNC_STAGES          ( SYNC_STAGES          ),
    .CLEAR_ON_ASYNC_RESET ( CLEAR_ON_ASYNC_RESET )
  ) i_clear_sync_half_b (
    .clk_i              ( b_clk_i              ),
    .rst_ni             ( b_rst_ni             ),
    .clear_i            ( b_clear_i            ),
    .clear_o            ( b_clear_o            ),
    .isolate_o          ( b_isolate_o          ),
    .isolate_ack_i      ( b_isolate_ack_i      ),
    (* async *) .async_next_phase_o ( async_b2a_next_phase ),
    (* async *) .async_req_o        ( async_b2a_req        ),
    (* async *) .async_ack_i        ( async_a2b_ack        ),
    (* async *) .async_next_phase_i ( async_a2b_next_phase ),
    (* async *) .async_req_i        ( async_a2b_req        ),
    (* async *) .async_ack_o        ( async_b2a_ack        )
  );
endmodule


module cdc_clear_sync_half
  import cdc_clear_sync_pkg::*;
#(
  /// The number of synchronization stages to use for the
  //clear signal request/acknowledge. Must be less or
  //equal to the number of sync stages used in the CDC
  parameter int unsigned SYNC_STAGES = 2,
  /// Whether an asynchronous reset shall cause a clear
  /// request to be sent to the other side.
  parameter logic CLEAR_ON_ASYNC_RESET = 1'b1
)(
  // Synchronous side
  input logic                clk_i,
  input logic                rst_ni,
  input logic                clear_i,
  output logic               isolate_o,
  input logic                isolate_ack_i,
  output logic               clear_o,
  // Asynchronous clear sequence hanshaking
  output clear_seq_phase_e   async_next_phase_o,
  output logic               async_req_o,
  input logic                async_ack_i,
  input clear_seq_phase_e    async_next_phase_i,
  input logic                async_req_i,
  output logic               async_ack_o
);


  // How this module works:

  // FSM controls the handshaking of the initiator side i.e. the logic that
  // controls handshaking when a clear is initiated in this clock domain. The
  // FSM transitions through 3 phases during a clear event. In the ISOLATE
  // phase, the freeze signal is asserted and the connected CDCs are expected to
  // block all further interactions with the outside world with the next clock
  // cycle. In the CLEAR phase, the clear signal is asserted which resets the
  // internal state of the CDC while keeping the freeze signal asserted. In the
  // POST_CLEAR phase, the freeze signal is deasserted to continue normal operation.
  // The FSM uses a dedicated 4phase handshaking CDC to transition between the
  // phases in lock-step and transmits the current state to the other domain to
  // avoid issues if the other domain is reset asynchronously while a clear
  // procedure is pending.

  //---------------------- Initiator Side ----------------------
  // Sends clear sequence state transitions to the other side.
  typedef enum               logic[2:0] {IDLE, ISOLATE, WAIT_PHASE_ACK, WAIT_ISOLATE_ACK, CLEAR, POST_CLEAR, FINISHED} initiator_state_e;
  initiator_state_e initiator_state_d, initiator_state_q;

  // The current phase of the clear sequence, sent to the other side using a
  // 4-phase CDC
  clear_seq_phase_e          initiator_clear_seq_phase;
  logic                      initiator_phase_transition_req;
  logic                      initiator_phase_transition_ack;
  logic                      initiator_isolate_out;
  logic                      initiator_clear_out;

  always_comb begin
    initiator_state_d              = initiator_state_q;
    initiator_phase_transition_req = 1'b0;
    initiator_isolate_out          = 1'b0;
    initiator_clear_out            = 1'b0;
    initiator_clear_seq_phase      = cdc_clear_sync_pkg::IDLE;

    case (initiator_state_q)
      IDLE: begin
        if (clear_i) begin
          initiator_state_d = ISOLATE;
        end
      end

      ISOLATE: begin
        initiator_phase_transition_req = 1'b1;
        initiator_clear_seq_phase      = cdc_clear_sync_pkg::ISOLATE;
        initiator_isolate_out          = 1'b1;
        initiator_clear_out            = 1'b0;
        if (initiator_phase_transition_ack && isolate_ack_i) begin
          initiator_state_d = CLEAR;
        end else if (initiator_phase_transition_ack) begin
          initiator_state_d = WAIT_ISOLATE_ACK;
        end else if (isolate_ack_i) begin
          initiator_state_d = WAIT_PHASE_ACK;
        end
      end

      WAIT_ISOLATE_ACK: begin
        initiator_isolate_out     = 1'b1;
        initiator_clear_out       = 1'b0;
        initiator_clear_seq_phase = cdc_clear_sync_pkg::ISOLATE;
        if (isolate_ack_i) begin
          initiator_state_d = CLEAR;
        end
      end

      WAIT_PHASE_ACK: begin
        initiator_phase_transition_req = 1'b1;
        initiator_clear_seq_phase      = cdc_clear_sync_pkg::ISOLATE;
        initiator_isolate_out          = 1'b1;
        initiator_clear_out            = 1'b0;
        if (initiator_phase_transition_ack) begin
          initiator_state_d = CLEAR;
        end
      end

      CLEAR: begin
        initiator_isolate_out          = 1'b1;
        initiator_clear_out            = 1'b1;
        initiator_phase_transition_req = 1'b1;
        initiator_clear_seq_phase      = cdc_clear_sync_pkg::CLEAR;
        if (initiator_phase_transition_ack) begin
          initiator_state_d = POST_CLEAR;
        end
      end

      POST_CLEAR: begin
        initiator_isolate_out          = 1'b1;
        initiator_clear_out            = 1'b0;
        initiator_phase_transition_req = 1'b1;
        initiator_clear_seq_phase      = cdc_clear_sync_pkg::POST_CLEAR;
        if (initiator_phase_transition_ack) begin
          initiator_state_d = FINISHED;
        end
      end

      FINISHED: begin
        initiator_isolate_out          = 1'b1;
        initiator_clear_out            = 1'b0;
        initiator_phase_transition_req = 1'b1;
        initiator_clear_seq_phase      = cdc_clear_sync_pkg::IDLE;
        if (initiator_phase_transition_ack) begin
          initiator_state_d = IDLE;
        end
      end

      default: begin
        initiator_state_d = ISOLATE;
      end
    endcase
  end

  always_ff @(posedge clk_i, negedge rst_ni) begin
    if (!rst_ni) begin
      if (CLEAR_ON_ASYNC_RESET) begin
        initiator_state_q <= ISOLATE; // Start in the ISOLATE state which is
                                        // the first state of a clear sequence.
      end else begin
        initiator_state_q <= IDLE;
      end
    end else begin
      initiator_state_q <= initiator_state_d;
    end
  end

  // Initiator CDC SRC
  // We use 4 phase handshaking. That way it doesn't matter if one side is
  // sudenly reset asynchronously. With a 2phase CDC, one-sided async resets might
  // introduce spurios transactions.

  cdc_4phase_src #(
    .T(clear_seq_phase_e),
    .SYNC_STAGES(2),
    .DECOUPLED(0), // Important! The CDC must not be in decoupled mode.
                   // Otherwise we will proceed to the next state without
                   // waiting for the new state to arrive on the other side.
    .SEND_RESET_MSG(1), // Send the ISOLATE phase request immediately on async
                        // reset
    .RESET_MSG(cdc_clear_sync_pkg::ISOLATE)
  ) i_state_transition_cdc_src(
    .clk_i,
    .rst_ni,
    .data_i(initiator_clear_seq_phase),
    .valid_i(initiator_phase_transition_req),
    .ready_o(initiator_phase_transition_ack),
    .async_req_o,
    .async_ack_i,
    .async_data_o(async_next_phase_o)
  );


  //---------------------- Receiver Side ----------------------
  // This part of the circuit receives clear sequence state transitions from the
  // other side.

  clear_seq_phase_e receiver_phase_q;
  clear_seq_phase_e receiver_next_phase;
  logic receiver_phase_req, receiver_phase_ack;

  logic receiver_isolate_out;
  logic receiver_clear_out;

  cdc_4phase_dst #(
    .T(clear_seq_phase_e),
    .SYNC_STAGES(2),
    .DECOUPLED(0) // Important! The CDC must not be in decoupled mode. Otherwise
                  // we will proceed to the next state without waiting for the
                  // new state to arrive on the other side.
  ) i_state_transition_cdc_dst(
    .clk_i,
    .rst_ni,
    .data_o(receiver_next_phase),
    .valid_o(receiver_phase_req),
    .ready_i(receiver_phase_ack),
    .async_req_i,
    .async_ack_o,
    .async_data_i(async_next_phase_i)
  );

  always_ff @(posedge clk_i, negedge rst_ni) begin
    if (!rst_ni) begin
      receiver_phase_q <= cdc_clear_sync_pkg::IDLE;
    end else if (receiver_phase_req && receiver_phase_ack) begin
      receiver_phase_q <= receiver_next_phase;
    end
  end

  always_comb begin
    receiver_isolate_out = 1'b0;
    receiver_clear_out   = 1'b0;
    receiver_phase_ack   = 1'b0;

    // If there is a new phase requestd, checkout which one it is and act accordingly
    if (receiver_phase_req) begin
      case (receiver_next_phase)
        cdc_clear_sync_pkg::IDLE: begin
          receiver_clear_out   = 1'b0;
          receiver_isolate_out = 1'b0;
          receiver_phase_ack   = 1'b1;
        end

        cdc_clear_sync_pkg::ISOLATE: begin
          receiver_clear_out   = 1'b0;
          receiver_isolate_out = 1'b1;
          // Wait for the isolate to be acknowledged before ack'ing the phase
          receiver_phase_ack = isolate_ack_i;
        end

        cdc_clear_sync_pkg::CLEAR: begin
          receiver_clear_out   = 1'b1;
          receiver_isolate_out = 1'b1;
          receiver_phase_ack   = 1'b1;
        end

        cdc_clear_sync_pkg::POST_CLEAR: begin
          receiver_clear_out   = 1'b0;
          receiver_isolate_out = 1'b1;
          receiver_phase_ack   = 1'b1;
        end

        default: begin
          receiver_clear_out   = 1'b0;
          receiver_isolate_out = 1'b0;
          receiver_phase_ack   = 1'b0;
        end
      endcase

    end else begin
      // No phase change is requested for the moment. Act according to the
      // current phase signal
      case (receiver_phase_q)
        cdc_clear_sync_pkg::IDLE: begin
          receiver_clear_out   = 1'b0;
          receiver_isolate_out = 1'b0;
        end

        cdc_clear_sync_pkg::ISOLATE: begin
          receiver_clear_out   = 1'b0;
          receiver_isolate_out = 1'b1;
        end

        cdc_clear_sync_pkg::CLEAR: begin
          receiver_clear_out   = 1'b1;
          receiver_isolate_out = 1'b1;
        end

        cdc_clear_sync_pkg::POST_CLEAR: begin
          receiver_clear_out   = 1'b0;
          receiver_isolate_out = 1'b1;
        end

        default: begin
          receiver_clear_out   = 1'b0;
          receiver_isolate_out = 1'b0;
          receiver_phase_ack   = 1'b0;
        end
      endcase
    end
  end

  // Output Assignment

  // The clear and isolate signal are the OR combination of the receiver and
  // initiator's clear/isolate signal. This ensures that the correct sequence is
  // followed even if both sides are cleared independently at roughly the same
  // time.
  assign clear_o = initiator_clear_out || receiver_clear_out;
  assign isolate_o = initiator_isolate_out || receiver_isolate_out;

endmodule : cdc_clear_sync_half
