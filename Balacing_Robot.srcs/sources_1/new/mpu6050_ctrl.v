`timescale 1ns / 1ps
//================================================================================
// mpu6050_ctrl
// - MPU6050 / MPU6500 계열 WHO_AM_I 레지스터 읽기 요청을 담당하는 상위 FSM
// - 현재는 WHO_AM_I(0x75) 1개만 읽어서 valid / who_am_i를 갱신
//
// valid = 1 조건
//   -> ack_ok == 1 이고
//   -> rx_data == 8'h68 또는 8'h70 인 경우
//
// 디버깅 편의를 위해 실패해도 who_am_i에는 실제 rx_data를 남긴다
//================================================================================

module mpu6050_ctrl
(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       tick,

    // i2c_master 상태/결과
    input  wire       busy,
    input  wire       done,
    input  wire       ack_ok,
    input  wire [7:0] rx_data,

    // i2c_master 제어
    output reg        start_req,
    output reg [7:0]  reg_addr,

    // 결과
    output reg [7:0]  who_am_i,
    output reg        valid
);

//================================================================================
// Register Definition
//================================================================================
localparam [7:0] REG_WHO_AM_I = 8'h75;

// 허용할 WHO_AM_I 값
localparam [7:0] WHO_AM_I_MPU6050 = 8'h68;
localparam [7:0] WHO_AM_I_MPU6500 = 8'h70;

//================================================================================
// State Definition
//================================================================================
localparam [2:0] CTRL_IDLE      = 3'd0;
localparam [2:0] CTRL_REQ       = 3'd1;
localparam [2:0] CTRL_WAIT_BUSY = 3'd2;
localparam [2:0] CTRL_WAIT_DONE = 3'd3;
localparam [2:0] CTRL_LATCH     = 3'd4;
localparam [2:0] CTRL_DONE      = 3'd5;
localparam [2:0] CTRL_FAIL      = 3'd6;

reg [2:0] state;

//================================================================================
// FSM
//================================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state     <= CTRL_IDLE;
        start_req <= 1'b0;
        reg_addr  <= REG_WHO_AM_I;
        who_am_i  <= 8'd0;
        valid     <= 1'b0;
    end
    else begin
        if (tick) begin
            case (state)
                //--------------------------------------------------------------
                // 초기 진입
                //--------------------------------------------------------------
                CTRL_IDLE: begin
                    start_req <= 1'b0;
                    reg_addr  <= REG_WHO_AM_I;
                    valid     <= 1'b0;
                    state     <= CTRL_REQ;
                end

                //--------------------------------------------------------------
                // i2c_master에 읽기 요청
                //--------------------------------------------------------------
                CTRL_REQ: begin
                    start_req <= 1'b1;
                    reg_addr  <= REG_WHO_AM_I;
                    state     <= CTRL_WAIT_BUSY;
                end

                //--------------------------------------------------------------
                // i2c_master가 busy 응답할 때까지 대기
                //--------------------------------------------------------------
                CTRL_WAIT_BUSY: begin
                    if (busy) begin
                        start_req <= 1'b0;
                        state     <= CTRL_WAIT_DONE;
                    end
                end

                //--------------------------------------------------------------
                // 트랜잭션 완료 대기
                //--------------------------------------------------------------
                CTRL_WAIT_DONE: begin
                    if (done) begin
                        state <= CTRL_LATCH;
                    end
                end

                //--------------------------------------------------------------
                // 결과 저장
                // 성공 조건:
                //   ack_ok == 1 && (rx_data == 8'h68 || rx_data == 8'h70)
                //--------------------------------------------------------------
                CTRL_LATCH: begin
                    who_am_i <= rx_data;

                    if (ack_ok &&
                        ((rx_data == WHO_AM_I_MPU6050) ||
                         (rx_data == WHO_AM_I_MPU6500))) begin
                        valid <= 1'b1;
                        state <= CTRL_DONE;
                    end
                    else begin
                        valid <= 1'b0;
                        state <= CTRL_FAIL;
                    end
                end

                //--------------------------------------------------------------
                // 성공 시 여기 머무름
                //--------------------------------------------------------------
                CTRL_DONE: begin
                    start_req <= 1'b0;
                    valid     <= 1'b1;
                    state     <= CTRL_DONE;
                end

                //--------------------------------------------------------------
                // 실패 시 여기 머무름
                //--------------------------------------------------------------
                CTRL_FAIL: begin
                    start_req <= 1'b0;
                    valid     <= 1'b0;
                    state     <= CTRL_FAIL;
                end

                //--------------------------------------------------------------
                default: begin
                    state     <= CTRL_IDLE;
                    start_req <= 1'b0;
                    reg_addr  <= REG_WHO_AM_I;
                    who_am_i  <= 8'd0;
                    valid     <= 1'b0;
                end
            endcase
        end
    end
end

endmodule