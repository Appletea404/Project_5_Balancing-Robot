`timescale 1ns / 1ps
//================================================================================
// i2c_master
// - MPU6050 WHO_AM_I(0x75) 1-byte read 전용 I2C 엔진
// - Repeated START 방식 사용
//
//   START
//   -> MPU6050_ADDR_W
//   -> ACK
//   -> reg_addr
//   -> ACK
//   -> REPEATED START
//   -> MPU6050_ADDR_R
//   -> ACK
//   -> 1-byte READ
//   -> NACK
//   -> STOP
//
// open-drain 제어 규칙
//   sda_enable = 1 -> SDA Low로 당김
//   sda_enable = 0 -> SDA release(Z)
//
//   scl_enable = 1 -> SCL Low로 당김
//   scl_enable = 0 -> SCL release(Z)
//================================================================================

module i2c_master
(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       tick,

    input  wire       start_req,   // 전체 transaction 시작 요청
    input  wire [7:0] reg_addr,    // 읽을 레지스터 주소
    input  wire       sda_in,      // SDA 입력 샘플

    output reg        busy,        // transaction 진행 중
    output reg        done,        // transaction 완료 pulse
    output reg        ack_ok,      // 전체 ACK 성공 여부
    output reg [7:0]  rx_data,     // 읽은 1-byte 데이터

    output reg        sda_enable,  // 1: SDA Low, 0: SDA Release
    output reg        scl_enable   // 1: SCL Low, 0: SCL Release
);

//================================================================================
// MPU6050 Address Definition
//================================================================================
localparam [6:0] MPU6050_ADDR_7BIT = 7'h68;
localparam [7:0] MPU6050_ADDR_W    = {MPU6050_ADDR_7BIT, 1'b0}; // 0xD0
localparam [7:0] MPU6050_ADDR_R    = {MPU6050_ADDR_7BIT, 1'b1}; // 0xD1

//================================================================================
// Sequence Step
//================================================================================
localparam [1:0] STEP_ADDR_W = 2'd0;
localparam [1:0] STEP_REG    = 2'd1;
localparam [1:0] STEP_ADDR_R = 2'd2;
localparam [1:0] STEP_READ   = 2'd3;

//================================================================================
// State Definition
//================================================================================
localparam [4:0] ST_IDLE         = 5'd0;

// START
localparam [4:0] ST_START_A      = 5'd1;
localparam [4:0] ST_START_B      = 5'd2;

// WRITE BYTE
localparam [4:0] ST_WRITE_LOW    = 5'd3;
localparam [4:0] ST_WRITE_HIGH   = 5'd4;

// ACK
localparam [4:0] ST_ACK_LOW      = 5'd5;
localparam [4:0] ST_ACK_HIGH     = 5'd6;

// REPEATED START
localparam [4:0] ST_RSTART_0     = 5'd7;
localparam [4:0] ST_RSTART_1     = 5'd8;
localparam [4:0] ST_RSTART_2     = 5'd9;
localparam [4:0] ST_RSTART_3     = 5'd10;

// READ BYTE
localparam [4:0] ST_READ_LOW     = 5'd11;
localparam [4:0] ST_READ_RISE    = 5'd12;
localparam [4:0] ST_READ_HOLD    = 5'd13;
localparam [4:0] ST_READ_SAMPLE  = 5'd14;

// NACK
localparam [4:0] ST_NACK_LOW     = 5'd15;
localparam [4:0] ST_NACK_HIGH    = 5'd16;

// STOP
localparam [4:0] ST_STOP_0       = 5'd17;
localparam [4:0] ST_STOP_1       = 5'd18;
localparam [4:0] ST_STOP_2       = 5'd19;

// DONE
localparam [4:0] ST_DONE         = 5'd20;

//================================================================================
// Internal Register
//================================================================================
reg [4:0] state;
reg [1:0] step;
reg [7:0] tx_buf;
reg [2:0] bit_cnt;

//================================================================================
// FSM
//================================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state      <= ST_IDLE;
        step       <= STEP_ADDR_W;
        tx_buf     <= 8'd0;
        bit_cnt    <= 3'd7;

        busy       <= 1'b0;
        done       <= 1'b0;
        ack_ok     <= 1'b0;
        rx_data    <= 8'd0;

        sda_enable <= 1'b0; // release
        scl_enable <= 1'b0; // release
    end
    else begin
        if (tick) begin
            case (state)

                //==============================================================
                // IDLE
                //==============================================================
                ST_IDLE: begin
                    sda_enable <= 1'b0;   // SDA High(release)
                    scl_enable <= 1'b0;   // SCL High(release)
                    busy       <= 1'b0;
                    done       <= 1'b0;

                    if (start_req) begin
                        busy       <= 1'b1;
                        done       <= 1'b0;
                        ack_ok     <= 1'b1;          // 시작 시 일단 성공 가정
                        step       <= STEP_ADDR_W;
                        tx_buf     <= MPU6050_ADDR_W;
                        bit_cnt    <= 3'd7;
                        state      <= ST_START_A;
                    end
                end

                //==============================================================
                // START
                // START = SCL High 상태에서 SDA High -> Low
                //==============================================================
                ST_START_A: begin
                    sda_enable <= 1'b0;   // SDA High
                    scl_enable <= 1'b0;   // SCL High
                    state      <= ST_START_B;
                end

                ST_START_B: begin
                    sda_enable <= 1'b1;   // SDA Low
                    scl_enable <= 1'b0;   // SCL High 유지
                    state      <= ST_WRITE_LOW;
                end

                //==============================================================
                // WRITE BYTE
                //==============================================================
                ST_WRITE_LOW: begin
                    scl_enable <= 1'b1;   // SCL Low

                    if (tx_buf[bit_cnt] == 1'b0)
                        sda_enable <= 1'b1;   // '0' 전송
                    else
                        sda_enable <= 1'b0;   // '1' 전송 = release

                    state <= ST_WRITE_HIGH;
                end

                ST_WRITE_HIGH: begin
                    scl_enable <= 1'b0;   // SCL High

                    if (bit_cnt == 3'd0)
                        state <= ST_ACK_LOW;
                    else begin
                        bit_cnt <= bit_cnt - 3'd1;
                        state   <= ST_WRITE_LOW;
                    end
                end

                //==============================================================
                // ACK
                //==============================================================
                ST_ACK_LOW: begin
                    sda_enable <= 1'b0;   // slave가 ACK 구동하도록 release
                    scl_enable <= 1'b1;   // SCL Low
                    state      <= ST_ACK_HIGH;
                end

                ST_ACK_HIGH: begin
                    scl_enable <= 1'b0;   // SCL High

                    if (sda_in != 1'b0) begin
                        // ACK 실패
                        ack_ok <= 1'b0;
                        state  <= ST_STOP_0;
                    end
                    else begin
                        // ACK 성공
                        case (step)
                            STEP_ADDR_W: begin
                                step    <= STEP_REG;
                                tx_buf  <= reg_addr;
                                bit_cnt <= 3'd7;
                                state   <= ST_WRITE_LOW;
                            end

                            STEP_REG: begin
                                // STOP 없이 Repeated START 수행
                                step    <= STEP_ADDR_R;
                                tx_buf  <= MPU6050_ADDR_R;
                                bit_cnt <= 3'd7;
                                state   <= ST_RSTART_0;
                            end

                            STEP_ADDR_R: begin
                                step    <= STEP_READ;
                                bit_cnt <= 3'd7;
                                rx_data <= 8'd0;
                                state   <= ST_READ_LOW;
                            end

                            default: begin
                                state <= ST_STOP_0;
                            end
                        endcase
                    end
                end

                //==============================================================
                // REPEATED START
                //==============================================================
                ST_RSTART_0: begin
                    sda_enable <= 1'b0;   // SDA release
                    scl_enable <= 1'b1;   // SCL Low
                    state      <= ST_RSTART_1;
                end

                ST_RSTART_1: begin
                    sda_enable <= 1'b0;   // SDA High 유지
                    scl_enable <= 1'b1;   // SCL Low 유지
                    state      <= ST_RSTART_2;
                end

                ST_RSTART_2: begin
                    sda_enable <= 1'b0;   // SDA High
                    scl_enable <= 1'b0;   // SCL High
                    state      <= ST_RSTART_3;
                end

                ST_RSTART_3: begin
                    sda_enable <= 1'b1;   // SDA High -> Low
                    scl_enable <= 1'b0;   // SCL High 유지
                    state      <= ST_WRITE_LOW;
                end

                //==============================================================
                // READ BYTE
                //
                // 안정화 목적:
                // 1) SCL Low에서 slave가 다음 비트를 준비할 시간 제공
                // 2) SCL을 High로 올리는 상태 분리
                // 3) High 상태를 한 tick 유지
                // 4) 그 다음 tick에서 샘플링
                //==============================================================
                ST_READ_LOW: begin
                    sda_enable <= 1'b0;   // master는 SDA release
                    scl_enable <= 1'b1;   // SCL Low
                    state      <= ST_READ_RISE;
                end

                ST_READ_RISE: begin
                    sda_enable <= 1'b0;   // SDA release 유지
                    scl_enable <= 1'b0;   // SCL High
                    state      <= ST_READ_HOLD;
                end

                ST_READ_HOLD: begin
                    sda_enable <= 1'b0;   // SDA release 유지
                    scl_enable <= 1'b0;   // SCL High 유지
                    state      <= ST_READ_SAMPLE;
                end

                ST_READ_SAMPLE: begin
                    sda_enable       <= 1'b0;   // SDA release 유지
                    scl_enable       <= 1'b0;   // SCL High 유지
                    rx_data[bit_cnt] <= sda_in; // 안정화 후 샘플링

                    if (bit_cnt == 3'd0)
                        state <= ST_NACK_LOW;
                    else begin
                        bit_cnt <= bit_cnt - 3'd1;
                        state   <= ST_READ_LOW;
                    end
                end

                //==============================================================
                // NACK
                //==============================================================
                ST_NACK_LOW: begin
                    sda_enable <= 1'b0;   // release
                    scl_enable <= 1'b1;   // SCL Low
                    state      <= ST_NACK_HIGH;
                end

                ST_NACK_HIGH: begin
                    sda_enable <= 1'b0;   // release 유지 -> NACK
                    scl_enable <= 1'b0;   // SCL High
                    state      <= ST_STOP_0;
                end

                //==============================================================
                // STOP
                // STOP = SCL High 상태에서 SDA Low -> High
                //==============================================================
                ST_STOP_0: begin
                    sda_enable <= 1'b1;   // SDA Low
                    scl_enable <= 1'b1;   // SCL Low
                    state      <= ST_STOP_1;
                end

                ST_STOP_1: begin
                    sda_enable <= 1'b1;   // SDA Low 유지
                    scl_enable <= 1'b0;   // SCL High
                    state      <= ST_STOP_2;
                end

                ST_STOP_2: begin
                    sda_enable <= 1'b0;   // SDA release -> High
                    scl_enable <= 1'b0;   // SCL High 유지
                    state      <= ST_DONE;
                end

                //==============================================================
                // DONE
                //==============================================================
                ST_DONE: begin
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    state <= ST_IDLE;
                end

                //==============================================================
                // default
                //==============================================================
                default: begin
                    state      <= ST_IDLE;
                    step       <= STEP_ADDR_W;
                    tx_buf     <= 8'd0;
                    bit_cnt    <= 3'd7;

                    busy       <= 1'b0;
                    done       <= 1'b0;
                    ack_ok     <= 1'b0;
                    rx_data    <= 8'd0;

                    sda_enable <= 1'b0;
                    scl_enable <= 1'b0;
                end
            endcase
        end
    end
end

endmodule