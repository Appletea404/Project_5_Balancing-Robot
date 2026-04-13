`timescale 1ns / 1ps
//================================================================================
// i2c_master
// - MPU6050용 1-byte register read / 1-byte register write 지원 I2C 엔진
//
// [Write sequence]
//   START
//   -> DEV_ADDR_W
//   -> ACK
//   -> REG_ADDR
//   -> ACK
//   -> TX_DATA
//   -> ACK
//   -> STOP
//
// [Read sequence]
//   START
//   -> DEV_ADDR_W
//   -> ACK
//   -> REG_ADDR
//   -> ACK
//   -> REPEATED START
//   -> DEV_ADDR_R
//   -> ACK
//   -> READ 1-byte
//   -> NACK
//   -> STOP
//
// open-drain 제어 규칙
//   sda_enable = 1 -> SDA Low
//   sda_enable = 0 -> SDA Release(Z)
//
//   scl_enable = 1 -> SCL Low
//   scl_enable = 0 -> SCL Release(Z)
//================================================================================

module i2c_master
(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       tick,

    input  wire       start_req,   // transaction 시작 요청
    input  wire       rw,          // 0: write, 1: read
    input  wire [7:0] reg_addr,    // 레지스터 주소
    input  wire [7:0] tx_data,     // write 시 전송 데이터
    input  wire       sda_in,      // SDA 입력 샘플

    output reg        busy,        // transaction 진행 중
    output reg        done,        // 완료 pulse
    output reg        ack_ok,      // 전체 ACK 성공 여부
    output reg [7:0]  rx_data,     // read 시 수신 데이터

    output reg        sda_enable,  // 1: SDA Low, 0: SDA Release
    output reg        scl_enable   // 1: SCL Low, 0: SCL Release
);

//================================================================================
// Device Address
//================================================================================
localparam [6:0] MPU6050_ADDR_7BIT = 7'h68;
localparam [7:0] MPU6050_ADDR_W    = {MPU6050_ADDR_7BIT, 1'b0};
localparam [7:0] MPU6050_ADDR_R    = {MPU6050_ADDR_7BIT, 1'b1};

//================================================================================
// Sequence Step
//================================================================================
localparam [1:0] STEP_ADDR_W = 2'd0;
localparam [1:0] STEP_REG    = 2'd1;
localparam [1:0] STEP_DATA   = 2'd2;
localparam [1:0] STEP_ADDR_R = 2'd3;

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
reg       rw_latched;

//================================================================================
// FSM
//================================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state      <= ST_IDLE;
        step       <= STEP_ADDR_W;
        tx_buf     <= 8'd0;
        bit_cnt    <= 3'd7;
        rw_latched <= 1'b0;

        busy       <= 1'b0;
        done       <= 1'b0;
        ack_ok     <= 1'b0;
        rx_data    <= 8'd0;

        sda_enable <= 1'b0;
        scl_enable <= 1'b0;
    end
    else begin
        if (tick) begin
            case (state)

                //==============================================================
                // IDLE
                //==============================================================
                ST_IDLE: begin
                    sda_enable <= 1'b0;   // SDA High
                    scl_enable <= 1'b0;   // SCL High
                    busy       <= 1'b0;
                    done       <= 1'b0;

                    if (start_req) begin
                        busy       <= 1'b1;
                        done       <= 1'b0;
                        ack_ok     <= 1'b1;
                        rw_latched <= rw;
                        step       <= STEP_ADDR_W;
                        tx_buf     <= MPU6050_ADDR_W;
                        bit_cnt    <= 3'd7;
                        state      <= ST_START_A;
                    end
                end

                //==============================================================
                // START
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
                        sda_enable <= 1'b1;   // '0'
                    else
                        sda_enable <= 1'b0;   // '1'

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
                    sda_enable <= 1'b0;   // slave ACK drive
                    scl_enable <= 1'b1;   // SCL Low
                    state      <= ST_ACK_HIGH;
                end

                ST_ACK_HIGH: begin
                    scl_enable <= 1'b0;   // SCL High

                    if (sda_in != 1'b0) begin
                        ack_ok <= 1'b0;
                        state  <= ST_STOP_0;
                    end
                    else begin
                        case (step)
                            //--------------------------------------------------
                            // DEV_ADDR_W ACK 이후 -> REG_ADDR 전송
                            //--------------------------------------------------
                            STEP_ADDR_W: begin
                                step    <= STEP_REG;
                                tx_buf  <= reg_addr;
                                bit_cnt <= 3'd7;
                                state   <= ST_WRITE_LOW;
                            end

                            //--------------------------------------------------
                            // REG_ADDR ACK 이후
                            // write면 DATA 전송
                            // read면 repeated start
                            //--------------------------------------------------
                            STEP_REG: begin
                                if (rw_latched == 1'b0) begin
                                    step    <= STEP_DATA;
                                    tx_buf  <= tx_data;
                                    bit_cnt <= 3'd7;
                                    state   <= ST_WRITE_LOW;
                                end
                                else begin
                                    step    <= STEP_ADDR_R;
                                    tx_buf  <= MPU6050_ADDR_R;
                                    bit_cnt <= 3'd7;
                                    state   <= ST_RSTART_0;
                                end
                            end

                            //--------------------------------------------------
                            // DATA ACK 이후 -> STOP
                            //--------------------------------------------------
                            STEP_DATA: begin
                                state <= ST_STOP_0;
                            end

                            //--------------------------------------------------
                            // DEV_ADDR_R ACK 이후 -> READ 시작
                            //--------------------------------------------------
                            STEP_ADDR_R: begin
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
                    sda_enable <= 1'b0;
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
                //==============================================================
                ST_READ_LOW: begin
                    sda_enable <= 1'b0;   // master release
                    scl_enable <= 1'b1;   // SCL Low
                    state      <= ST_READ_RISE;
                end

                ST_READ_RISE: begin
                    sda_enable <= 1'b0;
                    scl_enable <= 1'b0;   // SCL High
                    state      <= ST_READ_HOLD;
                end

                ST_READ_HOLD: begin
                    sda_enable <= 1'b0;
                    scl_enable <= 1'b0;   // SCL High hold
                    state      <= ST_READ_SAMPLE;
                end

                ST_READ_SAMPLE: begin
                    sda_enable       <= 1'b0;
                    scl_enable       <= 1'b0;
                    rx_data[bit_cnt] <= sda_in;

                    if (bit_cnt == 3'd0)
                        state <= ST_NACK_LOW;
                    else begin
                        bit_cnt <= bit_cnt - 3'd1;
                        state   <= ST_READ_LOW;
                    end
                end

                //==============================================================
                // NACK after final read byte
                //==============================================================
                ST_NACK_LOW: begin
                    sda_enable <= 1'b0;   // release
                    scl_enable <= 1'b1;   // SCL Low
                    state      <= ST_NACK_HIGH;
                end

                ST_NACK_HIGH: begin
                    sda_enable <= 1'b0;   // release => NACK
                    scl_enable <= 1'b0;   // SCL High
                    state      <= ST_STOP_0;
                end

                //==============================================================
                // STOP
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
                    rw_latched <= 1'b0;

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