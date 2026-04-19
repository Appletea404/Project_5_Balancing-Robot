`timescale 1ns / 1ps
//================================================================================
// i2c_master
// - MPU6050용 I2C 마스터
// - 1-byte register write 지원
// - burst register read 지원 (최대 16바이트)
//
// 지원 동작
// 1) Write transaction
//    START
//    -> DEV_ADDR_W
//    -> ACK
//    -> REG_ADDR
//    -> ACK
//    -> TX_DATA
//    -> ACK
//    -> STOP
//
// 2) Burst Read transaction
//    START
//    -> DEV_ADDR_W
//    -> ACK
//    -> REG_ADDR
//    -> ACK
//    -> REPEATED START
//    -> DEV_ADDR_R
//    -> ACK
//    -> READ BYTE0 -> ACK
//    -> READ BYTE1 -> ACK
//    -> ...
//    -> READ LAST  -> NACK
//    -> STOP
//
// open-drain 제어 규칙
// - sda_enable = 1 : SDA Low로 당김
// - sda_enable = 0 : SDA Release (외부 풀업으로 High)
// - scl_enable = 1 : SCL Low로 당김
// - scl_enable = 0 : SCL Release (외부 풀업으로 High)
//
// 핵심 수정
// - start_req는 상위 FSM에서 짧은 pulse로 들어와도
//   내부 req_pending에 보관했다가 tick 시점에 소비한다.
//================================================================================

module i2c_master
(
    input  wire         clk,
    input  wire         rst_n,
    input  wire         tick,

    input  wire         start_req,   // transaction 시작 요청
    input  wire         rw,          // 0: write, 1: read
    input  wire [7:0]   reg_addr,    // 시작 레지스터 주소
    input  wire [7:0]   tx_data,     // write 시 전송할 1-byte 데이터
    input  wire [7:0]   burst_len,   // read 시 읽을 바이트 수 (1~16)
    input  wire         sda_in,      // 실제 SDA 입력 샘플

    output reg          busy,        // transaction 진행 중
    output reg          done,        // 완료 pulse (1 tick)
    output reg          ack_ok,      // transaction ACK 성공 여부
    output reg  [2:0]   err_code,    // 에러 코드
    output reg  [7:0]   rx_data,     // 마지막으로 읽은 1-byte
    output reg  [127:0] rx_buf,      // 최대 16-byte 수신 버퍼
    output reg  [7:0]   rx_count,    // 실제 읽은 바이트 수

    output reg          sda_enable,  // 1: SDA Low, 0: SDA Release
    output reg          scl_enable   // 1: SCL Low, 0: SCL Release
);

    //============================================================================
    // MPU6050 I2C 주소
    // AD0=0 기준 7-bit 주소 = 0x68
    //============================================================================
    localparam [6:0] MPU6050_ADDR_7BIT = 7'h68;
    localparam [7:0] MPU6050_ADDR_W    = {MPU6050_ADDR_7BIT, 1'b0};
    localparam [7:0] MPU6050_ADDR_R    = {MPU6050_ADDR_7BIT, 1'b1};

    //============================================================================
    // 큰 transaction 단계
    //============================================================================
    localparam [1:0] STEP_ADDR_W = 2'd0;
    localparam [1:0] STEP_REG    = 2'd1;
    localparam [1:0] STEP_DATA   = 2'd2;
    localparam [1:0] STEP_ADDR_R = 2'd3;

    //============================================================================
    // 에러 코드
    //============================================================================
    localparam [2:0] ERR_NONE    = 3'd0;
    localparam [2:0] ERR_ADDR_W  = 3'd1;
    localparam [2:0] ERR_REG     = 3'd2;
    localparam [2:0] ERR_DATA    = 3'd3;
    localparam [2:0] ERR_ADDR_R  = 3'd4;
    localparam [2:0] ERR_PARAM   = 3'd5;

    //============================================================================
    // FSM 상태 정의
    //============================================================================
    localparam [4:0] ST_IDLE         = 5'd0;

    // START
    localparam [4:0] ST_START_A      = 5'd1;
    localparam [4:0] ST_START_B      = 5'd2;

    // WRITE 1bit
    localparam [4:0] ST_WRITE_LOW    = 5'd3;
    localparam [4:0] ST_WRITE_HIGH   = 5'd4;

    // ACK 수신
    localparam [4:0] ST_ACK_LOW      = 5'd5;
    localparam [4:0] ST_ACK_RISE     = 5'd6;
    localparam [4:0] ST_ACK_HOLD     = 5'd7;
    localparam [4:0] ST_ACK_SAMPLE   = 5'd8;

    // REPEATED START
    localparam [4:0] ST_RSTART_0     = 5'd9;
    localparam [4:0] ST_RSTART_1     = 5'd10;
    localparam [4:0] ST_RSTART_2     = 5'd11;
    localparam [4:0] ST_RSTART_3     = 5'd12;

    // READ 1bit
    localparam [4:0] ST_READ_LOW     = 5'd13;
    localparam [4:0] ST_READ_RISE    = 5'd14;
    localparam [4:0] ST_READ_HOLD    = 5'd15;
    localparam [4:0] ST_READ_SAMPLE  = 5'd16;
    localparam [4:0] ST_READ_STORE   = 5'd17;

    // master ACK after intermediate read byte
    localparam [4:0] ST_MACK_LOW     = 5'd18;
    localparam [4:0] ST_MACK_HIGH    = 5'd19;

    // NACK after last read byte
    localparam [4:0] ST_NACK_LOW     = 5'd20;
    localparam [4:0] ST_NACK_HIGH    = 5'd21;

    // STOP
    localparam [4:0] ST_STOP_0       = 5'd22;
    localparam [4:0] ST_STOP_1       = 5'd23;
    localparam [4:0] ST_STOP_2       = 5'd24;

    // DONE
    localparam [4:0] ST_DONE         = 5'd25;

    //============================================================================
    // 내부 레지스터
    //============================================================================
    reg [4:0] state;
    reg [1:0] step;
    reg [7:0] tx_buf;
    reg [2:0] bit_cnt;
    reg       rw_latched;

    reg [7:0] burst_len_latched;
    reg [7:0] byte_cnt;

    // 상위 start_req를 놓치지 않도록 내부에 보관
    reg       req_pending;

    //============================================================================
    // start_req 래치
    //============================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            req_pending <= 1'b0;
        end
        else begin
            // 상위 pulse가 오면 기억
            if (start_req) begin
                req_pending <= 1'b1;
            end
            // IDLE 상태에서 tick 시점에 실제로 소비하면 클리어
            else if (tick && (state == ST_IDLE) && req_pending) begin
                req_pending <= 1'b0;
            end
        end
    end

    //============================================================================
    // FSM
    //============================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state             <= ST_IDLE;
            step              <= STEP_ADDR_W;
            tx_buf            <= 8'd0;
            bit_cnt           <= 3'd7;
            rw_latched        <= 1'b0;
            burst_len_latched <= 8'd0;
            byte_cnt          <= 8'd0;

            busy              <= 1'b0;
            done              <= 1'b0;
            ack_ok            <= 1'b0;
            err_code          <= ERR_NONE;

            rx_data           <= 8'd0;
            rx_buf            <= 128'd0;
            rx_count          <= 8'd0;

            sda_enable        <= 1'b0;
            scl_enable        <= 1'b0;
        end
        else begin
            // done은 기본적으로 0, 완료 시점에만 1 tick pulse
            done <= 1'b0;

            if (tick) begin
                case (state)

                    //==========================================================
                    // IDLE
                    //==========================================================
                    ST_IDLE: begin
                        sda_enable <= 1'b0;  // SDA release -> High
                        scl_enable <= 1'b0;  // SCL release -> High
                        busy       <= 1'b0;

                        if (req_pending) begin
                            // read인데 burst_len=0이면 잘못된 요청
                            if (rw && (burst_len == 8'd0)) begin
                                ack_ok   <= 1'b0;
                                err_code <= ERR_PARAM;
                                busy     <= 1'b0;
                                state    <= ST_DONE;
                            end
                            else begin
                                busy              <= 1'b1;
                                ack_ok            <= 1'b1;
                                err_code          <= ERR_NONE;

                                rw_latched        <= rw;
                                burst_len_latched <= (rw && (burst_len > 8'd16)) ? 8'd16 : burst_len;
                                byte_cnt          <= 8'd0;

                                rx_data           <= 8'd0;
                                rx_buf            <= 128'd0;
                                rx_count          <= 8'd0;

                                step              <= STEP_ADDR_W;
                                tx_buf            <= MPU6050_ADDR_W;
                                bit_cnt           <= 3'd7;
                                state             <= ST_START_A;
                            end
                        end
                    end

                    //==========================================================
                    // START 조건 생성
                    //==========================================================
                    ST_START_A: begin
                        sda_enable <= 1'b0;  // SDA High
                        scl_enable <= 1'b0;  // SCL High
                        state      <= ST_START_B;
                    end

                    ST_START_B: begin
                        sda_enable <= 1'b1;  // SDA Low
                        scl_enable <= 1'b0;  // SCL High 유지
                        state      <= ST_WRITE_LOW;
                    end

                    //==========================================================
                    // WRITE 1bit
                    //==========================================================
                    ST_WRITE_LOW: begin
                        scl_enable <= 1'b1;  // SCL Low

                        // open-drain:
                        // 0 -> Low drive
                        // 1 -> release
                        if (tx_buf[bit_cnt] == 1'b0)
                            sda_enable <= 1'b1;
                        else
                            sda_enable <= 1'b0;

                        state <= ST_WRITE_HIGH;
                    end

                    ST_WRITE_HIGH: begin
                        scl_enable <= 1'b0;  // SCL High

                        if (bit_cnt == 3'd0)
                            state <= ST_ACK_LOW;
                        else begin
                            bit_cnt <= bit_cnt - 3'd1;
                            state   <= ST_WRITE_LOW;
                        end
                    end

                    //==========================================================
                    // slave ACK 수신
                    //==========================================================
                    ST_ACK_LOW: begin
                        sda_enable <= 1'b0;  // slave가 구동하도록 release
                        scl_enable <= 1'b1;  // SCL Low
                        state      <= ST_ACK_RISE;
                    end

                    ST_ACK_RISE: begin
                        sda_enable <= 1'b0;
                        scl_enable <= 1'b0;  // SCL High
                        state      <= ST_ACK_HOLD;
                    end

                    ST_ACK_HOLD: begin
                        sda_enable <= 1'b0;
                        scl_enable <= 1'b0;  // SCL High 유지
                        state      <= ST_ACK_SAMPLE;
                    end

                    ST_ACK_SAMPLE: begin
                        sda_enable <= 1'b0;
                        scl_enable <= 1'b0;

                        // ACK는 SDA=0
                        if (sda_in != 1'b0) begin
                            ack_ok <= 1'b0;

                            case (step)
                                STEP_ADDR_W: err_code <= ERR_ADDR_W;
                                STEP_REG   : err_code <= ERR_REG;
                                STEP_DATA  : err_code <= ERR_DATA;
                                STEP_ADDR_R: err_code <= ERR_ADDR_R;
                                default    : err_code <= ERR_NONE;
                            endcase

                            state <= ST_STOP_0;
                        end
                        else begin
                            case (step)
                                STEP_ADDR_W: begin
                                    step    <= STEP_REG;
                                    tx_buf  <= reg_addr;
                                    bit_cnt <= 3'd7;
                                    state   <= ST_WRITE_LOW;
                                end

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

                                STEP_DATA: begin
                                    state <= ST_STOP_0;
                                end

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

                    //==========================================================
                    // REPEATED START
                    //==========================================================
                    ST_RSTART_0: begin
                        sda_enable <= 1'b0;
                        scl_enable <= 1'b1;  // SCL Low
                        state      <= ST_RSTART_1;
                    end

                    ST_RSTART_1: begin
                        sda_enable <= 1'b0;  // SDA release
                        scl_enable <= 1'b1;  // SCL Low 유지
                        state      <= ST_RSTART_2;
                    end

                    ST_RSTART_2: begin
                        sda_enable <= 1'b0;  // SDA High
                        scl_enable <= 1'b0;  // SCL High
                        state      <= ST_RSTART_3;
                    end

                    ST_RSTART_3: begin
                        sda_enable <= 1'b1;  // SDA High -> Low
                        scl_enable <= 1'b0;  // SCL High 유지
                        state      <= ST_WRITE_LOW;
                    end

                    //==========================================================
                    // READ 1bit
                    //==========================================================
                    ST_READ_LOW: begin
                        sda_enable <= 1'b0;  // release
                        scl_enable <= 1'b1;  // SCL Low
                        state      <= ST_READ_RISE;
                    end

                    ST_READ_RISE: begin
                        sda_enable <= 1'b0;
                        scl_enable <= 1'b0;  // SCL High
                        state      <= ST_READ_HOLD;
                    end

                    ST_READ_HOLD: begin
                        sda_enable <= 1'b0;
                        scl_enable <= 1'b0;  // SCL High 유지
                        state      <= ST_READ_SAMPLE;
                    end

                    ST_READ_SAMPLE: begin
                        sda_enable       <= 1'b0;
                        scl_enable       <= 1'b0;
                        rx_data[bit_cnt] <= sda_in;

                        if (bit_cnt == 3'd0)
                            state <= ST_READ_STORE;
                        else begin
                            bit_cnt <= bit_cnt - 3'd1;
                            state   <= ST_READ_LOW;
                        end
                    end

                    //==========================================================
                    // READ_STORE
                    //==========================================================
                    ST_READ_STORE: begin
                        sda_enable <= 1'b0;
                        scl_enable <= 1'b0;

                        rx_buf[(byte_cnt*8) +: 8] <= rx_data;
                        rx_count                  <= byte_cnt + 8'd1;

                        if (byte_cnt == burst_len_latched - 1'b1)
                            state <= ST_NACK_LOW;
                        else
                            state <= ST_MACK_LOW;
                    end

                    //==========================================================
                    // burst read 중간 바이트 뒤 master ACK 전송
                    //==========================================================
                    ST_MACK_LOW: begin
                        sda_enable <= 1'b1;  // ACK = SDA Low
                        scl_enable <= 1'b1;  // SCL Low
                        state      <= ST_MACK_HIGH;
                    end

                    ST_MACK_HIGH: begin
                        sda_enable <= 1'b1;  // ACK 유지
                        scl_enable <= 1'b0;  // SCL High

                        byte_cnt <= byte_cnt + 8'd1;
                        bit_cnt  <= 3'd7;
                        rx_data  <= 8'd0;
                        state    <= ST_READ_LOW;
                    end

                    //==========================================================
                    // 마지막 바이트 뒤 NACK 전송
                    //==========================================================
                    ST_NACK_LOW: begin
                        sda_enable <= 1'b0;  // NACK = release
                        scl_enable <= 1'b1;  // SCL Low
                        state      <= ST_NACK_HIGH;
                    end

                    ST_NACK_HIGH: begin
                        sda_enable <= 1'b0;  // release 유지
                        scl_enable <= 1'b0;  // SCL High
                        state      <= ST_STOP_0;
                    end

                    //==========================================================
                    // STOP 조건 생성
                    //==========================================================
                    ST_STOP_0: begin
                        sda_enable <= 1'b1;  // SDA Low
                        scl_enable <= 1'b1;  // SCL Low
                        state      <= ST_STOP_1;
                    end

                    ST_STOP_1: begin
                        sda_enable <= 1'b1;  // SDA Low 유지
                        scl_enable <= 1'b0;  // SCL High
                        state      <= ST_STOP_2;
                    end

                    ST_STOP_2: begin
                        sda_enable <= 1'b0;  // SDA release -> High
                        scl_enable <= 1'b0;  // SCL High 유지
                        state      <= ST_DONE;
                    end

                    //==========================================================
                    // DONE
                    //==========================================================
                    ST_DONE: begin
                        busy  <= 1'b0;
                        done  <= 1'b1;
                        state <= ST_IDLE;
                    end

                    default: begin
                        state             <= ST_IDLE;
                        step              <= STEP_ADDR_W;
                        tx_buf            <= 8'd0;
                        bit_cnt           <= 3'd7;
                        rw_latched        <= 1'b0;
                        burst_len_latched <= 8'd0;
                        byte_cnt          <= 8'd0;

                        busy              <= 1'b0;
                        ack_ok            <= 1'b0;
                        err_code          <= ERR_NONE;

                        rx_data           <= 8'd0;
                        rx_buf            <= 128'd0;
                        rx_count          <= 8'd0;

                        sda_enable        <= 1'b0;
                        scl_enable        <= 1'b0;
                    end
                endcase
            end
        end
    end

endmodule