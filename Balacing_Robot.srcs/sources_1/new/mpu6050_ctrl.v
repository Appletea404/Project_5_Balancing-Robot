`timescale 1ns / 1ps
//==============================================================================
// mpu6050_ctrl
//------------------------------------------------------------------------------
// 목적
// - MPU6050 초기화 (PWR_MGMT_1 = 0x00)
// - ACCEL_XOUT_H부터 14바이트 burst read
// - AX, AY, AZ, GX, GY, GZ raw 6축 출력
//
// 핵심 수정
// 1) start_req는 tick 기준 pulse로 생성
// 2) i2c_master의 done이 짧은 pulse여도 놓치지 않도록 done_latched 사용
// 3) rx_buf는 byte0가 [7:0]에 저장되는 LSB-first packing 기준으로 파싱
//==============================================================================

module mpu6050_ctrl
(
    input  wire         clk,
    input  wire         rst_n,
    input  wire         tick,

    // i2c_master status
    input  wire         busy,
    input  wire         done,
    input  wire         ack_ok,
    input  wire [2:0]   err_code,
    input  wire [127:0] rx_buf,
    input  wire [7:0]   rx_count,

    // i2c_master control
    output reg          start_req,
    output reg          rw,
    output reg [7:0]    reg_addr,
    output reg [7:0]    tx_data,
    output reg [7:0]    burst_len,

    // status
    output reg          init_done,
    output reg          data_valid,
    output reg [2:0]    last_err,

    // raw sensor outputs
    output reg signed [15:0] ax,
    output reg signed [15:0] ay,
    output reg signed [15:0] az,
    output reg signed [15:0] gx,
    output reg signed [15:0] gy,
    output reg signed [15:0] gz
);

    localparam [7:0] REG_PWR_MGMT_1   = 8'h6B;
    localparam [7:0] REG_ACCEL_XOUT_H = 8'h3B;

    localparam [3:0]
        ST_RESET_WAIT      = 4'd0,
        ST_INIT_REQ        = 4'd1,
        ST_INIT_WAIT_BUSY  = 4'd2,
        ST_INIT_WAIT_DONE  = 4'd3,
        ST_INIT_CHECK      = 4'd4,
        ST_WAIT_PERIOD     = 4'd5,
        ST_READ_REQ        = 4'd6,
        ST_READ_WAIT_BUSY  = 4'd7,
        ST_READ_WAIT_DONE  = 4'd8,
        ST_READ_CHECK      = 4'd9;

    reg [3:0]  state;
    reg [15:0] wait_cnt;

    // i2c_master의 done pulse를 놓치지 않기 위한 latch
    reg done_latched;

    // 10us tick 기준
    // 5000 -> 50ms
    localparam [15:0] RESET_WAIT_TICKS  = 16'd5000;

    // 10us tick 기준
    // 500 -> 5ms
    localparam [15:0] READ_PERIOD_TICKS = 16'd500;

    //--------------------------------------------------------------------------
    // done pulse latch
    // - i2c_master의 done이 짧게 올라와도
    //   WAIT_DONE 상태에서 확실히 인식할 수 있도록 보관한다.
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            done_latched <= 1'b0;
        end
        else begin
            if (done) begin
                done_latched <= 1'b1;
            end
            else if ((state != ST_INIT_WAIT_DONE) && (state != ST_READ_WAIT_DONE)) begin
                done_latched <= 1'b0;
            end
        end
    end

    //--------------------------------------------------------------------------
    // main FSM
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= ST_RESET_WAIT;
            wait_cnt   <= 16'd0;

            start_req  <= 1'b0;
            rw         <= 1'b0;
            reg_addr   <= 8'd0;
            tx_data    <= 8'd0;
            burst_len  <= 8'd0;

            init_done  <= 1'b0;
            data_valid <= 1'b0;
            last_err   <= 3'd0;

            ax         <= 16'sd0;
            ay         <= 16'sd0;
            az         <= 16'sd0;
            gx         <= 16'sd0;
            gy         <= 16'sd0;
            gz         <= 16'sd0;
        end
        else begin
            // 기본값
            start_req  <= 1'b0;
            data_valid <= 1'b0;

            if (tick) begin
                case (state)

                    //==========================================================
                    // 전원 안정화 대기
                    //==========================================================
                    ST_RESET_WAIT: begin
                        init_done <= 1'b0;

                        if (wait_cnt < RESET_WAIT_TICKS) begin
                            wait_cnt <= wait_cnt + 16'd1;
                        end
                        else begin
                            wait_cnt <= 16'd0;
                            state    <= ST_INIT_REQ;
                        end
                    end

                    //==========================================================
                    // INIT WRITE 요청
                    //==========================================================
                    ST_INIT_REQ: begin
                        rw        <= 1'b0;          // write
                        reg_addr  <= REG_PWR_MGMT_1;
                        tx_data   <= 8'h00;         // sleep 해제
                        burst_len <= 8'd0;

                        // i2c_master에 transaction 시작 요청
                        start_req <= 1'b1;
                        state     <= ST_INIT_WAIT_BUSY;
                    end

                    //==========================================================
                    // master busy 진입 대기
                    //==========================================================
                    ST_INIT_WAIT_BUSY: begin
                        if (busy)
                            state <= ST_INIT_WAIT_DONE;
                    end

                    //==========================================================
                    // init done latch 대기
                    //==========================================================
                    ST_INIT_WAIT_DONE: begin
                        if (done_latched)
                            state <= ST_INIT_CHECK;
                    end

                    //==========================================================
                    // init 결과 확인
                    //==========================================================
                    ST_INIT_CHECK: begin
                        if (ack_ok) begin
                            init_done <= 1'b1;
                            wait_cnt  <= 16'd0;
                            state     <= ST_WAIT_PERIOD;
                        end
                        else begin
                            init_done <= 1'b0;
                            last_err  <= err_code;
                            wait_cnt  <= 16'd0;
                            state     <= ST_RESET_WAIT;
                        end
                    end

                    //==========================================================
                    // 다음 burst read까지 대기
                    //==========================================================
                    ST_WAIT_PERIOD: begin
                        if (wait_cnt < READ_PERIOD_TICKS) begin
                            wait_cnt <= wait_cnt + 16'd1;
                        end
                        else begin
                            wait_cnt <= 16'd0;
                            state    <= ST_READ_REQ;
                        end
                    end

                    //==========================================================
                    // BURST READ 요청
                    //==========================================================
                    ST_READ_REQ: begin
                        rw        <= 1'b1;             // read
                        reg_addr  <= REG_ACCEL_XOUT_H; // 0x3B부터
                        tx_data   <= 8'h00;
                        burst_len <= 8'd14;

                        start_req <= 1'b1;
                        state     <= ST_READ_WAIT_BUSY;
                    end

                    //==========================================================
                    // read busy 진입 대기
                    //==========================================================
                    ST_READ_WAIT_BUSY: begin
                        if (busy)
                            state <= ST_READ_WAIT_DONE;
                    end

                    //==========================================================
                    // read done latch 대기
                    //==========================================================
                    ST_READ_WAIT_DONE: begin
                        if (done_latched)
                            state <= ST_READ_CHECK;
                    end

                    //==========================================================
                    // read 결과 확인 및 파싱
                    //==========================================================
                    ST_READ_CHECK: begin
                        if (ack_ok && (rx_count == 8'd14)) begin
                            // rx_buf packing
                            // [7:0]    = byte0  = ACCEL_XOUT_H
                            // [15:8]   = byte1  = ACCEL_XOUT_L
                            // [23:16]  = byte2  = ACCEL_YOUT_H
                            // [31:24]  = byte3  = ACCEL_YOUT_L
                            // [39:32]  = byte4  = ACCEL_ZOUT_H
                            // [47:40]  = byte5  = ACCEL_ZOUT_L
                            // [55:48]  = byte6  = TEMP_OUT_H
                            // [63:56]  = byte7  = TEMP_OUT_L
                            // [71:64]  = byte8  = GYRO_XOUT_H
                            // [79:72]  = byte9  = GYRO_XOUT_L
                            // [87:80]  = byte10 = GYRO_YOUT_H
                            // [95:88]  = byte11 = GYRO_YOUT_L
                            // [103:96] = byte12 = GYRO_ZOUT_H
                            // [111:104]= byte13 = GYRO_ZOUT_L
                            ax <= {rx_buf[7:0],    rx_buf[15:8]};
                            ay <= {rx_buf[23:16],  rx_buf[31:24]};
                            az <= {rx_buf[39:32],  rx_buf[47:40]};
                            gx <= {rx_buf[71:64],  rx_buf[79:72]};
                            gy <= {rx_buf[87:80],  rx_buf[95:88]};
                            gz <= {rx_buf[103:96], rx_buf[111:104]};

                            data_valid <= 1'b1;
                        end
                        else begin
                            last_err <= err_code;
                        end

                        wait_cnt <= 16'd0;
                        state    <= ST_WAIT_PERIOD;
                    end

                    default: begin
                        state <= ST_RESET_WAIT;
                    end
                endcase
            end
        end
    end

endmodule
