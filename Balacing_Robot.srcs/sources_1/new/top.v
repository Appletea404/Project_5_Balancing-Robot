
`timescale 1ns / 1ps
//================================================================================
// top
// - Basys3 보드용 최상위 모듈
// - MPU6050 + I2C master + UART debug
// - top 내부에 bias 보정 로직 포함
// - 기존 led[9:0] 유지
// - 추가 led[15:10]으로 방향 표시
//
// 방향 LED 정책
// - front / back / left / right : 기울기 방향 표시
// - up / down : Z축 자세 표시
//
// 주의
// - "위(up)"는 평평하게 정상 방향으로 놓였을 때 켜지는 것이 정상
// - "아래(down)"는 뒤집혔을 때 켜짐
//================================================================================

module top
(
    input  wire        clk,
    input  wire        rst_n,

    inout  wire        i2c_sda,
    inout  wire        i2c_scl,

    output wire        uart_tx,
    output wire [15:0] led
);

    //--------------------------------------------------------------------------
    // 공통 tick
    //--------------------------------------------------------------------------
    wire         tick;

    //--------------------------------------------------------------------------
    // mpu6050_ctrl <-> i2c_master 제어 신호
    //--------------------------------------------------------------------------
    wire         start_req;
    wire         rw;
    wire [7:0]   reg_addr;
    wire [7:0]   tx_data;
    wire [7:0]   burst_len;

    //--------------------------------------------------------------------------
    // i2c_master 상태 신호
    //--------------------------------------------------------------------------
    wire         busy;
    wire         done;
    wire         ack_ok;
    wire [2:0]   err_code;
    wire [7:0]   rx_data;
    wire [127:0] rx_buf;
    wire [7:0]   rx_count;

    //--------------------------------------------------------------------------
    // open-drain 제어
    //--------------------------------------------------------------------------
    wire         sda_enable;
    wire         scl_enable;

    //--------------------------------------------------------------------------
    // MPU raw 데이터
    //--------------------------------------------------------------------------
    wire signed [15:0] ax_raw;
    wire signed [15:0] ay_raw;
    wire signed [15:0] az_raw;
    wire signed [15:0] gx_raw;
    wire signed [15:0] gy_raw;
    wire signed [15:0] gz_raw;

    //--------------------------------------------------------------------------
    // 보정 후 데이터
    //--------------------------------------------------------------------------
    reg                bias_done;

    reg signed [15:0]  ax_bias;
    reg signed [15:0]  ay_bias;
    reg signed [15:0]  az_bias;
    reg signed [15:0]  gx_bias;
    reg signed [15:0]  gy_bias;
    reg signed [15:0]  gz_bias;

    reg signed [15:0]  ax_corr;
    reg signed [15:0]  ay_corr;
    reg signed [15:0]  az_corr;
    reg signed [15:0]  gx_corr;
    reg signed [15:0]  gy_corr;
    reg signed [15:0]  gz_corr;

    //--------------------------------------------------------------------------
    // bias 계산용 누적합
    //--------------------------------------------------------------------------
    reg [7:0]          cal_cnt;
    reg signed [24:0]  ax_sum;
    reg signed [24:0]  ay_sum;
    reg signed [24:0]  az_sum;
    reg signed [24:0]  gx_sum;
    reg signed [24:0]  gy_sum;
    reg signed [24:0]  gz_sum;

    //--------------------------------------------------------------------------
    // 상위 상태
    //--------------------------------------------------------------------------
    wire               init_done;
    wire               data_valid;
    wire [2:0]         last_err;

    //--------------------------------------------------------------------------
    // UART 출력선
    //--------------------------------------------------------------------------
    wire               uart_tx_wire;

    //--------------------------------------------------------------------------
    // 방향 판단 threshold
    //--------------------------------------------------------------------------
    localparam signed [15:0] TILT_TH = 16'sd1000;   // 앞/뒤/좌/우 판정
    localparam signed [15:0] Z_TH    = 16'sd12000;  // 위/아래 판정 (약 0.73g)

    //--------------------------------------------------------------------------
    // 방향 표시 신호
    //--------------------------------------------------------------------------
    wire dir_front;
    wire dir_back;
    wire dir_left;
    wire dir_right;
    wire dir_up;
    wire dir_down;

    //--------------------------------------------------------------------------
    // 방향 판정
    //
    // 실제 보드 방향에 따라 front/back, left/right 부호가 반대일 수 있음
    // 반대로 나오면 해당 두 줄만 서로 바꾸면 됨
    //--------------------------------------------------------------------------
    assign dir_front = (ax_corr >  TILT_TH);
    assign dir_back  = (ax_corr < -TILT_TH);

    assign dir_right = (ay_corr >  TILT_TH);
    assign dir_left  = (ay_corr < -TILT_TH);

    // 위/아래는 평평한 자세 상태 판정
    // up   : 윗면이 위로 향함 (정상 방향)
    // down : 뒤집힘
    assign dir_up    = (az_corr >  Z_TH);
    assign dir_down  = (az_corr < -Z_TH);

    //==========================================================================
    // tick 생성
    //==========================================================================
    clk_divider u_clk_divider
    (
        .clk   (clk),
        .rst_n (rst_n),
        .tick  (tick)
    );

    //==========================================================================
    // MPU6050 상위 제어
    //==========================================================================
    mpu6050_ctrl u_mpu6050_ctrl
    (
        .clk        (clk),
        .rst_n      (rst_n),
        .tick       (tick),

        .busy       (busy),
        .done       (done),
        .ack_ok     (ack_ok),
        .err_code   (err_code),
        .rx_buf     (rx_buf),
        .rx_count   (rx_count),

        .start_req  (start_req),
        .rw         (rw),
        .reg_addr   (reg_addr),
        .tx_data    (tx_data),
        .burst_len  (burst_len),

        .init_done  (init_done),
        .data_valid (data_valid),
        .last_err   (last_err),

        .ax         (ax_raw),
        .ay         (ay_raw),
        .az         (az_raw),
        .gx         (gx_raw),
        .gy         (gy_raw),
        .gz         (gz_raw)
    );

    //==========================================================================
    // I2C master
    //==========================================================================
    i2c_master u_i2c_master
    (
        .clk        (clk),
        .rst_n      (rst_n),
        .tick       (tick),

        .start_req  (start_req),
        .rw         (rw),
        .reg_addr   (reg_addr),
        .tx_data    (tx_data),
        .burst_len  (burst_len),
        .sda_in     (i2c_sda),

        .busy       (busy),
        .done       (done),
        .ack_ok     (ack_ok),
        .err_code   (err_code),
        .rx_data    (rx_data),
        .rx_buf     (rx_buf),
        .rx_count   (rx_count),

        .sda_enable (sda_enable),
        .scl_enable (scl_enable)
    );

    //==========================================================================
    // bias 계산 + corrected 값 생성
    // - data_valid가 올라올 때만 샘플 사용
    // - 처음 256개 평균으로 bias 계산
    //==========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bias_done <= 1'b0;
            cal_cnt   <= 8'd0;

            ax_sum    <= 25'sd0;
            ay_sum    <= 25'sd0;
            az_sum    <= 25'sd0;
            gx_sum    <= 25'sd0;
            gy_sum    <= 25'sd0;
            gz_sum    <= 25'sd0;

            ax_bias   <= 16'sd0;
            ay_bias   <= 16'sd0;
            az_bias   <= 16'sd0;
            gx_bias   <= 16'sd0;
            gy_bias   <= 16'sd0;
            gz_bias   <= 16'sd0;

            ax_corr   <= 16'sd0;
            ay_corr   <= 16'sd0;
            az_corr   <= 16'sd0;
            gx_corr   <= 16'sd0;
            gy_corr   <= 16'sd0;
            gz_corr   <= 16'sd0;
        end
        else begin
            //------------------------------------------------------------------
            // bias 계산 전
            //------------------------------------------------------------------
            if (!bias_done) begin
                if (data_valid) begin
                    ax_sum <= ax_sum + ax_raw;
                    ay_sum <= ay_sum + ay_raw;
                    az_sum <= az_sum + az_raw;
                    gx_sum <= gx_sum + gx_raw;
                    gy_sum <= gy_sum + gy_raw;
                    gz_sum <= gz_sum + gz_raw;

                    if (cal_cnt == 8'hFF) begin
                        // 평균 = 합 / 256 = >>> 8
                        ax_bias   <= ax_sum >>> 8;
                        ay_bias   <= ay_sum >>> 8;
                        az_bias   <= az_sum >>> 8;
                        gx_bias   <= gx_sum >>> 8;
                        gy_bias   <= gy_sum >>> 8;
                        gz_bias   <= gz_sum >>> 8;
                        bias_done <= 1'b1;
                    end
                    else begin
                        cal_cnt <= cal_cnt + 8'd1;
                    end
                end

                // bias 계산 전에는 raw 그대로 출력
                ax_corr <= ax_raw;
                ay_corr <= ay_raw;
                az_corr <= az_raw;
                gx_corr <= gx_raw;
                gy_corr <= gy_raw;
                gz_corr <= gz_raw;
            end
            //------------------------------------------------------------------
            // bias 계산 후
            //------------------------------------------------------------------
            else begin
                // AX, AY는 offset 제거
                ax_corr <= ax_raw - ax_bias;
                ay_corr <= ay_raw - ay_bias;

                // AZ는 초기 단계에서는 raw 유지
                az_corr <= az_raw;

                // 자이로 bias 제거
                gx_corr <= gx_raw - gx_bias;
                gy_corr <= gy_raw - gy_bias;
                gz_corr <= gz_raw - gz_bias;
            end
        end
    end

    //==========================================================================
    // UART debug
    // - corrected 값 출력
    //==========================================================================
    mpu6050_debug_uart u_debug_uart
    (
        .clk        (clk),
        .rst_n      (rst_n),
        .init_done  (init_done),
        .data_valid (data_valid),

        .accel_x    (ax_corr),
        .accel_y    (ay_corr),
        .accel_z    (az_corr),
        .gyro_x     (gx_corr),
        .gyro_y     (gy_corr),
        .gyro_z     (gz_corr),

        .uart_tx_o  (uart_tx_wire)
    );

    //==========================================================================
    // open-drain I2C
    //==========================================================================
    assign i2c_sda = (sda_enable) ? 1'b0 : 1'bz;
    assign i2c_scl = (scl_enable) ? 1'b0 : 1'bz;

    //==========================================================================
    // UART 출력
    //==========================================================================
    assign uart_tx = uart_tx_wire;

    //==========================================================================
    // 기존 LED 유지
    // led[0] : rst_n
    // led[1] : init_done
    // led[2] : bias_done
    // led[3] : ack_ok
    // led[4] : 0
    // led[5] : 0
    // led[6] : 0
    // led[7] : 0
    // led[8] : 0
    // led[9] : uart_tx_wire
    //==========================================================================
    assign led[0] = rst_n;
    assign led[1] = init_done;
    assign led[2] = bias_done;
    assign led[3] = ack_ok;
    assign led[4] = 1'b0;
    assign led[5] = 1'b0;
    assign led[6] = 1'b0;
    assign led[7] = 1'b0;
    assign led[8] = 1'b0;
    assign led[9] = uart_tx_wire;

    //==========================================================================
    // 추가 방향 LED
    // led[10] : front
    // led[11] : back
    // led[12] : left
    // led[13] : right
    // led[14] : up
    // led[15] : down
    //==========================================================================
    assign led[10] = dir_front;
    assign led[11] = dir_back;
    assign led[12] = dir_left;
    assign led[13] = dir_right;
    assign led[14] = dir_up;
    assign led[15] = dir_down;

endmodule




// // Accel +- 2g Gyro +-250도/s
// `timescale 1ns / 1ps
// //================================================================================
// // top
// // - Basys3 보드용 최상위 모듈
// // - MPU6050 + I2C master + UART debug
// // - top 내부에 bias 보정 로직 포함
// //
// // 보정 방식
// // - data_valid가 들어올 때마다 raw 6축 샘플 누적
// // - 처음 256개 샘플 평균으로 bias 계산
// // - 이후 corrected = raw - bias 형태로 출력
// //
// // 현재 정책
// // - 자이로(GX/GY/GZ)는 bias 제거
// // - 가속도 AX/AY는 offset 제거
// // - AZ는 초기 단계에서는 raw 유지
// //================================================================================

// module top
// (
//     input  wire       clk,
//     input  wire       rst_n,

//     inout  wire       i2c_sda,
//     inout  wire       i2c_scl,

//     output wire       uart_tx,
//     output wire [9:0] led
// );

//     //--------------------------------------------------------------------------
//     // 공통 tick
//     //--------------------------------------------------------------------------
//     wire         tick;

//     //--------------------------------------------------------------------------
//     // mpu6050_ctrl <-> i2c_master 제어 신호
//     //--------------------------------------------------------------------------
//     wire         start_req;
//     wire         rw;
//     wire [7:0]   reg_addr;
//     wire [7:0]   tx_data;
//     wire [7:0]   burst_len;

//     //--------------------------------------------------------------------------
//     // i2c_master 상태 신호
//     //--------------------------------------------------------------------------
//     wire         busy;
//     wire         done;
//     wire         ack_ok;
//     wire [2:0]   err_code;
//     wire [7:0]   rx_data;
//     wire [127:0] rx_buf;
//     wire [7:0]   rx_count;

//     //--------------------------------------------------------------------------
//     // open-drain 제어
//     //--------------------------------------------------------------------------
//     wire         sda_enable;
//     wire         scl_enable;

//     //--------------------------------------------------------------------------
//     // MPU raw 데이터
//     //--------------------------------------------------------------------------
//     wire signed [15:0] ax_raw;
//     wire signed [15:0] ay_raw;
//     wire signed [15:0] az_raw;
//     wire signed [15:0] gx_raw;
//     wire signed [15:0] gy_raw;
//     wire signed [15:0] gz_raw;

//     //--------------------------------------------------------------------------
//     // 보정 후 데이터
//     //--------------------------------------------------------------------------
//     reg                bias_done;

//     reg signed [15:0]  ax_bias;
//     reg signed [15:0]  ay_bias;
//     reg signed [15:0]  az_bias;
//     reg signed [15:0]  gx_bias;
//     reg signed [15:0]  gy_bias;
//     reg signed [15:0]  gz_bias;

//     reg signed [15:0]  ax_corr;
//     reg signed [15:0]  ay_corr;
//     reg signed [15:0]  az_corr;
//     reg signed [15:0]  gx_corr;
//     reg signed [15:0]  gy_corr;
//     reg signed [15:0]  gz_corr;

//     //--------------------------------------------------------------------------
//     // bias 계산용 누적합
//     //--------------------------------------------------------------------------
//     reg [7:0]          cal_cnt;
//     reg signed [23:0]  ax_sum;
//     reg signed [23:0]  ay_sum;
//     reg signed [23:0]  az_sum;
//     reg signed [23:0]  gx_sum;
//     reg signed [23:0]  gy_sum;
//     reg signed [23:0]  gz_sum;

//     //--------------------------------------------------------------------------
//     // 상위 상태
//     //--------------------------------------------------------------------------
//     wire               init_done;
//     wire               data_valid;
//     wire [2:0]         last_err;

//     //--------------------------------------------------------------------------
//     // UART 출력선
//     //--------------------------------------------------------------------------
//     wire               uart_tx_wire;

//     //==========================================================================
//     // tick 생성
//     //==========================================================================
//     clk_divider u_clk_divider
//     (
//         .clk   (clk),
//         .rst_n (rst_n),
//         .tick  (tick)
//     );

//     //==========================================================================
//     // MPU6050 상위 제어
//     //==========================================================================
//     mpu6050_ctrl u_mpu6050_ctrl
//     (
//         .clk        (clk),
//         .rst_n      (rst_n),
//         .tick       (tick),

//         .busy       (busy),
//         .done       (done),
//         .ack_ok     (ack_ok),
//         .err_code   (err_code),
//         .rx_buf     (rx_buf),
//         .rx_count   (rx_count),

//         .start_req  (start_req),
//         .rw         (rw),
//         .reg_addr   (reg_addr),
//         .tx_data    (tx_data),
//         .burst_len  (burst_len),

//         .init_done  (init_done),
//         .data_valid (data_valid),
//         .last_err   (last_err),

//         .ax         (ax_raw),
//         .ay         (ay_raw),
//         .az         (az_raw),
//         .gx         (gx_raw),
//         .gy         (gy_raw),
//         .gz         (gz_raw)
//     );

//     //==========================================================================
//     // I2C master
//     //==========================================================================
//     i2c_master u_i2c_master
//     (
//         .clk        (clk),
//         .rst_n      (rst_n),
//         .tick       (tick),

//         .start_req  (start_req),
//         .rw         (rw),
//         .reg_addr   (reg_addr),
//         .tx_data    (tx_data),
//         .burst_len  (burst_len),
//         .sda_in     (i2c_sda),

//         .busy       (busy),
//         .done       (done),
//         .ack_ok     (ack_ok),
//         .err_code   (err_code),
//         .rx_data    (rx_data),
//         .rx_buf     (rx_buf),
//         .rx_count   (rx_count),

//         .sda_enable (sda_enable),
//         .scl_enable (scl_enable)
//     );

//     //==========================================================================
//     // bias 계산 + corrected 값 생성
//     // - data_valid가 올라올 때만 샘플을 사용
//     // - 처음 256개 평균으로 bias 계산
//     //==========================================================================
//     always @(posedge clk or negedge rst_n) begin
//         if (!rst_n) begin
//             bias_done <= 1'b0;
//             cal_cnt   <= 8'd0;

//             ax_sum    <= 24'sd0;
//             ay_sum    <= 24'sd0;
//             az_sum    <= 24'sd0;
//             gx_sum    <= 24'sd0;
//             gy_sum    <= 24'sd0;
//             gz_sum    <= 24'sd0;

//             ax_bias   <= 16'sd0;
//             ay_bias   <= 16'sd0;
//             az_bias   <= 16'sd0;
//             gx_bias   <= 16'sd0;
//             gy_bias   <= 16'sd0;
//             gz_bias   <= 16'sd0;

//             ax_corr   <= 16'sd0;
//             ay_corr   <= 16'sd0;
//             az_corr   <= 16'sd0;
//             gx_corr   <= 16'sd0;
//             gy_corr   <= 16'sd0;
//             gz_corr   <= 16'sd0;
//         end
//         else begin
//             //------------------------------------------------------------------
//             // bias 계산 전
//             //------------------------------------------------------------------
//             if (!bias_done) begin
//                 if (data_valid) begin
//                     ax_sum <= ax_sum + ax_raw;
//                     ay_sum <= ay_sum + ay_raw;
//                     az_sum <= az_sum + az_raw;
//                     gx_sum <= gx_sum + gx_raw;
//                     gy_sum <= gy_sum + gy_raw;
//                     gz_sum <= gz_sum + gz_raw;

//                     if (cal_cnt == 8'hFF) begin
//                         // 평균 = 합 / 256 = >> 8
//                         ax_bias   <= ax_sum[23:8];
//                         ay_bias   <= ay_sum[23:8];
//                         az_bias   <= az_sum[23:8];
//                         gx_bias   <= gx_sum[23:8];
//                         gy_bias   <= gy_sum[23:8];
//                         gz_bias   <= gz_sum[23:8];
//                         bias_done <= 1'b1;
//                     end
//                     else begin
//                         cal_cnt <= cal_cnt + 8'd1;
//                     end
//                 end

//                 // bias 계산 전에는 raw 그대로 출력
//                 ax_corr <= ax_raw;
//                 ay_corr <= ay_raw;
//                 az_corr <= az_raw;
//                 gx_corr <= gx_raw;
//                 gy_corr <= gy_raw;
//                 gz_corr <= gz_raw;
//             end
//             //------------------------------------------------------------------
//             // bias 계산 후
//             //------------------------------------------------------------------
//             else begin
//                 // AX, AY는 offset 제거
//                 ax_corr <= ax_raw - ax_bias;
//                 ay_corr <= ay_raw - ay_bias;

//                 // AZ는 초기 단계에서는 raw 유지
//                 // 추후 필요 시 az_raw - az_bias + 16'sd16384 형태로 변경 가능
//                 az_corr <= az_raw;

//                 // 자이로 bias 제거
//                 gx_corr <= gx_raw - gx_bias;
//                 gy_corr <= gy_raw - gy_bias;
//                 gz_corr <= gz_raw - gz_bias;
//             end
//         end
//     end

//     //==========================================================================
//     // UART debug
//     // - raw 대신 corrected 값 출력
//     //==========================================================================
//     mpu6050_debug_uart u_debug_uart
//     (
//         .clk        (clk),
//         .rst_n      (rst_n),
//         .init_done  (init_done),
//         .data_valid (data_valid),

//         .accel_x    (ax_corr),
//         .accel_y    (ay_corr),
//         .accel_z    (az_corr),
//         .gyro_x     (gx_corr),
//         .gyro_y     (gy_corr),
//         .gyro_z     (gz_corr),

//         .uart_tx_o  (uart_tx_wire)
//     );

//     //==========================================================================
//     // open-drain I2C
//     //==========================================================================
//     assign i2c_sda = (sda_enable) ? 1'b0 : 1'bz;
//     assign i2c_scl = (scl_enable) ? 1'b0 : 1'bz;

//     //==========================================================================
//     // UART 출력
//     //==========================================================================
//     assign uart_tx = uart_tx_wire;

//     //==========================================================================
//     // LED 디버그
//     // led[0] : rst_n
//     // led[1] : init_done
//     // led[2] : busy
//     // led[3] : ack_ok
//     // led[4] : data_valid
//     // led[5] : bias_done
//     // led[6] : last_err[0]
//     // led[7] : last_err[1]
//     // led[8] : last_err[2]
//     // led[9] : uart_tx_wire
//     //==========================================================================
//     assign led[0] = rst_n;
//     assign led[1] = init_done;
//     assign led[2] = busy;
//     assign led[3] = ack_ok;
//     assign led[4] = data_valid;
//     assign led[5] = bias_done;
//     assign led[6] = last_err[0];
//     assign led[7] = last_err[1];
//     assign led[8] = last_err[2];
//     assign led[9] = uart_tx_wire;

// endmodule