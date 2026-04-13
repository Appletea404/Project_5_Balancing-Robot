`timescale 1ns / 1ps
//================================================================================
// top
// - Basys3 보드용 최상위 모듈
// - clk_divider, mpu6050_ctrl, i2c_master 연결
// - I2C open-drain 핀 제어
// - 현재는 LED로 MPU6050 상태를 간단히 확인
//
// LED 해석
//   led[7:0] : accel_x[15:8] (X축 가속도 상위 바이트)
//   led[8]   : init_done     (초기화 완료)
//   led[9]   : data_valid    (데이터 갱신 완료 pulse 성격)
//================================================================================

module top
(
    input  wire       clk,
    input  wire       rst_n,

    inout  wire       i2c_sda,
    inout  wire       i2c_scl,

    output wire [9:0] led
);

//================================================================================
// 내부 신호
//================================================================================
wire tick;

// mpu6050_ctrl <-> i2c_master
wire        start_req;
wire        rw;
wire [7:0]  reg_addr;
wire [7:0]  tx_data;
wire        busy;
wire        done;
wire        ack_ok;
wire [7:0]  rx_data;

// i2c 핀 제어
wire sda_enable;
wire scl_enable;

// MPU6050 데이터
wire signed [15:0] accel_x;
wire signed [15:0] accel_y;
wire signed [15:0] accel_z;
wire               init_done;
wire               data_valid;

//================================================================================
// clk_divider
//================================================================================
clk_divider u_clk_divider
(
    .clk   (clk),
    .rst_n (rst_n),
    .tick  (tick)
);

//================================================================================
// mpu6050_ctrl
// - MPU6050 초기화 + accel 3축 읽기 FSM
//================================================================================
mpu6050_ctrl u_mpu6050_ctrl
(
    .clk        (clk),
    .rst_n      (rst_n),
    .tick       (tick),

    .busy       (busy),
    .done       (done),
    .ack_ok     (ack_ok),
    .rx_data    (rx_data),

    .start_req  (start_req),
    .rw         (rw),
    .reg_addr   (reg_addr),
    .tx_data    (tx_data),

    .accel_x    (accel_x),
    .accel_y    (accel_y),
    .accel_z    (accel_z),
    .init_done  (init_done),
    .data_valid (data_valid)
);

//================================================================================
// i2c_master
// - 1-byte register read / write 지원
//================================================================================
i2c_master u_i2c_master
(
    .clk        (clk),
    .rst_n      (rst_n),
    .tick       (tick),

    .start_req  (start_req),
    .rw         (rw),
    .reg_addr   (reg_addr),
    .tx_data    (tx_data),
    .sda_in     (i2c_sda),

    .busy       (busy),
    .done       (done),
    .ack_ok     (ack_ok),
    .rx_data    (rx_data),

    .sda_enable (sda_enable),
    .scl_enable (scl_enable)
);

//================================================================================
// I2C open-drain 연결
// enable = 1 -> Low로 당김
// enable = 0 -> release(Z), 외부 pull-up에 의해 High
//================================================================================
assign i2c_sda = (sda_enable) ? 1'b0 : 1'bz;
assign i2c_scl = (scl_enable) ? 1'b0 : 1'bz;

//================================================================================
// LED 출력
// - accel_x의 상위 바이트를 표시
// - init_done / data_valid 상태 확인
//================================================================================
assign led[7:0] = accel_x[15:8];
assign led[8]   = init_done;
assign led[9]   = data_valid;

endmodule