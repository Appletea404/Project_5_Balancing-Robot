`timescale 1ns / 1ps
//================================================================================
// top -> Basys3 보드의 최상위 모듈
//        clk_divider, mpu6050_ctrl, i2c_master를 연결하고
//        I2C open-drain 핀을 관리
//================================================================================

module top
(
    input  wire clk,
    input  wire rst_n,

    inout  wire i2c_sda,
    inout  wire i2c_scl,


    // test용
    output wire [9:0] led
);

//================================================================================
// 내부 신호
//================================================================================
wire tick;

// mpu6050_ctrl <-> i2c_master
wire        start_req;
wire [7:0]  reg_addr;
wire        busy;
wire        done;
wire        ack_ok;
wire [7:0]  rx_data;

// i2c 핀 제어
wire sda_enable;
wire scl_enable;

// 결과
wire [7:0] who_am_i;
wire       valid;

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
//================================================================================
mpu6050_ctrl u_mpu6050_ctrl
(
    .clk       (clk),
    .rst_n     (rst_n),
    .tick      (tick),

    .busy      (busy),
    .done      (done),
    .ack_ok    (ack_ok),
    .rx_data   (rx_data),

    .start_req (start_req),
    .reg_addr  (reg_addr),

    .who_am_i  (who_am_i),
    .valid     (valid)
);

//================================================================================
// i2c_master
// - reg_addr에 지정된 1-byte register를 읽는 WHO_AM_I 확인용 I2C 엔진
//================================================================================
i2c_master u_i2c_master
(
    .clk        (clk),
    .rst_n      (rst_n),
    .tick       (tick),

    .start_req  (start_req),
    .reg_addr   (reg_addr),
    .sda_in     (i2c_sda),

    .busy       (busy),
    .done       (done),
    .ack_ok     (ack_ok),
    .rx_data    (rx_data),

    .sda_enable (sda_enable),
    .scl_enable (scl_enable)
);

//================================================================================
// open-drain 연결
// enable = 1 -> 라인을 Low로 당김
// enable = 0 -> release(Z), 외부 pull-up에 의해 High
//================================================================================
assign i2c_sda = (sda_enable) ? 1'b0 : 1'bz;
assign i2c_scl = (scl_enable) ? 1'b0 : 1'bz;

//================================================================================
// Basys3 LED 출력
// led[7:0] : WHO_AM_I 값
// led[8]   : valid
// led[9]   : ack_ok
//================================================================================
assign led[7:0] = who_am_i;
assign led[8]   = valid;
assign led[9]   = ack_ok;

endmodule