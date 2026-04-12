`timescale 1ns / 1ps
//================================================================================
// top -> Basys3 보드의 최상위 모듈
//        하위 모듈을 연결하고 I2C, LED, reset 등의 외부 핀을 관리
//================================================================================





module top
(
    input wire clk,
    input wire rst_n,

    inout wire i2c_sda,
    inout wire i2c_scl
    
);


// 내부 신호
wire tick;
wire sda_enable;
wire scl_enable;


// clk_divider 인스턴스
clk_divider u_clk_divider
(
    .clk        (clk),
    .rst_n      (rst_n),
    .tick       (tick)
);

// i2c_master 인스턴스
i2c_master u_i2c_master
(
    .clk           (clk),
    .rst_n         (rst_n),
    .tick          (tick),
    .sda_enable    (sda_enable),
    .scl_enable    (scl_enable)
);


//================================================================================
// open-drain 연결
// enable = 1 -> 라인을 Low로 당김
// enable = 0 -> release(Z), 외부 pull-up에 의해 High
//================================================================================
assign i2c_sda = (sda_enable) ? 1'b0 : 1'bz;
assign i2c_scl = (scl_enable) ? 1'b0 : 1'bz;


endmodule
