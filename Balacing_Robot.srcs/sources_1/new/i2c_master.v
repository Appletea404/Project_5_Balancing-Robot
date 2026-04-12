`timescale 1ns / 1ps
//================================================================================
// i2c_master -> I2C 프로토콜의 START, STOP, WRITE, READ, ACK 처리를 담당하는 모듈
//================================================================================
module i2c_master
(
    input wire clk,             // System clock
    input wire rst_n,           // Active low reset
    input wire tick,            // Slow tick from clk_divider

    output reg sda_enable,      // 1: SDA Low, 0: SDA Release
    output reg scl_enable       // 1: SCL Low, 0: SCL Release
);

//================================================================================
// 상태 정의
//================================================================================
localparam IDLE  = 2'd0;
localparam START = 2'd1;
localparam STOP  = 2'd2;

reg [1:0] state;

//================================================================================
// FSM
//================================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state       <= IDLE;    
        sda_enable  <= 1'b0;    // release -> High
        scl_enable  <= 1'b0;    // release -> High
    end

    else begin
        if (tick) begin
            case (state)
                IDLE: begin
                    // I2C idle 상태: SDA, SCL 둘 다 release
                    sda_enable <= 1'b0;
                    scl_enable <= 1'b0;
                    state      <= START;
                end 

                START: begin
                    // START 조건:
                    // SCL: High 일때 SDA: High -> Low
                    sda_enable <= 1'b1;     // SDA Low
                    scl_enable <= 1'b0;     // SCL High(release)
                    state  <= STOP;
                end

                STOP: begin
                    // STOP 조건:
                    // SCL: High 일때 SDA: Low -> High
                    sda_enable <= 1'b0;     // SDA release -> High
                    scl_enable <= 1'b0;     // SCL High(release)
                    state  <= IDLE;
                end

                default: begin
                    state      <= IDLE;
                    sda_enable <= 1'b0;
                    scl_enable <= 1'b0;
                end 
            endcase
        end
    end
end



endmodule
