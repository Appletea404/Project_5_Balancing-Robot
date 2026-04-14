`timescale 1ns / 1ps

module JGB37_520(

    input clk,
    input reset,

    input sw_A_cw,
    input sw_A_ccw,
    input sw_B_cw,
    input sw_B_ccw,

    output PWMA_out,
    output PWMB_out,
    output AIN1_out,
    output AIN2_out,
    output BIN1_out,
    output BIN2_out
    );

    wire [1:0] w_dirA, w_dirB;
    wire [12:0] w_dutyA, w_dutyB;

    assign w_dirA = (sw_A_cw && !sw_A_ccw) ? 2'b10 :
                    (sw_A_ccw && !sw_A_cw) ? 2'b01 : 2'b00;
    assign w_dirB = (sw_B_cw && !sw_B_ccw) ? 2'b10 :
                    (sw_B_ccw && !sw_B_cw) ? 2'b01 : 2'b00;

    assign w_dutyA = (w_dirA != 2'b00) ? 13'd5000 : 13'd0;
    assign w_dutyB = (w_dirB != 2'b00) ? 13'd5000 : 13'd0;


    TB6612FNG motor_driver(
        .clk(clk),
        .reset(reset),

        .dirA_cmd(w_dirA),
        .dutyA(w_dutyA),
        .dirB_cmd(w_dirB),
        .dutyB(w_dutyB),
        
        .PWMA(PWMA_out),
        .AIN1(AIN1_out),
        .AIN2(AIN2_out),
        
        .PWMB(PWMB_out),
        .BIN1(BIN1_out),
        .BIN2(BIN2_out)

    );

endmodule


