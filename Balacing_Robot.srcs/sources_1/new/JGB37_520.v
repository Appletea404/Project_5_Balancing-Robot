`timescale 1ns / 1ps

module JGB37_520(

    input clk,
    input reset,

    input sw_A_cw,
    input sw_A_ccw,
    input sw_B_cw,
    input sw_B_ccw,

    input duty_up,    // 누를 때마다 duty +10 (최대 100)
    input duty_down,  // 누를 때마다 duty -10 (최소 0)

    output PWMA_out,
    output PWMB_out,
    output AIN1_out,
    output AIN2_out,
    output BIN1_out,
    output BIN2_out
    );

    wire [1:0] w_dirA, w_dirB;
    wire [6:0] w_dutyA, w_dutyB;

    reg [1:0] sync_up, sync_down;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            sync_up   <= 2'b00;
            sync_down <= 2'b00;
        end else begin
            sync_up   <= {sync_up[0],   duty_up};
            sync_down <= {sync_down[0], duty_down};
        end
    end

    wire up_edge   = sync_up[0]   & ~sync_up[1];
    wire down_edge = sync_down[0] & ~sync_down[1];

    reg [6:0] duty;
    always @(posedge clk or posedge reset) begin
        if (reset)
            duty <= 7'd0;
        else if (up_edge)
            duty <= (duty >= 7'd90) ? 7'd100 : duty + 7'd10;
        else if (down_edge)
            duty <= (duty <= 7'd10) ? 7'd0   : duty - 7'd10;
    end

    assign w_dirA = (sw_A_cw && !sw_A_ccw) ? 2'b10 :
                    (sw_A_ccw && !sw_A_cw) ? 2'b01 : 2'b00;
    assign w_dirB = (sw_B_cw && !sw_B_ccw) ? 2'b10 :
                    (sw_B_ccw && !sw_B_cw) ? 2'b01 : 2'b00;

    assign w_dutyA = (w_dirA != 2'b00) ? duty : 7'd0;
    assign w_dutyB = (w_dirB != 2'b00) ? duty : 7'd0;


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


