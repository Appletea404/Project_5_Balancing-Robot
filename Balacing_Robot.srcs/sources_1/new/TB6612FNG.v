`timescale 1ns / 1ps



module TB6612FNG(

    input clk,
    input reset,

    input [1:0] dirA_cmd,
    input [6:0] dutyA,

    input [1:0] dirB_cmd,
    input [6:0] dutyB,

    output reg PWMA,
    output reg AIN1,
    output reg AIN2,

    output reg PWMB,
    output reg BIN1,
    output reg BIN2
    );

    always @(*) begin
        case (dirA_cmd)
            2'b00: begin AIN1 = 1'b0; AIN2 = 1'b0; end      // Stop
            2'b10: begin AIN1 = 1'b1; AIN2 = 1'b0; end      // CW
            2'b01: begin AIN1 = 1'b0; AIN2 = 1'b1; end      // CCW
            2'b11: begin AIN1 = 1'b1; AIN2 = 1'b1; end      // Brake
            default: begin AIN1 = 1'b0; AIN2 = 1'b0; end
        endcase

        case (dirB_cmd)
            2'b00: begin BIN1 = 1'b0; BIN2 = 1'b0; end      // Stop
            2'b10: begin BIN1 = 1'b1; BIN2 = 1'b0; end      // CW
            2'b01: begin BIN1 = 1'b0; BIN2 = 1'b1; end      // CCW
            2'b11: begin BIN1 = 1'b1; BIN2 = 1'b1; end      // Brake
            default: begin BIN1 = 1'b0; BIN2 = 1'b0; end
        endcase
    end

    reg [15:0] pwm_counter;
    wire [12:0] dutyA_50;
    wire [12:0] dutyB_50;
    assign dutyA_50 = 50 * dutyA;
    assign dutyB_50 = 50 * dutyB;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pwm_counter <= 0;
            PWMA <= 0;
            PWMB <= 0;
        end else begin
            // 20kHz로 잡자 1틱 = 10ns
            // 100M / 20k = 5000
            if(pwm_counter >= 4999) pwm_counter <= 0;
            else pwm_counter <= pwm_counter + 1;

            if (pwm_counter < dutyA_50) PWMA <= 1'b1;
            else PWMA <= 1'b0;

            if (pwm_counter < dutyB_50) PWMB <= 1'b1;
            else PWMB <= 1'b0;

        end
        
    end

endmodule

