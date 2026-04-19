`timescale 1ns / 1ps

module encoder
(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        encA_a,
    input  wire        encA_b,
    input  wire        encB_a,
    input  wire        encB_b,

    input  wire [23:0] vel_period,

    output reg signed [31:0] pos_a,
    output reg signed [31:0] pos_b,
    output reg signed [15:0] vel_a,
    output reg signed [15:0] vel_b
);

    // 2-stage 동기화
    reg [1:0] sA_a, sA_b, sB_a, sB_b;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sA_a <= 2'b0; sA_b <= 2'b0;
            sB_a <= 2'b0; sB_b <= 2'b0;
        end else begin
            sA_a <= {sA_a[0], encA_a};
            sA_b <= {sA_b[0], encA_b};
            sB_a <= {sB_a[0], encB_a};
            sB_b <= {sB_b[0], encB_b};
        end
    end

    wire a_a = sA_a[1], a_b = sA_b[1];
    wire b_a = sB_a[1], b_b = sB_b[1];

    reg prev_a_a, prev_a_b, prev_b_a, prev_b_b;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_a_a <= 1'b0; prev_a_b <= 1'b0;
            prev_b_a <= 1'b0; prev_b_b <= 1'b0;
        end else begin
            prev_a_a <= a_a; prev_a_b <= a_b;
            prev_b_a <= b_a; prev_b_b <= b_b;
        end
    end

    wire [1:0] stateA_now  = {a_a, a_b};
    wire [1:0] stateA_prev = {prev_a_a, prev_a_b};
    wire [1:0] stateB_now  = {b_a, b_b};
    wire [1:0] stateB_prev = {prev_b_a, prev_b_b};

    // ANSI C 스타일 함수 선언
    function signed [1:0] quad_decode(
        input [1:0] prev_s,
        input [1:0] curr_s
    );
        reg [3:0] trans;
        begin
            trans = {prev_s, curr_s};
            case (trans)
                4'b00_01, 4'b01_11, 4'b11_10, 4'b10_00: quad_decode = 2'sd1;
                4'b00_10, 4'b10_11, 4'b11_01, 4'b01_00: quad_decode = -2'sd1;
                default: quad_decode = 2'sd0;
            endcase
        end
    endfunction

    wire signed [1:0] deltaA = quad_decode(stateA_prev, stateA_now);
    wire signed [1:0] deltaB = quad_decode(stateB_prev, stateB_now);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos_a <= 32'sd0;
            pos_b <= 32'sd0;
        end else begin
            pos_a <= pos_a + deltaA;
            pos_b <= pos_b + deltaB;
        end
    end

    reg [23:0] vel_cnt;
    reg signed [31:0] pos_a_prev, pos_b_prev;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vel_cnt    <= 24'd0;
            vel_a      <= 16'sd0;
            vel_b      <= 16'sd0;
            pos_a_prev <= 32'sd0;
            pos_b_prev <= 32'sd0;
        end else begin
            if (vel_cnt >= vel_period - 1) begin
                vel_cnt    <= 24'd0;
                
                if ((pos_a - pos_a_prev) > 32'sd32767)
                    vel_a <= 16'sd32767;
                else if ((pos_a - pos_a_prev) < -32'sd32767)
                    vel_a <= -16'sd32767;
                else
                    vel_a <= (pos_a - pos_a_prev);

                if ((pos_b - pos_b_prev) > 32'sd32767)
                    vel_b <= 16'sd32767;
                else if ((pos_b - pos_b_prev) < -32'sd32767)
                    vel_b <= -16'sd32767;
                else
                    vel_b <= (pos_b - pos_b_prev);

                pos_a_prev <= pos_a;
                pos_b_prev <= pos_b;
            end else begin
                vel_cnt <= vel_cnt + 24'd1;
            end
        end
    end
endmodule