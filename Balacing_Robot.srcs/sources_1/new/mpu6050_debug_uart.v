
`timescale 1ns / 1ps
//==============================================================================
// mpu6050_debug_uart
// - MPU6050 raw/corrected 6축 데이터 + roll/pitch를 UART로 출력
// - 출력 형식:
//   AX=XXXX AY=XXXX AZ=XXXX GX=XXXX GY=XXXX GZ=XXXX RL=XXXX PT=XXXX
//
// 설명:
// - 16bit 값을 16진수 4자리로 출력
// - roll / pitch는 degree가 아닌 기울기 비례값
//==============================================================================

module mpu6050_debug_uart
(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               init_done,
    input  wire               data_valid,

    input  wire signed [15:0] accel_x,
    input  wire signed [15:0] accel_y,
    input  wire signed [15:0] accel_z,
    input  wire signed [15:0] gyro_x,
    input  wire signed [15:0] gyro_y,
    input  wire signed [15:0] gyro_z,

    input  wire signed [15:0] roll,
    input  wire signed [15:0] pitch,

    output wire               uart_tx_o
);

    //--------------------------------------------------------------------------
    // 상태 정의
    //--------------------------------------------------------------------------
    localparam [7:0]
        ST_IDLE        = 8'd0,
        ST_CAPTURE     = 8'd1,
        ST_SPLIT       = 8'd2,

        ST_TX_A1       = 8'd3,   ST_W_A1       = 8'd4,
        ST_TX_X1       = 8'd5,   ST_W_X1       = 8'd6,
        ST_TX_EQ1      = 8'd7,   ST_W_EQ1      = 8'd8,
        ST_TX_AX3      = 8'd9,   ST_W_AX3      = 8'd10,
        ST_TX_AX2      = 8'd11,  ST_W_AX2      = 8'd12,
        ST_TX_AX1      = 8'd13,  ST_W_AX1      = 8'd14,
        ST_TX_AX0      = 8'd15,  ST_W_AX0      = 8'd16,
        ST_TX_SP1      = 8'd17,  ST_W_SP1      = 8'd18,

        ST_TX_A2       = 8'd19,  ST_W_A2       = 8'd20,
        ST_TX_Y1       = 8'd21,  ST_W_Y1       = 8'd22,
        ST_TX_EQ2      = 8'd23,  ST_W_EQ2      = 8'd24,
        ST_TX_AY3      = 8'd25,  ST_W_AY3      = 8'd26,
        ST_TX_AY2      = 8'd27,  ST_W_AY2      = 8'd28,
        ST_TX_AY1      = 8'd29,  ST_W_AY1      = 8'd30,
        ST_TX_AY0      = 8'd31,  ST_W_AY0      = 8'd32,
        ST_TX_SP2      = 8'd33,  ST_W_SP2      = 8'd34,

        ST_TX_A3       = 8'd35,  ST_W_A3       = 8'd36,
        ST_TX_Z1       = 8'd37,  ST_W_Z1       = 8'd38,
        ST_TX_EQ3      = 8'd39,  ST_W_EQ3      = 8'd40,
        ST_TX_AZ3      = 8'd41,  ST_W_AZ3      = 8'd42,
        ST_TX_AZ2      = 8'd43,  ST_W_AZ2      = 8'd44,
        ST_TX_AZ1      = 8'd45,  ST_W_AZ1      = 8'd46,
        ST_TX_AZ0      = 8'd47,  ST_W_AZ0      = 8'd48,
        ST_TX_SP3      = 8'd49,  ST_W_SP3      = 8'd50,

        ST_TX_G1       = 8'd51,  ST_W_G1       = 8'd52,
        ST_TX_X2       = 8'd53,  ST_W_X2       = 8'd54,
        ST_TX_EQ4      = 8'd55,  ST_W_EQ4      = 8'd56,
        ST_TX_GX3      = 8'd57,  ST_W_GX3      = 8'd58,
        ST_TX_GX2      = 8'd59,  ST_W_GX2      = 8'd60,
        ST_TX_GX1      = 8'd61,  ST_W_GX1      = 8'd62,
        ST_TX_GX0      = 8'd63,  ST_W_GX0      = 8'd64,
        ST_TX_SP4      = 8'd65,  ST_W_SP4      = 8'd66,

        ST_TX_G2       = 8'd67,  ST_W_G2       = 8'd68,
        ST_TX_Y2       = 8'd69,  ST_W_Y2       = 8'd70,
        ST_TX_EQ5      = 8'd71,  ST_W_EQ5      = 8'd72,
        ST_TX_GY3      = 8'd73,  ST_W_GY3      = 8'd74,
        ST_TX_GY2      = 8'd75,  ST_W_GY2      = 8'd76,
        ST_TX_GY1      = 8'd77,  ST_W_GY1      = 8'd78,
        ST_TX_GY0      = 8'd79,  ST_W_GY0      = 8'd80,
        ST_TX_SP5      = 8'd81,  ST_W_SP5      = 8'd82,

        ST_TX_G3       = 8'd83,  ST_W_G3       = 8'd84,
        ST_TX_Z2       = 8'd85,  ST_W_Z2       = 8'd86,
        ST_TX_EQ6      = 8'd87,  ST_W_EQ6      = 8'd88,
        ST_TX_GZ3      = 8'd89,  ST_W_GZ3      = 8'd90,
        ST_TX_GZ2      = 8'd91,  ST_W_GZ2      = 8'd92,
        ST_TX_GZ1      = 8'd93,  ST_W_GZ1      = 8'd94,
        ST_TX_GZ0      = 8'd95,  ST_W_GZ0      = 8'd96,
        ST_TX_SP6      = 8'd97,  ST_W_SP6      = 8'd98,

        ST_TX_R1       = 8'd99,  ST_W_R1       = 8'd100,
        ST_TX_L1       = 8'd101, ST_W_L1       = 8'd102,
        ST_TX_EQ7      = 8'd103, ST_W_EQ7      = 8'd104,
        ST_TX_RL3      = 8'd105, ST_W_RL3      = 8'd106,
        ST_TX_RL2      = 8'd107, ST_W_RL2      = 8'd108,
        ST_TX_RL1      = 8'd109, ST_W_RL1      = 8'd110,
        ST_TX_RL0      = 8'd111, ST_W_RL0      = 8'd112,
        ST_TX_SP7      = 8'd113, ST_W_SP7      = 8'd114,

        ST_TX_P1       = 8'd115, ST_W_P1       = 8'd116,
        ST_TX_T1       = 8'd117, ST_W_T1       = 8'd118,
        ST_TX_EQ8      = 8'd119, ST_W_EQ8      = 8'd120,
        ST_TX_PT3      = 8'd121, ST_W_PT3      = 8'd122,
        ST_TX_PT2      = 8'd123, ST_W_PT2      = 8'd124,
        ST_TX_PT1      = 8'd125, ST_W_PT1      = 8'd126,
        ST_TX_PT0      = 8'd127, ST_W_PT0      = 8'd128,

        ST_TX_CR       = 8'd129, ST_W_CR       = 8'd130,
        ST_TX_LF       = 8'd131, ST_W_LF       = 8'd132;

    reg [7:0] state;

    //--------------------------------------------------------------------------
    // UART 송신기 연결
    //--------------------------------------------------------------------------
    reg        tx_start;
    reg [7:0]  tx_data;
    wire       tx_busy;
    wire       tx_done;

    uart_tx u_uart_tx
    (
        .clk      (clk),
        .rst_n    (rst_n),
        .tx_start (tx_start),
        .tx_data  (tx_data),
        .tx       (uart_tx_o),
        .busy     (tx_busy),
        .done     (tx_done)
    );

    //--------------------------------------------------------------------------
    // 출력 주기 생성
    //--------------------------------------------------------------------------
    localparam [31:0] PRINT_CNT_MAX = 32'd9_999_999;

    reg [31:0] print_cnt;
    reg        print_tick;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            print_cnt  <= 32'd0;
            print_tick <= 1'b0;
        end
        else begin
            print_tick <= 1'b0;

            if (print_cnt == PRINT_CNT_MAX) begin
                print_cnt  <= 32'd0;
                print_tick <= 1'b1;
            end
            else begin
                print_cnt <= print_cnt + 32'd1;
            end
        end
    end

    //--------------------------------------------------------------------------
    // data_valid를 출력 시점까지 보관
    //--------------------------------------------------------------------------
    reg sample_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sample_ready <= 1'b0;
        end
        else begin
            if (!init_done)
                sample_ready <= 1'b0;
            else begin
                if (data_valid)
                    sample_ready <= 1'b1;
                else if (state == ST_CAPTURE)
                    sample_ready <= 1'b0;
            end
        end
    end

    //--------------------------------------------------------------------------
    // 출력 중 값 고정
    //--------------------------------------------------------------------------
    reg [15:0] ax_r, ay_r, az_r;
    reg [15:0] gx_r, gy_r, gz_r;
    reg [15:0] rl_r, pt_r;

    //--------------------------------------------------------------------------
    // 각 항목 nibble 분리
    //--------------------------------------------------------------------------
    reg [3:0] ax3, ax2, ax1, ax0;
    reg [3:0] ay3, ay2, ay1, ay0;
    reg [3:0] az3, az2, az1, az0;
    reg [3:0] gx3, gx2, gx1, gx0;
    reg [3:0] gy3, gy2, gy1, gy0;
    reg [3:0] gz3, gz2, gz1, gz0;
    reg [3:0] rl3, rl2, rl1, rl0;
    reg [3:0] pt3, pt2, pt1, pt0;

    //--------------------------------------------------------------------------
    // 4bit hex -> ASCII
    //--------------------------------------------------------------------------
    function [7:0] hex_to_ascii;
        input [3:0] nib;
        begin
            case (nib)
                4'h0: hex_to_ascii = "0";
                4'h1: hex_to_ascii = "1";
                4'h2: hex_to_ascii = "2";
                4'h3: hex_to_ascii = "3";
                4'h4: hex_to_ascii = "4";
                4'h5: hex_to_ascii = "5";
                4'h6: hex_to_ascii = "6";
                4'h7: hex_to_ascii = "7";
                4'h8: hex_to_ascii = "8";
                4'h9: hex_to_ascii = "9";
                4'hA: hex_to_ascii = "A";
                4'hB: hex_to_ascii = "B";
                4'hC: hex_to_ascii = "C";
                4'hD: hex_to_ascii = "D";
                4'hE: hex_to_ascii = "E";
                default: hex_to_ascii = "F";
            endcase
        end
    endfunction

    //--------------------------------------------------------------------------
    // UART 출력 FSM
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_start <= 1'b0;
            tx_data  <= 8'd0;

            ax_r <= 16'd0; ay_r <= 16'd0; az_r <= 16'd0;
            gx_r <= 16'd0; gy_r <= 16'd0; gz_r <= 16'd0;
            rl_r <= 16'd0; pt_r <= 16'd0;

            ax3 <= 4'd0; ax2 <= 4'd0; ax1 <= 4'd0; ax0 <= 4'd0;
            ay3 <= 4'd0; ay2 <= 4'd0; ay1 <= 4'd0; ay0 <= 4'd0;
            az3 <= 4'd0; az2 <= 4'd0; az1 <= 4'd0; az0 <= 4'd0;
            gx3 <= 4'd0; gx2 <= 4'd0; gx1 <= 4'd0; gx0 <= 4'd0;
            gy3 <= 4'd0; gy2 <= 4'd0; gy1 <= 4'd0; gy0 <= 4'd0;
            gz3 <= 4'd0; gz2 <= 4'd0; gz1 <= 4'd0; gz0 <= 4'd0;
            rl3 <= 4'd0; rl2 <= 4'd0; rl1 <= 4'd0; rl0 <= 4'd0;
            pt3 <= 4'd0; pt2 <= 4'd0; pt1 <= 4'd0; pt0 <= 4'd0;

            state <= ST_IDLE;
        end
        else begin
            tx_start <= 1'b0;

            case (state)
                ST_IDLE: begin
                    if (init_done && sample_ready && print_tick)
                        state <= ST_CAPTURE;
                end

                ST_CAPTURE: begin
                    ax_r <= accel_x[15:0];
                    ay_r <= accel_y[15:0];
                    az_r <= accel_z[15:0];
                    gx_r <= gyro_x[15:0];
                    gy_r <= gyro_y[15:0];
                    gz_r <= gyro_z[15:0];
                    rl_r <= roll[15:0];
                    pt_r <= pitch[15:0];
                    state <= ST_SPLIT;
                end

                ST_SPLIT: begin
                    ax3 <= ax_r[15:12]; ax2 <= ax_r[11:8]; ax1 <= ax_r[7:4]; ax0 <= ax_r[3:0];
                    ay3 <= ay_r[15:12]; ay2 <= ay_r[11:8]; ay1 <= ay_r[7:4]; ay0 <= ay_r[3:0];
                    az3 <= az_r[15:12]; az2 <= az_r[11:8]; az1 <= az_r[7:4]; az0 <= az_r[3:0];
                    gx3 <= gx_r[15:12]; gx2 <= gx_r[11:8]; gx1 <= gx_r[7:4]; gx0 <= gx_r[3:0];
                    gy3 <= gy_r[15:12]; gy2 <= gy_r[11:8]; gy1 <= gy_r[7:4]; gy0 <= gy_r[3:0];
                    gz3 <= gz_r[15:12]; gz2 <= gz_r[11:8]; gz1 <= gz_r[7:4]; gz0 <= gz_r[3:0];
                    rl3 <= rl_r[15:12]; rl2 <= rl_r[11:8]; rl1 <= rl_r[7:4]; rl0 <= rl_r[3:0];
                    pt3 <= pt_r[15:12]; pt2 <= pt_r[11:8]; pt1 <= pt_r[7:4]; pt0 <= pt_r[3:0];
                    state <= ST_TX_A1;
                end

                ST_TX_A1:  begin if (!tx_busy) begin tx_data <= "A"; tx_start <= 1'b1; state <= ST_W_A1; end end
                ST_W_A1:   begin if (tx_done) state <= ST_TX_X1; end
                ST_TX_X1:  begin if (!tx_busy) begin tx_data <= "X"; tx_start <= 1'b1; state <= ST_W_X1; end end
                ST_W_X1:   begin if (tx_done) state <= ST_TX_EQ1; end
                ST_TX_EQ1: begin if (!tx_busy) begin tx_data <= "="; tx_start <= 1'b1; state <= ST_W_EQ1; end end
                ST_W_EQ1:  begin if (tx_done) state <= ST_TX_AX3; end
                ST_TX_AX3: begin if (!tx_busy) begin tx_data <= hex_to_ascii(ax3); tx_start <= 1'b1; state <= ST_W_AX3; end end
                ST_W_AX3:  begin if (tx_done) state <= ST_TX_AX2; end
                ST_TX_AX2: begin if (!tx_busy) begin tx_data <= hex_to_ascii(ax2); tx_start <= 1'b1; state <= ST_W_AX2; end end
                ST_W_AX2:  begin if (tx_done) state <= ST_TX_AX1; end
                ST_TX_AX1: begin if (!tx_busy) begin tx_data <= hex_to_ascii(ax1); tx_start <= 1'b1; state <= ST_W_AX1; end end
                ST_W_AX1:  begin if (tx_done) state <= ST_TX_AX0; end
                ST_TX_AX0: begin if (!tx_busy) begin tx_data <= hex_to_ascii(ax0); tx_start <= 1'b1; state <= ST_W_AX0; end end
                ST_W_AX0:  begin if (tx_done) state <= ST_TX_SP1; end
                ST_TX_SP1: begin if (!tx_busy) begin tx_data <= " "; tx_start <= 1'b1; state <= ST_W_SP1; end end
                ST_W_SP1:  begin if (tx_done) state <= ST_TX_A2; end

                ST_TX_A2:  begin if (!tx_busy) begin tx_data <= "A"; tx_start <= 1'b1; state <= ST_W_A2; end end
                ST_W_A2:   begin if (tx_done) state <= ST_TX_Y1; end
                ST_TX_Y1:  begin if (!tx_busy) begin tx_data <= "Y"; tx_start <= 1'b1; state <= ST_W_Y1; end end
                ST_W_Y1:   begin if (tx_done) state <= ST_TX_EQ2; end
                ST_TX_EQ2: begin if (!tx_busy) begin tx_data <= "="; tx_start <= 1'b1; state <= ST_W_EQ2; end end
                ST_W_EQ2:  begin if (tx_done) state <= ST_TX_AY3; end
                ST_TX_AY3: begin if (!tx_busy) begin tx_data <= hex_to_ascii(ay3); tx_start <= 1'b1; state <= ST_W_AY3; end end
                ST_W_AY3:  begin if (tx_done) state <= ST_TX_AY2; end
                ST_TX_AY2: begin if (!tx_busy) begin tx_data <= hex_to_ascii(ay2); tx_start <= 1'b1; state <= ST_W_AY2; end end
                ST_W_AY2:  begin if (tx_done) state <= ST_TX_AY1; end
                ST_TX_AY1: begin if (!tx_busy) begin tx_data <= hex_to_ascii(ay1); tx_start <= 1'b1; state <= ST_W_AY1; end end
                ST_W_AY1:  begin if (tx_done) state <= ST_TX_AY0; end
                ST_TX_AY0: begin if (!tx_busy) begin tx_data <= hex_to_ascii(ay0); tx_start <= 1'b1; state <= ST_W_AY0; end end
                ST_W_AY0:  begin if (tx_done) state <= ST_TX_SP2; end
                ST_TX_SP2: begin if (!tx_busy) begin tx_data <= " "; tx_start <= 1'b1; state <= ST_W_SP2; end end
                ST_W_SP2:  begin if (tx_done) state <= ST_TX_A3; end

                ST_TX_A3:  begin if (!tx_busy) begin tx_data <= "A"; tx_start <= 1'b1; state <= ST_W_A3; end end
                ST_W_A3:   begin if (tx_done) state <= ST_TX_Z1; end
                ST_TX_Z1:  begin if (!tx_busy) begin tx_data <= "Z"; tx_start <= 1'b1; state <= ST_W_Z1; end end
                ST_W_Z1:   begin if (tx_done) state <= ST_TX_EQ3; end
                ST_TX_EQ3: begin if (!tx_busy) begin tx_data <= "="; tx_start <= 1'b1; state <= ST_W_EQ3; end end
                ST_W_EQ3:  begin if (tx_done) state <= ST_TX_AZ3; end
                ST_TX_AZ3: begin if (!tx_busy) begin tx_data <= hex_to_ascii(az3); tx_start <= 1'b1; state <= ST_W_AZ3; end end
                ST_W_AZ3:  begin if (tx_done) state <= ST_TX_AZ2; end
                ST_TX_AZ2: begin if (!tx_busy) begin tx_data <= hex_to_ascii(az2); tx_start <= 1'b1; state <= ST_W_AZ2; end end
                ST_W_AZ2:  begin if (tx_done) state <= ST_TX_AZ1; end
                ST_TX_AZ1: begin if (!tx_busy) begin tx_data <= hex_to_ascii(az1); tx_start <= 1'b1; state <= ST_W_AZ1; end end
                ST_W_AZ1:  begin if (tx_done) state <= ST_TX_AZ0; end
                ST_TX_AZ0: begin if (!tx_busy) begin tx_data <= hex_to_ascii(az0); tx_start <= 1'b1; state <= ST_W_AZ0; end end
                ST_W_AZ0:  begin if (tx_done) state <= ST_TX_SP3; end
                ST_TX_SP3: begin if (!tx_busy) begin tx_data <= " "; tx_start <= 1'b1; state <= ST_W_SP3; end end
                ST_W_SP3:  begin if (tx_done) state <= ST_TX_G1; end

                ST_TX_G1:  begin if (!tx_busy) begin tx_data <= "G"; tx_start <= 1'b1; state <= ST_W_G1; end end
                ST_W_G1:   begin if (tx_done) state <= ST_TX_X2; end
                ST_TX_X2:  begin if (!tx_busy) begin tx_data <= "X"; tx_start <= 1'b1; state <= ST_W_X2; end end
                ST_W_X2:   begin if (tx_done) state <= ST_TX_EQ4; end
                ST_TX_EQ4: begin if (!tx_busy) begin tx_data <= "="; tx_start <= 1'b1; state <= ST_W_EQ4; end end
                ST_W_EQ4:  begin if (tx_done) state <= ST_TX_GX3; end
                ST_TX_GX3: begin if (!tx_busy) begin tx_data <= hex_to_ascii(gx3); tx_start <= 1'b1; state <= ST_W_GX3; end end
                ST_W_GX3:  begin if (tx_done) state <= ST_TX_GX2; end
                ST_TX_GX2: begin if (!tx_busy) begin tx_data <= hex_to_ascii(gx2); tx_start <= 1'b1; state <= ST_W_GX2; end end
                ST_W_GX2:  begin if (tx_done) state <= ST_TX_GX1; end
                ST_TX_GX1: begin if (!tx_busy) begin tx_data <= hex_to_ascii(gx1); tx_start <= 1'b1; state <= ST_W_GX1; end end
                ST_W_GX1:  begin if (tx_done) state <= ST_TX_GX0; end
                ST_TX_GX0: begin if (!tx_busy) begin tx_data <= hex_to_ascii(gx0); tx_start <= 1'b1; state <= ST_W_GX0; end end
                ST_W_GX0:  begin if (tx_done) state <= ST_TX_SP4; end
                ST_TX_SP4: begin if (!tx_busy) begin tx_data <= " "; tx_start <= 1'b1; state <= ST_W_SP4; end end
                ST_W_SP4:  begin if (tx_done) state <= ST_TX_G2; end

                ST_TX_G2:  begin if (!tx_busy) begin tx_data <= "G"; tx_start <= 1'b1; state <= ST_W_G2; end end
                ST_W_G2:   begin if (tx_done) state <= ST_TX_Y2; end
                ST_TX_Y2:  begin if (!tx_busy) begin tx_data <= "Y"; tx_start <= 1'b1; state <= ST_W_Y2; end end
                ST_W_Y2:   begin if (tx_done) state <= ST_TX_EQ5; end
                ST_TX_EQ5: begin if (!tx_busy) begin tx_data <= "="; tx_start <= 1'b1; state <= ST_W_EQ5; end end
                ST_W_EQ5:  begin if (tx_done) state <= ST_TX_GY3; end
                ST_TX_GY3: begin if (!tx_busy) begin tx_data <= hex_to_ascii(gy3); tx_start <= 1'b1; state <= ST_W_GY3; end end
                ST_W_GY3:  begin if (tx_done) state <= ST_TX_GY2; end
                ST_TX_GY2: begin if (!tx_busy) begin tx_data <= hex_to_ascii(gy2); tx_start <= 1'b1; state <= ST_W_GY2; end end
                ST_W_GY2:  begin if (tx_done) state <= ST_TX_GY1; end
                ST_TX_GY1: begin if (!tx_busy) begin tx_data <= hex_to_ascii(gy1); tx_start <= 1'b1; state <= ST_W_GY1; end end
                ST_W_GY1:  begin if (tx_done) state <= ST_TX_GY0; end
                ST_TX_GY0: begin if (!tx_busy) begin tx_data <= hex_to_ascii(gy0); tx_start <= 1'b1; state <= ST_W_GY0; end end
                ST_W_GY0:  begin if (tx_done) state <= ST_TX_SP5; end
                ST_TX_SP5: begin if (!tx_busy) begin tx_data <= " "; tx_start <= 1'b1; state <= ST_W_SP5; end end
                ST_W_SP5:  begin if (tx_done) state <= ST_TX_G3; end

                ST_TX_G3:  begin if (!tx_busy) begin tx_data <= "G"; tx_start <= 1'b1; state <= ST_W_G3; end end
                ST_W_G3:   begin if (tx_done) state <= ST_TX_Z2; end
                ST_TX_Z2:  begin if (!tx_busy) begin tx_data <= "Z"; tx_start <= 1'b1; state <= ST_W_Z2; end end
                ST_W_Z2:   begin if (tx_done) state <= ST_TX_EQ6; end
                ST_TX_EQ6: begin if (!tx_busy) begin tx_data <= "="; tx_start <= 1'b1; state <= ST_W_EQ6; end end
                ST_W_EQ6:  begin if (tx_done) state <= ST_TX_GZ3; end
                ST_TX_GZ3: begin if (!tx_busy) begin tx_data <= hex_to_ascii(gz3); tx_start <= 1'b1; state <= ST_W_GZ3; end end
                ST_W_GZ3:  begin if (tx_done) state <= ST_TX_GZ2; end
                ST_TX_GZ2: begin if (!tx_busy) begin tx_data <= hex_to_ascii(gz2); tx_start <= 1'b1; state <= ST_W_GZ2; end end
                ST_W_GZ2:  begin if (tx_done) state <= ST_TX_GZ1; end
                ST_TX_GZ1: begin if (!tx_busy) begin tx_data <= hex_to_ascii(gz1); tx_start <= 1'b1; state <= ST_W_GZ1; end end
                ST_W_GZ1:  begin if (tx_done) state <= ST_TX_GZ0; end
                ST_TX_GZ0: begin if (!tx_busy) begin tx_data <= hex_to_ascii(gz0); tx_start <= 1'b1; state <= ST_W_GZ0; end end
                ST_W_GZ0:  begin if (tx_done) state <= ST_TX_SP6; end
                ST_TX_SP6: begin if (!tx_busy) begin tx_data <= " "; tx_start <= 1'b1; state <= ST_W_SP6; end end
                ST_W_SP6:  begin if (tx_done) state <= ST_TX_R1; end

                ST_TX_R1:  begin if (!tx_busy) begin tx_data <= "R"; tx_start <= 1'b1; state <= ST_W_R1; end end
                ST_W_R1:   begin if (tx_done) state <= ST_TX_L1; end
                ST_TX_L1:  begin if (!tx_busy) begin tx_data <= "L"; tx_start <= 1'b1; state <= ST_W_L1; end end
                ST_W_L1:   begin if (tx_done) state <= ST_TX_EQ7; end
                ST_TX_EQ7: begin if (!tx_busy) begin tx_data <= "="; tx_start <= 1'b1; state <= ST_W_EQ7; end end
                ST_W_EQ7:  begin if (tx_done) state <= ST_TX_RL3; end
                ST_TX_RL3: begin if (!tx_busy) begin tx_data <= hex_to_ascii(rl3); tx_start <= 1'b1; state <= ST_W_RL3; end end
                ST_W_RL3:  begin if (tx_done) state <= ST_TX_RL2; end
                ST_TX_RL2: begin if (!tx_busy) begin tx_data <= hex_to_ascii(rl2); tx_start <= 1'b1; state <= ST_W_RL2; end end
                ST_W_RL2:  begin if (tx_done) state <= ST_TX_RL1; end
                ST_TX_RL1: begin if (!tx_busy) begin tx_data <= hex_to_ascii(rl1); tx_start <= 1'b1; state <= ST_W_RL1; end end
                ST_W_RL1:  begin if (tx_done) state <= ST_TX_RL0; end
                ST_TX_RL0: begin if (!tx_busy) begin tx_data <= hex_to_ascii(rl0); tx_start <= 1'b1; state <= ST_W_RL0; end end
                ST_W_RL0:  begin if (tx_done) state <= ST_TX_SP7; end
                ST_TX_SP7: begin if (!tx_busy) begin tx_data <= " "; tx_start <= 1'b1; state <= ST_W_SP7; end end
                ST_W_SP7:  begin if (tx_done) state <= ST_TX_P1; end

                ST_TX_P1:  begin if (!tx_busy) begin tx_data <= "P"; tx_start <= 1'b1; state <= ST_W_P1; end end
                ST_W_P1:   begin if (tx_done) state <= ST_TX_T1; end
                ST_TX_T1:  begin if (!tx_busy) begin tx_data <= "T"; tx_start <= 1'b1; state <= ST_W_T1; end end
                ST_W_T1:   begin if (tx_done) state <= ST_TX_EQ8; end
                ST_TX_EQ8: begin if (!tx_busy) begin tx_data <= "="; tx_start <= 1'b1; state <= ST_W_EQ8; end end
                ST_W_EQ8:  begin if (tx_done) state <= ST_TX_PT3; end
                ST_TX_PT3: begin if (!tx_busy) begin tx_data <= hex_to_ascii(pt3); tx_start <= 1'b1; state <= ST_W_PT3; end end
                ST_W_PT3:  begin if (tx_done) state <= ST_TX_PT2; end
                ST_TX_PT2: begin if (!tx_busy) begin tx_data <= hex_to_ascii(pt2); tx_start <= 1'b1; state <= ST_W_PT2; end end
                ST_W_PT2:  begin if (tx_done) state <= ST_TX_PT1; end
                ST_TX_PT1: begin if (!tx_busy) begin tx_data <= hex_to_ascii(pt1); tx_start <= 1'b1; state <= ST_W_PT1; end end
                ST_W_PT1:  begin if (tx_done) state <= ST_TX_PT0; end
                ST_TX_PT0: begin if (!tx_busy) begin tx_data <= hex_to_ascii(pt0); tx_start <= 1'b1; state <= ST_W_PT0; end end
                ST_W_PT0:  begin if (tx_done) state <= ST_TX_CR; end

                ST_TX_CR:  begin if (!tx_busy) begin tx_data <= 8'h0D; tx_start <= 1'b1; state <= ST_W_CR; end end
                ST_W_CR:   begin if (tx_done) state <= ST_TX_LF; end
                ST_TX_LF:  begin if (!tx_busy) begin tx_data <= 8'h0A; tx_start <= 1'b1; state <= ST_W_LF; end end
                ST_W_LF:   begin if (tx_done) state <= ST_IDLE; end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule


// `timescale 1ns / 1ps
// //==============================================================================
// // mpu6050_debug_uart
// // - MPU6050 raw 6축 데이터를 UART로 출력
// // - 출력 형식:
// //   AX=XXXX AY=XXXX AZ=XXXX GX=XXXX GY=XXXX GZ=XXXX
// //
// // 설명:
// // - accel/gyro 16bit raw 값을 16진수 4자리로 출력
// // - init_done 이후 data_valid가 들어오면 최신 샘플 준비로 보고
// //   print_tick 주기마다 한 줄씩 출력
// //
// // 주의:
// // - angle 계산은 제외한 raw 데이터 검증용
// //==============================================================================

// module mpu6050_debug_uart
// (
//     input  wire               clk,
//     input  wire               rst_n,
//     input  wire               init_done,
//     input  wire               data_valid,

//     input  wire signed [15:0] accel_x,
//     input  wire signed [15:0] accel_y,
//     input  wire signed [15:0] accel_z,
//     input  wire signed [15:0] gyro_x,
//     input  wire signed [15:0] gyro_y,
//     input  wire signed [15:0] gyro_z,

//     output wire               uart_tx_o
// );

//     //--------------------------------------------------------------------------
//     // 상태 정의
//     //--------------------------------------------------------------------------
//     localparam [7:0]
//         ST_IDLE        = 8'd0,
//         ST_CAPTURE     = 8'd1,
//         ST_SPLIT       = 8'd2,

//         ST_TX_A1       = 8'd3,   ST_W_A1       = 8'd4,
//         ST_TX_X1       = 8'd5,   ST_W_X1       = 8'd6,
//         ST_TX_EQ1      = 8'd7,   ST_W_EQ1      = 8'd8,
//         ST_TX_AX3      = 8'd9,   ST_W_AX3      = 8'd10,
//         ST_TX_AX2      = 8'd11,  ST_W_AX2      = 8'd12,
//         ST_TX_AX1      = 8'd13,  ST_W_AX1      = 8'd14,
//         ST_TX_AX0      = 8'd15,  ST_W_AX0      = 8'd16,
//         ST_TX_SP1      = 8'd17,  ST_W_SP1      = 8'd18,

//         ST_TX_A2       = 8'd19,  ST_W_A2       = 8'd20,
//         ST_TX_Y1       = 8'd21,  ST_W_Y1       = 8'd22,
//         ST_TX_EQ2      = 8'd23,  ST_W_EQ2      = 8'd24,
//         ST_TX_AY3      = 8'd25,  ST_W_AY3      = 8'd26,
//         ST_TX_AY2      = 8'd27,  ST_W_AY2      = 8'd28,
//         ST_TX_AY1      = 8'd29,  ST_W_AY1      = 8'd30,
//         ST_TX_AY0      = 8'd31,  ST_W_AY0      = 8'd32,
//         ST_TX_SP2      = 8'd33,  ST_W_SP2      = 8'd34,

//         ST_TX_A3       = 8'd35,  ST_W_A3       = 8'd36,
//         ST_TX_Z1       = 8'd37,  ST_W_Z1       = 8'd38,
//         ST_TX_EQ3      = 8'd39,  ST_W_EQ3      = 8'd40,
//         ST_TX_AZ3      = 8'd41,  ST_W_AZ3      = 8'd42,
//         ST_TX_AZ2      = 8'd43,  ST_W_AZ2      = 8'd44,
//         ST_TX_AZ1      = 8'd45,  ST_W_AZ1      = 8'd46,
//         ST_TX_AZ0      = 8'd47,  ST_W_AZ0      = 8'd48,
//         ST_TX_SP3      = 8'd49,  ST_W_SP3      = 8'd50,

//         ST_TX_G1       = 8'd51,  ST_W_G1       = 8'd52,
//         ST_TX_X2       = 8'd53,  ST_W_X2       = 8'd54,
//         ST_TX_EQ4      = 8'd55,  ST_W_EQ4      = 8'd56,
//         ST_TX_GX3      = 8'd57,  ST_W_GX3      = 8'd58,
//         ST_TX_GX2      = 8'd59,  ST_W_GX2      = 8'd60,
//         ST_TX_GX1      = 8'd61,  ST_W_GX1      = 8'd62,
//         ST_TX_GX0      = 8'd63,  ST_W_GX0      = 8'd64,
//         ST_TX_SP4      = 8'd65,  ST_W_SP4      = 8'd66,

//         ST_TX_G2       = 8'd67,  ST_W_G2       = 8'd68,
//         ST_TX_Y2       = 8'd69,  ST_W_Y2       = 8'd70,
//         ST_TX_EQ5      = 8'd71,  ST_W_EQ5      = 8'd72,
//         ST_TX_GY3      = 8'd73,  ST_W_GY3      = 8'd74,
//         ST_TX_GY2      = 8'd75,  ST_W_GY2      = 8'd76,
//         ST_TX_GY1      = 8'd77,  ST_W_GY1      = 8'd78,
//         ST_TX_GY0      = 8'd79,  ST_W_GY0      = 8'd80,
//         ST_TX_SP5      = 8'd81,  ST_W_SP5      = 8'd82,

//         ST_TX_G3       = 8'd83,  ST_W_G3       = 8'd84,
//         ST_TX_Z2       = 8'd85,  ST_W_Z2       = 8'd86,
//         ST_TX_EQ6      = 8'd87,  ST_W_EQ6      = 8'd88,
//         ST_TX_GZ3      = 8'd89,  ST_W_GZ3      = 8'd90,
//         ST_TX_GZ2      = 8'd91,  ST_W_GZ2      = 8'd92,
//         ST_TX_GZ1      = 8'd93,  ST_W_GZ1      = 8'd94,
//         ST_TX_GZ0      = 8'd95,  ST_W_GZ0      = 8'd96,

//         ST_TX_CR       = 8'd97,  ST_W_CR       = 8'd98,
//         ST_TX_LF       = 8'd99,  ST_W_LF       = 8'd100;

//     reg [7:0] state;

//     //--------------------------------------------------------------------------
//     // UART 송신기 연결
//     //--------------------------------------------------------------------------
//     reg        tx_start;
//     reg [7:0]  tx_data;
//     wire       tx_busy;
//     wire       tx_done;

//     uart_tx u_uart_tx
//     (
//         .clk      (clk),
//         .rst_n    (rst_n),
//         .tx_start (tx_start),
//         .tx_data  (tx_data),
//         .tx       (uart_tx_o),
//         .busy     (tx_busy),
//         .done     (tx_done)
//     );

//     //--------------------------------------------------------------------------
//     // 출력 주기 생성
//     // 100MHz 기준 약 10Hz
//     //--------------------------------------------------------------------------
//     localparam [31:0] PRINT_CNT_MAX = 32'd9_999_999;

//     reg [31:0] print_cnt;
//     reg        print_tick;

//     always @(posedge clk or negedge rst_n) begin
//         if (!rst_n) begin
//             print_cnt  <= 32'd0;
//             print_tick <= 1'b0;
//         end
//         else begin
//             print_tick <= 1'b0;

//             if (print_cnt == PRINT_CNT_MAX) begin
//                 print_cnt  <= 32'd0;
//                 print_tick <= 1'b1;
//             end
//             else begin
//                 print_cnt <= print_cnt + 32'd1;
//             end
//         end
//     end

//     //--------------------------------------------------------------------------
//     // data_valid를 출력 시점까지 보관
//     //--------------------------------------------------------------------------
//     reg sample_ready;

//     always @(posedge clk or negedge rst_n) begin
//         if (!rst_n) begin
//             sample_ready <= 1'b0;
//         end
//         else begin
//             if (!init_done)
//                 sample_ready <= 1'b0;
//             else begin
//                 if (data_valid)
//                     sample_ready <= 1'b1;
//                 else if (state == ST_CAPTURE)
//                     sample_ready <= 1'b0;
//             end
//         end
//     end

//     //--------------------------------------------------------------------------
//     // 출력 중 값 고정
//     //--------------------------------------------------------------------------
//     reg [15:0] ax_r, ay_r, az_r;
//     reg [15:0] gx_r, gy_r, gz_r;

//     //--------------------------------------------------------------------------
//     // 각 축별 16진수 nibble 분리
//     //--------------------------------------------------------------------------
//     reg [3:0] ax3, ax2, ax1, ax0;
//     reg [3:0] ay3, ay2, ay1, ay0;
//     reg [3:0] az3, az2, az1, az0;
//     reg [3:0] gx3, gx2, gx1, gx0;
//     reg [3:0] gy3, gy2, gy1, gy0;
//     reg [3:0] gz3, gz2, gz1, gz0;

//     //--------------------------------------------------------------------------
//     // 4bit hex 값을 ASCII 문자로 변환
//     //--------------------------------------------------------------------------
//     function [7:0] hex_to_ascii;
//         input [3:0] nib;
//         begin
//             case (nib)
//                 4'h0: hex_to_ascii = "0";
//                 4'h1: hex_to_ascii = "1";
//                 4'h2: hex_to_ascii = "2";
//                 4'h3: hex_to_ascii = "3";
//                 4'h4: hex_to_ascii = "4";
//                 4'h5: hex_to_ascii = "5";
//                 4'h6: hex_to_ascii = "6";
//                 4'h7: hex_to_ascii = "7";
//                 4'h8: hex_to_ascii = "8";
//                 4'h9: hex_to_ascii = "9";
//                 4'hA: hex_to_ascii = "A";
//                 4'hB: hex_to_ascii = "B";
//                 4'hC: hex_to_ascii = "C";
//                 4'hD: hex_to_ascii = "D";
//                 4'hE: hex_to_ascii = "E";
//                 default: hex_to_ascii = "F";
//             endcase
//         end
//     endfunction

//     //--------------------------------------------------------------------------
//     // UART 출력 FSM
//     //--------------------------------------------------------------------------
//     always @(posedge clk or negedge rst_n) begin
//         if (!rst_n) begin
//             tx_start <= 1'b0;
//             tx_data  <= 8'd0;

//             ax_r <= 16'd0; ay_r <= 16'd0; az_r <= 16'd0;
//             gx_r <= 16'd0; gy_r <= 16'd0; gz_r <= 16'd0;

//             ax3 <= 4'd0; ax2 <= 4'd0; ax1 <= 4'd0; ax0 <= 4'd0;
//             ay3 <= 4'd0; ay2 <= 4'd0; ay1 <= 4'd0; ay0 <= 4'd0;
//             az3 <= 4'd0; az2 <= 4'd0; az1 <= 4'd0; az0 <= 4'd0;
//             gx3 <= 4'd0; gx2 <= 4'd0; gx1 <= 4'd0; gx0 <= 4'd0;
//             gy3 <= 4'd0; gy2 <= 4'd0; gy1 <= 4'd0; gy0 <= 4'd0;
//             gz3 <= 4'd0; gz2 <= 4'd0; gz1 <= 4'd0; gz0 <= 4'd0;

//             state <= ST_IDLE;
//         end
//         else begin
//             tx_start <= 1'b0;

//             case (state)
//                 ST_IDLE: begin
//                     if (init_done && sample_ready && print_tick)
//                         state <= ST_CAPTURE;
//                 end

//                 ST_CAPTURE: begin
//                     ax_r  <= accel_x[15:0];
//                     ay_r  <= accel_y[15:0];
//                     az_r  <= accel_z[15:0];
//                     gx_r  <= gyro_x[15:0];
//                     gy_r  <= gyro_y[15:0];
//                     gz_r  <= gyro_z[15:0];
//                     state <= ST_SPLIT;
//                 end

//                 ST_SPLIT: begin
//                     ax3 <= ax_r[15:12]; ax2 <= ax_r[11:8];  ax1 <= ax_r[7:4];  ax0 <= ax_r[3:0];
//                     ay3 <= ay_r[15:12]; ay2 <= ay_r[11:8];  ay1 <= ay_r[7:4];  ay0 <= ay_r[3:0];
//                     az3 <= az_r[15:12]; az2 <= az_r[11:8];  az1 <= az_r[7:4];  az0 <= az_r[3:0];
//                     gx3 <= gx_r[15:12]; gx2 <= gx_r[11:8];  gx1 <= gx_r[7:4];  gx0 <= gx_r[3:0];
//                     gy3 <= gy_r[15:12]; gy2 <= gy_r[11:8];  gy1 <= gy_r[7:4];  gy0 <= gy_r[3:0];
//                     gz3 <= gz_r[15:12]; gz2 <= gz_r[11:8];  gz1 <= gz_r[7:4];  gz0 <= gz_r[3:0];
//                     state <= ST_TX_A1;
//                 end

//                 ST_TX_A1:  begin if (!tx_busy) begin tx_data <= "A"; tx_start <= 1'b1; state <= ST_W_A1; end end
//                 ST_W_A1:   begin if (tx_done) state <= ST_TX_X1; end
//                 ST_TX_X1:  begin if (!tx_busy) begin tx_data <= "X"; tx_start <= 1'b1; state <= ST_W_X1; end end
//                 ST_W_X1:   begin if (tx_done) state <= ST_TX_EQ1; end
//                 ST_TX_EQ1: begin if (!tx_busy) begin tx_data <= "="; tx_start <= 1'b1; state <= ST_W_EQ1; end end
//                 ST_W_EQ1:  begin if (tx_done) state <= ST_TX_AX3; end
//                 ST_TX_AX3: begin if (!tx_busy) begin tx_data <= hex_to_ascii(ax3); tx_start <= 1'b1; state <= ST_W_AX3; end end
//                 ST_W_AX3:  begin if (tx_done) state <= ST_TX_AX2; end
//                 ST_TX_AX2: begin if (!tx_busy) begin tx_data <= hex_to_ascii(ax2); tx_start <= 1'b1; state <= ST_W_AX2; end end
//                 ST_W_AX2:  begin if (tx_done) state <= ST_TX_AX1; end
//                 ST_TX_AX1: begin if (!tx_busy) begin tx_data <= hex_to_ascii(ax1); tx_start <= 1'b1; state <= ST_W_AX1; end end
//                 ST_W_AX1:  begin if (tx_done) state <= ST_TX_AX0; end
//                 ST_TX_AX0: begin if (!tx_busy) begin tx_data <= hex_to_ascii(ax0); tx_start <= 1'b1; state <= ST_W_AX0; end end
//                 ST_W_AX0:  begin if (tx_done) state <= ST_TX_SP1; end
//                 ST_TX_SP1: begin if (!tx_busy) begin tx_data <= " "; tx_start <= 1'b1; state <= ST_W_SP1; end end
//                 ST_W_SP1:  begin if (tx_done) state <= ST_TX_A2; end

//                 ST_TX_A2:  begin if (!tx_busy) begin tx_data <= "A"; tx_start <= 1'b1; state <= ST_W_A2; end end
//                 ST_W_A2:   begin if (tx_done) state <= ST_TX_Y1; end
//                 ST_TX_Y1:  begin if (!tx_busy) begin tx_data <= "Y"; tx_start <= 1'b1; state <= ST_W_Y1; end end
//                 ST_W_Y1:   begin if (tx_done) state <= ST_TX_EQ2; end
//                 ST_TX_EQ2: begin if (!tx_busy) begin tx_data <= "="; tx_start <= 1'b1; state <= ST_W_EQ2; end end
//                 ST_W_EQ2:  begin if (tx_done) state <= ST_TX_AY3; end
//                 ST_TX_AY3: begin if (!tx_busy) begin tx_data <= hex_to_ascii(ay3); tx_start <= 1'b1; state <= ST_W_AY3; end end
//                 ST_W_AY3:  begin if (tx_done) state <= ST_TX_AY2; end
//                 ST_TX_AY2: begin if (!tx_busy) begin tx_data <= hex_to_ascii(ay2); tx_start <= 1'b1; state <= ST_W_AY2; end end
//                 ST_W_AY2:  begin if (tx_done) state <= ST_TX_AY1; end
//                 ST_TX_AY1: begin if (!tx_busy) begin tx_data <= hex_to_ascii(ay1); tx_start <= 1'b1; state <= ST_W_AY1; end end
//                 ST_W_AY1:  begin if (tx_done) state <= ST_TX_AY0; end
//                 ST_TX_AY0: begin if (!tx_busy) begin tx_data <= hex_to_ascii(ay0); tx_start <= 1'b1; state <= ST_W_AY0; end end
//                 ST_W_AY0:  begin if (tx_done) state <= ST_TX_SP2; end
//                 ST_TX_SP2: begin if (!tx_busy) begin tx_data <= " "; tx_start <= 1'b1; state <= ST_W_SP2; end end
//                 ST_W_SP2:  begin if (tx_done) state <= ST_TX_A3; end

//                 ST_TX_A3:  begin if (!tx_busy) begin tx_data <= "A"; tx_start <= 1'b1; state <= ST_W_A3; end end
//                 ST_W_A3:   begin if (tx_done) state <= ST_TX_Z1; end
//                 ST_TX_Z1:  begin if (!tx_busy) begin tx_data <= "Z"; tx_start <= 1'b1; state <= ST_W_Z1; end end
//                 ST_W_Z1:   begin if (tx_done) state <= ST_TX_EQ3; end
//                 ST_TX_EQ3: begin if (!tx_busy) begin tx_data <= "="; tx_start <= 1'b1; state <= ST_W_EQ3; end end
//                 ST_W_EQ3:  begin if (tx_done) state <= ST_TX_AZ3; end
//                 ST_TX_AZ3: begin if (!tx_busy) begin tx_data <= hex_to_ascii(az3); tx_start <= 1'b1; state <= ST_W_AZ3; end end
//                 ST_W_AZ3:  begin if (tx_done) state <= ST_TX_AZ2; end
//                 ST_TX_AZ2: begin if (!tx_busy) begin tx_data <= hex_to_ascii(az2); tx_start <= 1'b1; state <= ST_W_AZ2; end end
//                 ST_W_AZ2:  begin if (tx_done) state <= ST_TX_AZ1; end
//                 ST_TX_AZ1: begin if (!tx_busy) begin tx_data <= hex_to_ascii(az1); tx_start <= 1'b1; state <= ST_W_AZ1; end end
//                 ST_W_AZ1:  begin if (tx_done) state <= ST_TX_AZ0; end
//                 ST_TX_AZ0: begin if (!tx_busy) begin tx_data <= hex_to_ascii(az0); tx_start <= 1'b1; state <= ST_W_AZ0; end end
//                 ST_W_AZ0:  begin if (tx_done) state <= ST_TX_SP3; end
//                 ST_TX_SP3: begin if (!tx_busy) begin tx_data <= " "; tx_start <= 1'b1; state <= ST_W_SP3; end end
//                 ST_W_SP3:  begin if (tx_done) state <= ST_TX_G1; end

//                 ST_TX_G1:  begin if (!tx_busy) begin tx_data <= "G"; tx_start <= 1'b1; state <= ST_W_G1; end end
//                 ST_W_G1:   begin if (tx_done) state <= ST_TX_X2; end
//                 ST_TX_X2:  begin if (!tx_busy) begin tx_data <= "X"; tx_start <= 1'b1; state <= ST_W_X2; end end
//                 ST_W_X2:   begin if (tx_done) state <= ST_TX_EQ4; end
//                 ST_TX_EQ4: begin if (!tx_busy) begin tx_data <= "="; tx_start <= 1'b1; state <= ST_W_EQ4; end end
//                 ST_W_EQ4:  begin if (tx_done) state <= ST_TX_GX3; end
//                 ST_TX_GX3: begin if (!tx_busy) begin tx_data <= hex_to_ascii(gx3); tx_start <= 1'b1; state <= ST_W_GX3; end end
//                 ST_W_GX3:  begin if (tx_done) state <= ST_TX_GX2; end
//                 ST_TX_GX2: begin if (!tx_busy) begin tx_data <= hex_to_ascii(gx2); tx_start <= 1'b1; state <= ST_W_GX2; end end
//                 ST_W_GX2:  begin if (tx_done) state <= ST_TX_GX1; end
//                 ST_TX_GX1: begin if (!tx_busy) begin tx_data <= hex_to_ascii(gx1); tx_start <= 1'b1; state <= ST_W_GX1; end end
//                 ST_W_GX1:  begin if (tx_done) state <= ST_TX_GX0; end
//                 ST_TX_GX0: begin if (!tx_busy) begin tx_data <= hex_to_ascii(gx0); tx_start <= 1'b1; state <= ST_W_GX0; end end
//                 ST_W_GX0:  begin if (tx_done) state <= ST_TX_SP4; end
//                 ST_TX_SP4: begin if (!tx_busy) begin tx_data <= " "; tx_start <= 1'b1; state <= ST_W_SP4; end end
//                 ST_W_SP4:  begin if (tx_done) state <= ST_TX_G2; end

//                 ST_TX_G2:  begin if (!tx_busy) begin tx_data <= "G"; tx_start <= 1'b1; state <= ST_W_G2; end end
//                 ST_W_G2:   begin if (tx_done) state <= ST_TX_Y2; end
//                 ST_TX_Y2:  begin if (!tx_busy) begin tx_data <= "Y"; tx_start <= 1'b1; state <= ST_W_Y2; end end
//                 ST_W_Y2:   begin if (tx_done) state <= ST_TX_EQ5; end
//                 ST_TX_EQ5: begin if (!tx_busy) begin tx_data <= "="; tx_start <= 1'b1; state <= ST_W_EQ5; end end
//                 ST_W_EQ5:  begin if (tx_done) state <= ST_TX_GY3; end
//                 ST_TX_GY3: begin if (!tx_busy) begin tx_data <= hex_to_ascii(gy3); tx_start <= 1'b1; state <= ST_W_GY3; end end
//                 ST_W_GY3:  begin if (tx_done) state <= ST_TX_GY2; end
//                 ST_TX_GY2: begin if (!tx_busy) begin tx_data <= hex_to_ascii(gy2); tx_start <= 1'b1; state <= ST_W_GY2; end end
//                 ST_W_GY2:  begin if (tx_done) state <= ST_TX_GY1; end
//                 ST_TX_GY1: begin if (!tx_busy) begin tx_data <= hex_to_ascii(gy1); tx_start <= 1'b1; state <= ST_W_GY1; end end
//                 ST_W_GY1:  begin if (tx_done) state <= ST_TX_GY0; end
//                 ST_TX_GY0: begin if (!tx_busy) begin tx_data <= hex_to_ascii(gy0); tx_start <= 1'b1; state <= ST_W_GY0; end end
//                 ST_W_GY0:  begin if (tx_done) state <= ST_TX_SP5; end
//                 ST_TX_SP5: begin if (!tx_busy) begin tx_data <= " "; tx_start <= 1'b1; state <= ST_W_SP5; end end
//                 ST_W_SP5:  begin if (tx_done) state <= ST_TX_G3; end

//                 ST_TX_G3:  begin if (!tx_busy) begin tx_data <= "G"; tx_start <= 1'b1; state <= ST_W_G3; end end
//                 ST_W_G3:   begin if (tx_done) state <= ST_TX_Z2; end
//                 ST_TX_Z2:  begin if (!tx_busy) begin tx_data <= "Z"; tx_start <= 1'b1; state <= ST_W_Z2; end end
//                 ST_W_Z2:   begin if (tx_done) state <= ST_TX_EQ6; end
//                 ST_TX_EQ6: begin if (!tx_busy) begin tx_data <= "="; tx_start <= 1'b1; state <= ST_W_EQ6; end end
//                 ST_W_EQ6:  begin if (tx_done) state <= ST_TX_GZ3; end
//                 ST_TX_GZ3: begin if (!tx_busy) begin tx_data <= hex_to_ascii(gz3); tx_start <= 1'b1; state <= ST_W_GZ3; end end
//                 ST_W_GZ3:  begin if (tx_done) state <= ST_TX_GZ2; end
//                 ST_TX_GZ2: begin if (!tx_busy) begin tx_data <= hex_to_ascii(gz2); tx_start <= 1'b1; state <= ST_W_GZ2; end end
//                 ST_W_GZ2:  begin if (tx_done) state <= ST_TX_GZ1; end
//                 ST_TX_GZ1: begin if (!tx_busy) begin tx_data <= hex_to_ascii(gz1); tx_start <= 1'b1; state <= ST_W_GZ1; end end
//                 ST_W_GZ1:  begin if (tx_done) state <= ST_TX_GZ0; end
//                 ST_TX_GZ0: begin if (!tx_busy) begin tx_data <= hex_to_ascii(gz0); tx_start <= 1'b1; state <= ST_W_GZ0; end end
//                 ST_W_GZ0:  begin if (tx_done) state <= ST_TX_CR; end

//                 ST_TX_CR:  begin if (!tx_busy) begin tx_data <= 8'h0D; tx_start <= 1'b1; state <= ST_W_CR; end end
//                 ST_W_CR:   begin if (tx_done) state <= ST_TX_LF; end
//                 ST_TX_LF:  begin if (!tx_busy) begin tx_data <= 8'h0A; tx_start <= 1'b1; state <= ST_W_LF; end end
//                 ST_W_LF:   begin if (tx_done) state <= ST_IDLE; end

//                 default: begin
//                     state <= ST_IDLE;
//                 end
//             endcase
//         end
//     end

// endmodule