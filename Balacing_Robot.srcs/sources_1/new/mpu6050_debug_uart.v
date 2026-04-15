`timescale 1ns / 1ps
//==============================================================================
// mpu6050_debug_uart
// - MPU6050 6축 데이터 + 상보 필터 각도를 10진수로 UART 출력
// - 출력 형식:
//   AX=+00096 AY=-00005 AZ=+16320 GX=+00000 GY=+00001 GZ=+00000 AN=+00520\r\n
// - 부호: '+'/'-', 자릿수: 5자리 고정 (앞 0 포함)
// - AN: angle_calc 출력 (0.01° 단위)
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

    input  wire signed [15:0] angle,      // 상보 필터 각도 (0.01° 단위)

    // 런타임 PID 이득값 (출력용)
    input  wire [15:0] kp,
    input  wire [15:0] ki,
    input  wire [15:0] kd,

    output wire               uart_tx_o
);

    //--------------------------------------------------------------------------
    // 상태 정의
    // 각 축: label2 + '=' + sign + 5digits + space = 10문자 (GZ는 space 없음)
    // 총 125개 상태
    //--------------------------------------------------------------------------
    localparam [7:0]
        ST_IDLE    = 8'd0,
        ST_CAPTURE = 8'd1,
        ST_SPLIT   = 8'd2,

        // 새로 추가할 반복 뺄셈 상태
        ST_CALC_10000 = 8'd200,
        ST_CALC_1000  = 8'd201,
        ST_CALC_100   = 8'd202,
        ST_CALC_10    = 8'd203,
        ST_CALC_1     = 8'd204,

        // AX (states 3~22)
        ST_TX_AX_L1 = 8'd3,   ST_W_AX_L1 = 8'd4,
        ST_TX_AX_L2 = 8'd5,   ST_W_AX_L2 = 8'd6,
        ST_TX_AX_EQ = 8'd7,   ST_W_AX_EQ = 8'd8,
        ST_TX_AX_SG = 8'd9,   ST_W_AX_SG = 8'd10,
        ST_TX_AX_D4 = 8'd11,  ST_W_AX_D4 = 8'd12,
        ST_TX_AX_D3 = 8'd13,  ST_W_AX_D3 = 8'd14,
        ST_TX_AX_D2 = 8'd15,  ST_W_AX_D2 = 8'd16,
        ST_TX_AX_D1 = 8'd17,  ST_W_AX_D1 = 8'd18,
        ST_TX_AX_D0 = 8'd19,  ST_W_AX_D0 = 8'd20,
        ST_TX_AX_SP = 8'd21,  ST_W_AX_SP = 8'd22,

        // AY (states 23~42)
        ST_TX_AY_L1 = 8'd23,  ST_W_AY_L1 = 8'd24,
        ST_TX_AY_L2 = 8'd25,  ST_W_AY_L2 = 8'd26,
        ST_TX_AY_EQ = 8'd27,  ST_W_AY_EQ = 8'd28,
        ST_TX_AY_SG = 8'd29,  ST_W_AY_SG = 8'd30,
        ST_TX_AY_D4 = 8'd31,  ST_W_AY_D4 = 8'd32,
        ST_TX_AY_D3 = 8'd33,  ST_W_AY_D3 = 8'd34,
        ST_TX_AY_D2 = 8'd35,  ST_W_AY_D2 = 8'd36,
        ST_TX_AY_D1 = 8'd37,  ST_W_AY_D1 = 8'd38,
        ST_TX_AY_D0 = 8'd39,  ST_W_AY_D0 = 8'd40,
        ST_TX_AY_SP = 8'd41,  ST_W_AY_SP = 8'd42,

        // AZ (states 43~62)
        ST_TX_AZ_L1 = 8'd43,  ST_W_AZ_L1 = 8'd44,
        ST_TX_AZ_L2 = 8'd45,  ST_W_AZ_L2 = 8'd46,
        ST_TX_AZ_EQ = 8'd47,  ST_W_AZ_EQ = 8'd48,
        ST_TX_AZ_SG = 8'd49,  ST_W_AZ_SG = 8'd50,
        ST_TX_AZ_D4 = 8'd51,  ST_W_AZ_D4 = 8'd52,
        ST_TX_AZ_D3 = 8'd53,  ST_W_AZ_D3 = 8'd54,
        ST_TX_AZ_D2 = 8'd55,  ST_W_AZ_D2 = 8'd56,
        ST_TX_AZ_D1 = 8'd57,  ST_W_AZ_D1 = 8'd58,
        ST_TX_AZ_D0 = 8'd59,  ST_W_AZ_D0 = 8'd60,
        ST_TX_AZ_SP = 8'd61,  ST_W_AZ_SP = 8'd62,

        // GX (states 63~82)
        ST_TX_GX_L1 = 8'd63,  ST_W_GX_L1 = 8'd64,
        ST_TX_GX_L2 = 8'd65,  ST_W_GX_L2 = 8'd66,
        ST_TX_GX_EQ = 8'd67,  ST_W_GX_EQ = 8'd68,
        ST_TX_GX_SG = 8'd69,  ST_W_GX_SG = 8'd70,
        ST_TX_GX_D4 = 8'd71,  ST_W_GX_D4 = 8'd72,
        ST_TX_GX_D3 = 8'd73,  ST_W_GX_D3 = 8'd74,
        ST_TX_GX_D2 = 8'd75,  ST_W_GX_D2 = 8'd76,
        ST_TX_GX_D1 = 8'd77,  ST_W_GX_D1 = 8'd78,
        ST_TX_GX_D0 = 8'd79,  ST_W_GX_D0 = 8'd80,
        ST_TX_GX_SP = 8'd81,  ST_W_GX_SP = 8'd82,

        // GY (states 83~102)
        ST_TX_GY_L1 = 8'd83,  ST_W_GY_L1 = 8'd84,
        ST_TX_GY_L2 = 8'd85,  ST_W_GY_L2 = 8'd86,
        ST_TX_GY_EQ = 8'd87,  ST_W_GY_EQ = 8'd88,
        ST_TX_GY_SG = 8'd89,  ST_W_GY_SG = 8'd90,
        ST_TX_GY_D4 = 8'd91,  ST_W_GY_D4 = 8'd92,
        ST_TX_GY_D3 = 8'd93,  ST_W_GY_D3 = 8'd94,
        ST_TX_GY_D2 = 8'd95,  ST_W_GY_D2 = 8'd96,
        ST_TX_GY_D1 = 8'd97,  ST_W_GY_D1 = 8'd98,
        ST_TX_GY_D0 = 8'd99,  ST_W_GY_D0 = 8'd100,
        ST_TX_GY_SP = 8'd101, ST_W_GY_SP = 8'd102,

        // GZ (states 103~120, space 없음)
        ST_TX_GZ_L1 = 8'd103, ST_W_GZ_L1 = 8'd104,
        ST_TX_GZ_L2 = 8'd105, ST_W_GZ_L2 = 8'd106,
        ST_TX_GZ_EQ = 8'd107, ST_W_GZ_EQ = 8'd108,
        ST_TX_GZ_SG = 8'd109, ST_W_GZ_SG = 8'd110,
        ST_TX_GZ_D4 = 8'd111, ST_W_GZ_D4 = 8'd112,
        ST_TX_GZ_D3 = 8'd113, ST_W_GZ_D3 = 8'd114,
        ST_TX_GZ_D2 = 8'd115, ST_W_GZ_D2 = 8'd116,
        ST_TX_GZ_D1 = 8'd117, ST_W_GZ_D1 = 8'd118,
        ST_TX_GZ_D0 = 8'd119, ST_W_GZ_D0 = 8'd120,

        // AN (angle, space 포함)
        ST_TX_AN_SP = 8'd125, ST_W_AN_SP = 8'd126,
        ST_TX_AN_L1 = 8'd127, ST_W_AN_L1 = 8'd128,
        ST_TX_AN_L2 = 8'd129, ST_W_AN_L2 = 8'd130,
        ST_TX_AN_EQ = 8'd131, ST_W_AN_EQ = 8'd132,
        ST_TX_AN_SG = 8'd133, ST_W_AN_SG = 8'd134,
        ST_TX_AN_D4 = 8'd135, ST_W_AN_D4 = 8'd136,
        ST_TX_AN_D3 = 8'd137, ST_W_AN_D3 = 8'd138,
        ST_TX_AN_D2 = 8'd139, ST_W_AN_D2 = 8'd140,
        ST_TX_AN_D1 = 8'd141, ST_W_AN_D1 = 8'd142,
        ST_TX_AN_D0 = 8'd143, ST_W_AN_D0 = 8'd144,

        // CR LF
        ST_TX_CR = 8'd121, ST_W_CR = 8'd122,
        ST_TX_LF = 8'd123, ST_W_LF = 8'd124,

        // KP (states 145~162)
        ST_TX_KP_SP = 8'd145, ST_W_KP_SP = 8'd146,
        ST_TX_KP_L1 = 8'd147, ST_W_KP_L1 = 8'd148,
        ST_TX_KP_L2 = 8'd149, ST_W_KP_L2 = 8'd150,
        ST_TX_KP_EQ = 8'd151, ST_W_KP_EQ = 8'd152,
        ST_TX_KP_D4 = 8'd153, ST_W_KP_D4 = 8'd154,
        ST_TX_KP_D3 = 8'd155, ST_W_KP_D3 = 8'd156,
        ST_TX_KP_D2 = 8'd157, ST_W_KP_D2 = 8'd158,
        ST_TX_KP_D1 = 8'd159, ST_W_KP_D1 = 8'd160,
        ST_TX_KP_D0 = 8'd161, ST_W_KP_D0 = 8'd162,

        // KI (states 163~180)
        ST_TX_KI_SP = 8'd163, ST_W_KI_SP = 8'd164,
        ST_TX_KI_L1 = 8'd165, ST_W_KI_L1 = 8'd166,
        ST_TX_KI_L2 = 8'd167, ST_W_KI_L2 = 8'd168,
        ST_TX_KI_EQ = 8'd169, ST_W_KI_EQ = 8'd170,
        ST_TX_KI_D4 = 8'd171, ST_W_KI_D4 = 8'd172,
        ST_TX_KI_D3 = 8'd173, ST_W_KI_D3 = 8'd174,
        ST_TX_KI_D2 = 8'd175, ST_W_KI_D2 = 8'd176,
        ST_TX_KI_D1 = 8'd177, ST_W_KI_D1 = 8'd178,
        ST_TX_KI_D0 = 8'd179, ST_W_KI_D0 = 8'd180,

        // KD (states 181~198)
        ST_TX_KD_SP = 8'd181, ST_W_KD_SP = 8'd182,
        ST_TX_KD_L1 = 8'd183, ST_W_KD_L1 = 8'd184,
        ST_TX_KD_L2 = 8'd185, ST_W_KD_L2 = 8'd186,
        ST_TX_KD_EQ = 8'd187, ST_W_KD_EQ = 8'd188,
        ST_TX_KD_D4 = 8'd189, ST_W_KD_D4 = 8'd190,
        ST_TX_KD_D3 = 8'd191, ST_W_KD_D3 = 8'd192,
        ST_TX_KD_D2 = 8'd193, ST_W_KD_D2 = 8'd194,
        ST_TX_KD_D1 = 8'd195, ST_W_KD_D1 = 8'd196,
        ST_TX_KD_D0 = 8'd197, ST_W_KD_D0 = 8'd198;

    reg [7:0] state;

    //--------------------------------------------------------------------------
    // UART 송신기 연결
    //--------------------------------------------------------------------------
    reg        tx_start;
    reg [7:0]  tx_data;
    reg        tx_busy_r;
    wire       tx_done;
    wire       b_tick;

    baud_rate u_baud_debug (
        .clk   (clk),
        .reset (~rst_n),
        .b_tick(b_tick)
    );

    uart_tx u_uart_tx (
        .clk     (clk),
        .reset   (~rst_n),
        .b_tick  (b_tick),
        .tx_start(tx_start),
        .tx_data (tx_data),
        .tx_pin  (uart_tx_o),
        .tx_done (tx_done)
    );

    // tx_busy_r: tx_start로 set, tx_done으로 clear
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)        tx_busy_r <= 1'b0;
        else if (tx_start) tx_busy_r <= 1'b1;
        else if (tx_done)  tx_busy_r <= 1'b0;
    end


    localparam [31:0] PRINT_CNT_MAX = 32'd50_000_000;  // 0.5초 (1Hz)

    reg [31:0] print_cnt;
    reg        print_tick;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            print_cnt  <= 32'd0;
            print_tick <= 1'b0;
        end else begin
            print_tick <= 1'b0;
            if (print_cnt == PRINT_CNT_MAX) begin
                print_cnt  <= 32'd0;
                print_tick <= 1'b1;
            end else begin
                print_cnt <= print_cnt + 32'd1;
            end
        end
    end

    //--------------------------------------------------------------------------
    // data_valid 보관
    //--------------------------------------------------------------------------
    reg sample_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sample_ready <= 1'b0;
        end else begin
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
    reg [15:0] ax_r, ay_r, az_r, gx_r, gy_r, gz_r;

    //--------------------------------------------------------------------------
    // 십진수 변환 결과 저장
    // sign: '+' or '-' (ASCII)
    // d4~d0: 만, 천, 백, 십, 일 자리 (0~9)
    //--------------------------------------------------------------------------
    reg [7:0] ax_sg; reg [3:0] ax_d4, ax_d3, ax_d2, ax_d1, ax_d0;
    reg [7:0] ay_sg; reg [3:0] ay_d4, ay_d3, ay_d2, ay_d1, ay_d0;
    reg [7:0] az_sg; reg [3:0] az_d4, az_d3, az_d2, az_d1, az_d0;
    reg [7:0] gx_sg; reg [3:0] gx_d4, gx_d3, gx_d2, gx_d1, gx_d0;
    reg [7:0] gy_sg; reg [3:0] gy_d4, gy_d3, gy_d2, gy_d1, gy_d0;
    reg [7:0] gz_sg; reg [3:0] gz_d4, gz_d3, gz_d2, gz_d1, gz_d0;
    reg [7:0] an_sg; reg [3:0] an_d4, an_d3, an_d2, an_d1, an_d0;
    // KP/KI/KD: 부호 없음 (항상 양수)
    reg [3:0] kp_d4, kp_d3, kp_d2, kp_d1, kp_d0;
    reg [3:0] ki_d4, ki_d3, ki_d2, ki_d1, ki_d0;
    reg [3:0] kd_d4, kd_d3, kd_d2, kd_d1, kd_d0;

    // ST_SPLIT 내부에서 절댓값 계산용 임시 변수 (blocking 대입으로 사용)
    reg [15:0] tmp_u;
    reg [15:0] t_ax, t_ay, t_az, t_gx, t_gy, t_gz, t_an;

    //--------------------------------------------------------------------------
    // digit (0~9) → ASCII 문자
    //--------------------------------------------------------------------------
    function [7:0] d2a;
        input [3:0] d;
        begin
            d2a = {4'b0011, d};  // '0'=0x30, '1'=0x31, ...
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

            ax_sg <= "+"; ax_d4 <= 0; ax_d3 <= 0; ax_d2 <= 0; ax_d1 <= 0; ax_d0 <= 0;
            ay_sg <= "+"; ay_d4 <= 0; ay_d3 <= 0; ay_d2 <= 0; ay_d1 <= 0; ay_d0 <= 0;
            az_sg <= "+"; az_d4 <= 0; az_d3 <= 0; az_d2 <= 0; az_d1 <= 0; az_d0 <= 0;
            gx_sg <= "+"; gx_d4 <= 0; gx_d3 <= 0; gx_d2 <= 0; gx_d1 <= 0; gx_d0 <= 0;
            gy_sg <= "+"; gy_d4 <= 0; gy_d3 <= 0; gy_d2 <= 0; gy_d1 <= 0; gy_d0 <= 0;
            gz_sg <= "+"; gz_d4 <= 0; gz_d3 <= 0; gz_d2 <= 0; gz_d1 <= 0; gz_d0 <= 0;
            an_sg <= "+"; an_d4 <= 0; an_d3 <= 0; an_d2 <= 0; an_d1 <= 0; an_d0 <= 0;
            kp_d4 <= 0; kp_d3 <= 0; kp_d2 <= 0; kp_d1 <= 0; kp_d0 <= 0;
            ki_d4 <= 0; ki_d3 <= 0; ki_d2 <= 0; ki_d1 <= 0; ki_d0 <= 0;
            kd_d4 <= 0; kd_d3 <= 0; kd_d2 <= 0; kd_d1 <= 0; kd_d0 <= 0;

            tmp_u <= 16'd0;
            state <= ST_IDLE;
        end
        else begin
            tx_start <= 1'b0;

            case (state)

                ST_IDLE: begin
                    if (init_done && sample_ready && print_tick)
                        state <= ST_CAPTURE;
                end

                //==========================================================
                // 수정된 부분: 값 캡처 및 초기화
                //==========================================================
                ST_CAPTURE: begin
                    // 1. 부호 판별
                    ax_sg <= accel_x[15] ? "-" : "+";
                    ay_sg <= accel_y[15] ? "-" : "+";
                    az_sg <= accel_z[15] ? "-" : "+";
                    gx_sg <= gyro_x[15]  ? "-" : "+";
                    gy_sg <= gyro_y[15]  ? "-" : "+";
                    gz_sg <= gyro_z[15]  ? "-" : "+";
                    an_sg <= angle[15]   ? "-" : "+";

                    // 2. 절댓값 저장
                    t_ax <= accel_x[15] ? (~accel_x + 1'b1) : accel_x;
                    t_ay <= accel_y[15] ? (~accel_y + 1'b1) : accel_y;
                    t_az <= accel_z[15] ? (~accel_z + 1'b1) : accel_z;
                    t_gx <= gyro_x[15]  ? (~gyro_x  + 1'b1) : gyro_x;
                    t_gy <= gyro_y[15]  ? (~gyro_y  + 1'b1) : gyro_y;
                    t_gz <= gyro_z[15]  ? (~gyro_z  + 1'b1) : gyro_z;
                    t_an <= angle[15]   ? (~angle   + 1'b1) : angle;

                    // 3. 자릿수 카운터 초기화
                    ax_d4 <= 0; ax_d3 <= 0; ax_d2 <= 0; ax_d1 <= 0; ax_d0 <= 0;
                    ay_d4 <= 0; ay_d3 <= 0; ay_d2 <= 0; ay_d1 <= 0; ay_d0 <= 0;
                    az_d4 <= 0; az_d3 <= 0; az_d2 <= 0; az_d1 <= 0; az_d0 <= 0;
                    gx_d4 <= 0; gx_d3 <= 0; gx_d2 <= 0; gx_d1 <= 0; gx_d0 <= 0;
                    gy_d4 <= 0; gy_d3 <= 0; gy_d2 <= 0; gy_d1 <= 0; gy_d0 <= 0;
                    gz_d4 <= 0; gz_d3 <= 0; gz_d2 <= 0; gz_d1 <= 0; gz_d0 <= 0;
                    an_d4 <= 0; an_d3 <= 0; an_d2 <= 0; an_d1 <= 0; an_d0 <= 0;

                    state <= ST_CALC_10000;
                end

                //==========================================================
                // 수정된 부분: 반복 뺄셈 (나눗셈 대체)
                //==========================================================
                ST_CALC_10000: begin
                    if (t_ax >= 10000 || t_ay >= 10000 || t_az >= 10000 || t_gx >= 10000 || t_gy >= 10000 || t_gz >= 10000 || t_an >= 10000) begin
                        if (t_ax >= 10000) begin t_ax <= t_ax - 10000; ax_d4 <= ax_d4 + 4'd1; end
                        if (t_ay >= 10000) begin t_ay <= t_ay - 10000; ay_d4 <= ay_d4 + 4'd1; end
                        if (t_az >= 10000) begin t_az <= t_az - 10000; az_d4 <= az_d4 + 4'd1; end
                        if (t_gx >= 10000) begin t_gx <= t_gx - 10000; gx_d4 <= gx_d4 + 4'd1; end
                        if (t_gy >= 10000) begin t_gy <= t_gy - 10000; gy_d4 <= gy_d4 + 4'd1; end
                        if (t_gz >= 10000) begin t_gz <= t_gz - 10000; gz_d4 <= gz_d4 + 4'd1; end
                        if (t_an >= 10000) begin t_an <= t_an - 10000; an_d4 <= an_d4 + 4'd1; end
                    end else state <= ST_CALC_1000;
                end

                ST_CALC_1000: begin
                    if (t_ax >= 1000 || t_ay >= 1000 || t_az >= 1000 || t_gx >= 1000 || t_gy >= 1000 || t_gz >= 1000 || t_an >= 1000) begin
                        if (t_ax >= 1000) begin t_ax <= t_ax - 1000; ax_d3 <= ax_d3 + 4'd1; end
                        if (t_ay >= 1000) begin t_ay <= t_ay - 1000; ay_d3 <= ay_d3 + 4'd1; end
                        if (t_az >= 1000) begin t_az <= t_az - 1000; az_d3 <= az_d3 + 4'd1; end
                        if (t_gx >= 1000) begin t_gx <= t_gx - 1000; gx_d3 <= gx_d3 + 4'd1; end
                        if (t_gy >= 1000) begin t_gy <= t_gy - 1000; gy_d3 <= gy_d3 + 4'd1; end
                        if (t_gz >= 1000) begin t_gz <= t_gz - 1000; gz_d3 <= gz_d3 + 4'd1; end
                        if (t_an >= 1000) begin t_an <= t_an - 1000; an_d3 <= an_d3 + 4'd1; end
                    end else state <= ST_CALC_100;
                end

                ST_CALC_100: begin
                    if (t_ax >= 100 || t_ay >= 100 || t_az >= 100 || t_gx >= 100 || t_gy >= 100 || t_gz >= 100 || t_an >= 100) begin
                        if (t_ax >= 100) begin t_ax <= t_ax - 100; ax_d2 <= ax_d2 + 4'd1; end
                        if (t_ay >= 100) begin t_ay <= t_ay - 100; ay_d2 <= ay_d2 + 4'd1; end
                        if (t_az >= 100) begin t_az <= t_az - 100; az_d2 <= az_d2 + 4'd1; end
                        if (t_gx >= 100) begin t_gx <= t_gx - 100; gx_d2 <= gx_d2 + 4'd1; end
                        if (t_gy >= 100) begin t_gy <= t_gy - 100; gy_d2 <= gy_d2 + 4'd1; end
                        if (t_gz >= 100) begin t_gz <= t_gz - 100; gz_d2 <= gz_d2 + 4'd1; end
                        if (t_an >= 100) begin t_an <= t_an - 100; an_d2 <= an_d2 + 4'd1; end
                    end else state <= ST_CALC_10;
                end

                ST_CALC_10: begin
                    if (t_ax >= 10 || t_ay >= 10 || t_az >= 10 || t_gx >= 10 || t_gy >= 10 || t_gz >= 10 || t_an >= 10) begin
                        if (t_ax >= 10) begin t_ax <= t_ax - 10; ax_d1 <= ax_d1 + 4'd1; end
                        if (t_ay >= 10) begin t_ay <= t_ay - 10; ay_d1 <= ay_d1 + 4'd1; end
                        if (t_az >= 10) begin t_az <= t_az - 10; az_d1 <= az_d1 + 4'd1; end
                        if (t_gx >= 10) begin t_gx <= t_gx - 10; gx_d1 <= gx_d1 + 4'd1; end
                        if (t_gy >= 10) begin t_gy <= t_gy - 10; gy_d1 <= gy_d1 + 4'd1; end
                        if (t_gz >= 10) begin t_gz <= t_gz - 10; gz_d1 <= gz_d1 + 4'd1; end
                        if (t_an >= 10) begin t_an <= t_an - 10; an_d1 <= an_d1 + 4'd1; end
                    end else state <= ST_CALC_1;
                end

                ST_CALC_1: begin
                    ax_d0 <= t_ax[3:0]; ay_d0 <= t_ay[3:0]; az_d0 <= t_az[3:0];
                    gx_d0 <= t_gx[3:0]; gy_d0 <= t_gy[3:0]; gz_d0 <= t_gz[3:0];
                    an_d0 <= t_an[3:0];
                    state <= ST_TX_AX_L1; // 변환 완료 후 UART 송신 시작
                end

                // ------ AX ------
                ST_TX_AX_L1: begin if (!tx_busy_r) begin tx_data <= "A";      tx_start <= 1'b1; state <= ST_W_AX_L1; end end
                ST_W_AX_L1:  begin if (tx_done) state <= ST_TX_AX_L2; end
                ST_TX_AX_L2: begin if (!tx_busy_r) begin tx_data <= "X";      tx_start <= 1'b1; state <= ST_W_AX_L2; end end
                ST_W_AX_L2:  begin if (tx_done) state <= ST_TX_AX_EQ; end
                ST_TX_AX_EQ: begin if (!tx_busy_r) begin tx_data <= "=";      tx_start <= 1'b1; state <= ST_W_AX_EQ; end end
                ST_W_AX_EQ:  begin if (tx_done) state <= ST_TX_AX_SG; end
                ST_TX_AX_SG: begin if (!tx_busy_r) begin tx_data <= ax_sg;    tx_start <= 1'b1; state <= ST_W_AX_SG; end end
                ST_W_AX_SG:  begin if (tx_done) state <= ST_TX_AX_D4; end
                ST_TX_AX_D4: begin if (!tx_busy_r) begin tx_data <= d2a(ax_d4); tx_start <= 1'b1; state <= ST_W_AX_D4; end end
                ST_W_AX_D4:  begin if (tx_done) state <= ST_TX_AX_D3; end
                ST_TX_AX_D3: begin if (!tx_busy_r) begin tx_data <= d2a(ax_d3); tx_start <= 1'b1; state <= ST_W_AX_D3; end end
                ST_W_AX_D3:  begin if (tx_done) state <= ST_TX_AX_D2; end
                ST_TX_AX_D2: begin if (!tx_busy_r) begin tx_data <= d2a(ax_d2); tx_start <= 1'b1; state <= ST_W_AX_D2; end end
                ST_W_AX_D2:  begin if (tx_done) state <= ST_TX_AX_D1; end
                ST_TX_AX_D1: begin if (!tx_busy_r) begin tx_data <= d2a(ax_d1); tx_start <= 1'b1; state <= ST_W_AX_D1; end end
                ST_W_AX_D1:  begin if (tx_done) state <= ST_TX_AX_D0; end
                ST_TX_AX_D0: begin if (!tx_busy_r) begin tx_data <= d2a(ax_d0); tx_start <= 1'b1; state <= ST_W_AX_D0; end end
                ST_W_AX_D0:  begin if (tx_done) state <= ST_TX_AX_SP; end
                ST_TX_AX_SP: begin if (!tx_busy_r) begin tx_data <= " ";      tx_start <= 1'b1; state <= ST_W_AX_SP; end end
                ST_W_AX_SP:  begin if (tx_done) state <= ST_TX_AY_L1; end

                // ------ AY ------
                ST_TX_AY_L1: begin if (!tx_busy_r) begin tx_data <= "A";      tx_start <= 1'b1; state <= ST_W_AY_L1; end end
                ST_W_AY_L1:  begin if (tx_done) state <= ST_TX_AY_L2; end
                ST_TX_AY_L2: begin if (!tx_busy_r) begin tx_data <= "Y";      tx_start <= 1'b1; state <= ST_W_AY_L2; end end
                ST_W_AY_L2:  begin if (tx_done) state <= ST_TX_AY_EQ; end
                ST_TX_AY_EQ: begin if (!tx_busy_r) begin tx_data <= "=";      tx_start <= 1'b1; state <= ST_W_AY_EQ; end end
                ST_W_AY_EQ:  begin if (tx_done) state <= ST_TX_AY_SG; end
                ST_TX_AY_SG: begin if (!tx_busy_r) begin tx_data <= ay_sg;    tx_start <= 1'b1; state <= ST_W_AY_SG; end end
                ST_W_AY_SG:  begin if (tx_done) state <= ST_TX_AY_D4; end
                ST_TX_AY_D4: begin if (!tx_busy_r) begin tx_data <= d2a(ay_d4); tx_start <= 1'b1; state <= ST_W_AY_D4; end end
                ST_W_AY_D4:  begin if (tx_done) state <= ST_TX_AY_D3; end
                ST_TX_AY_D3: begin if (!tx_busy_r) begin tx_data <= d2a(ay_d3); tx_start <= 1'b1; state <= ST_W_AY_D3; end end
                ST_W_AY_D3:  begin if (tx_done) state <= ST_TX_AY_D2; end
                ST_TX_AY_D2: begin if (!tx_busy_r) begin tx_data <= d2a(ay_d2); tx_start <= 1'b1; state <= ST_W_AY_D2; end end
                ST_W_AY_D2:  begin if (tx_done) state <= ST_TX_AY_D1; end
                ST_TX_AY_D1: begin if (!tx_busy_r) begin tx_data <= d2a(ay_d1); tx_start <= 1'b1; state <= ST_W_AY_D1; end end
                ST_W_AY_D1:  begin if (tx_done) state <= ST_TX_AY_D0; end
                ST_TX_AY_D0: begin if (!tx_busy_r) begin tx_data <= d2a(ay_d0); tx_start <= 1'b1; state <= ST_W_AY_D0; end end
                ST_W_AY_D0:  begin if (tx_done) state <= ST_TX_AY_SP; end
                ST_TX_AY_SP: begin if (!tx_busy_r) begin tx_data <= " ";      tx_start <= 1'b1; state <= ST_W_AY_SP; end end
                ST_W_AY_SP:  begin if (tx_done) state <= ST_TX_AZ_L1; end

                // ------ AZ ------
                ST_TX_AZ_L1: begin if (!tx_busy_r) begin tx_data <= "A";      tx_start <= 1'b1; state <= ST_W_AZ_L1; end end
                ST_W_AZ_L1:  begin if (tx_done) state <= ST_TX_AZ_L2; end
                ST_TX_AZ_L2: begin if (!tx_busy_r) begin tx_data <= "Z";      tx_start <= 1'b1; state <= ST_W_AZ_L2; end end
                ST_W_AZ_L2:  begin if (tx_done) state <= ST_TX_AZ_EQ; end
                ST_TX_AZ_EQ: begin if (!tx_busy_r) begin tx_data <= "=";      tx_start <= 1'b1; state <= ST_W_AZ_EQ; end end
                ST_W_AZ_EQ:  begin if (tx_done) state <= ST_TX_AZ_SG; end
                ST_TX_AZ_SG: begin if (!tx_busy_r) begin tx_data <= az_sg;    tx_start <= 1'b1; state <= ST_W_AZ_SG; end end
                ST_W_AZ_SG:  begin if (tx_done) state <= ST_TX_AZ_D4; end
                ST_TX_AZ_D4: begin if (!tx_busy_r) begin tx_data <= d2a(az_d4); tx_start <= 1'b1; state <= ST_W_AZ_D4; end end
                ST_W_AZ_D4:  begin if (tx_done) state <= ST_TX_AZ_D3; end
                ST_TX_AZ_D3: begin if (!tx_busy_r) begin tx_data <= d2a(az_d3); tx_start <= 1'b1; state <= ST_W_AZ_D3; end end
                ST_W_AZ_D3:  begin if (tx_done) state <= ST_TX_AZ_D2; end
                ST_TX_AZ_D2: begin if (!tx_busy_r) begin tx_data <= d2a(az_d2); tx_start <= 1'b1; state <= ST_W_AZ_D2; end end
                ST_W_AZ_D2:  begin if (tx_done) state <= ST_TX_AZ_D1; end
                ST_TX_AZ_D1: begin if (!tx_busy_r) begin tx_data <= d2a(az_d1); tx_start <= 1'b1; state <= ST_W_AZ_D1; end end
                ST_W_AZ_D1:  begin if (tx_done) state <= ST_TX_AZ_D0; end
                ST_TX_AZ_D0: begin if (!tx_busy_r) begin tx_data <= d2a(az_d0); tx_start <= 1'b1; state <= ST_W_AZ_D0; end end
                ST_W_AZ_D0:  begin if (tx_done) state <= ST_TX_AZ_SP; end
                ST_TX_AZ_SP: begin if (!tx_busy_r) begin tx_data <= " ";      tx_start <= 1'b1; state <= ST_W_AZ_SP; end end
                ST_W_AZ_SP:  begin if (tx_done) state <= ST_TX_GX_L1; end

                // ------ GX ------
                ST_TX_GX_L1: begin if (!tx_busy_r) begin tx_data <= "G";      tx_start <= 1'b1; state <= ST_W_GX_L1; end end
                ST_W_GX_L1:  begin if (tx_done) state <= ST_TX_GX_L2; end
                ST_TX_GX_L2: begin if (!tx_busy_r) begin tx_data <= "X";      tx_start <= 1'b1; state <= ST_W_GX_L2; end end
                ST_W_GX_L2:  begin if (tx_done) state <= ST_TX_GX_EQ; end
                ST_TX_GX_EQ: begin if (!tx_busy_r) begin tx_data <= "=";      tx_start <= 1'b1; state <= ST_W_GX_EQ; end end
                ST_W_GX_EQ:  begin if (tx_done) state <= ST_TX_GX_SG; end
                ST_TX_GX_SG: begin if (!tx_busy_r) begin tx_data <= gx_sg;    tx_start <= 1'b1; state <= ST_W_GX_SG; end end
                ST_W_GX_SG:  begin if (tx_done) state <= ST_TX_GX_D4; end
                ST_TX_GX_D4: begin if (!tx_busy_r) begin tx_data <= d2a(gx_d4); tx_start <= 1'b1; state <= ST_W_GX_D4; end end
                ST_W_GX_D4:  begin if (tx_done) state <= ST_TX_GX_D3; end
                ST_TX_GX_D3: begin if (!tx_busy_r) begin tx_data <= d2a(gx_d3); tx_start <= 1'b1; state <= ST_W_GX_D3; end end
                ST_W_GX_D3:  begin if (tx_done) state <= ST_TX_GX_D2; end
                ST_TX_GX_D2: begin if (!tx_busy_r) begin tx_data <= d2a(gx_d2); tx_start <= 1'b1; state <= ST_W_GX_D2; end end
                ST_W_GX_D2:  begin if (tx_done) state <= ST_TX_GX_D1; end
                ST_TX_GX_D1: begin if (!tx_busy_r) begin tx_data <= d2a(gx_d1); tx_start <= 1'b1; state <= ST_W_GX_D1; end end
                ST_W_GX_D1:  begin if (tx_done) state <= ST_TX_GX_D0; end
                ST_TX_GX_D0: begin if (!tx_busy_r) begin tx_data <= d2a(gx_d0); tx_start <= 1'b1; state <= ST_W_GX_D0; end end
                ST_W_GX_D0:  begin if (tx_done) state <= ST_TX_GX_SP; end
                ST_TX_GX_SP: begin if (!tx_busy_r) begin tx_data <= " ";      tx_start <= 1'b1; state <= ST_W_GX_SP; end end
                ST_W_GX_SP:  begin if (tx_done) state <= ST_TX_GY_L1; end

                // ------ GY ------
                ST_TX_GY_L1: begin if (!tx_busy_r) begin tx_data <= "G";      tx_start <= 1'b1; state <= ST_W_GY_L1; end end
                ST_W_GY_L1:  begin if (tx_done) state <= ST_TX_GY_L2; end
                ST_TX_GY_L2: begin if (!tx_busy_r) begin tx_data <= "Y";      tx_start <= 1'b1; state <= ST_W_GY_L2; end end
                ST_W_GY_L2:  begin if (tx_done) state <= ST_TX_GY_EQ; end
                ST_TX_GY_EQ: begin if (!tx_busy_r) begin tx_data <= "=";      tx_start <= 1'b1; state <= ST_W_GY_EQ; end end
                ST_W_GY_EQ:  begin if (tx_done) state <= ST_TX_GY_SG; end
                ST_TX_GY_SG: begin if (!tx_busy_r) begin tx_data <= gy_sg;    tx_start <= 1'b1; state <= ST_W_GY_SG; end end
                ST_W_GY_SG:  begin if (tx_done) state <= ST_TX_GY_D4; end
                ST_TX_GY_D4: begin if (!tx_busy_r) begin tx_data <= d2a(gy_d4); tx_start <= 1'b1; state <= ST_W_GY_D4; end end
                ST_W_GY_D4:  begin if (tx_done) state <= ST_TX_GY_D3; end
                ST_TX_GY_D3: begin if (!tx_busy_r) begin tx_data <= d2a(gy_d3); tx_start <= 1'b1; state <= ST_W_GY_D3; end end
                ST_W_GY_D3:  begin if (tx_done) state <= ST_TX_GY_D2; end
                ST_TX_GY_D2: begin if (!tx_busy_r) begin tx_data <= d2a(gy_d2); tx_start <= 1'b1; state <= ST_W_GY_D2; end end
                ST_W_GY_D2:  begin if (tx_done) state <= ST_TX_GY_D1; end
                ST_TX_GY_D1: begin if (!tx_busy_r) begin tx_data <= d2a(gy_d1); tx_start <= 1'b1; state <= ST_W_GY_D1; end end
                ST_W_GY_D1:  begin if (tx_done) state <= ST_TX_GY_D0; end
                ST_TX_GY_D0: begin if (!tx_busy_r) begin tx_data <= d2a(gy_d0); tx_start <= 1'b1; state <= ST_W_GY_D0; end end
                ST_W_GY_D0:  begin if (tx_done) state <= ST_TX_GY_SP; end
                ST_TX_GY_SP: begin if (!tx_busy_r) begin tx_data <= " ";      tx_start <= 1'b1; state <= ST_W_GY_SP; end end
                ST_W_GY_SP:  begin if (tx_done) state <= ST_TX_GZ_L1; end

                // ------ GZ (뒤에 space 없음) ------
                ST_TX_GZ_L1: begin if (!tx_busy_r) begin tx_data <= "G";      tx_start <= 1'b1; state <= ST_W_GZ_L1; end end
                ST_W_GZ_L1:  begin if (tx_done) state <= ST_TX_GZ_L2; end
                ST_TX_GZ_L2: begin if (!tx_busy_r) begin tx_data <= "Z";      tx_start <= 1'b1; state <= ST_W_GZ_L2; end end
                ST_W_GZ_L2:  begin if (tx_done) state <= ST_TX_GZ_EQ; end
                ST_TX_GZ_EQ: begin if (!tx_busy_r) begin tx_data <= "=";      tx_start <= 1'b1; state <= ST_W_GZ_EQ; end end
                ST_W_GZ_EQ:  begin if (tx_done) state <= ST_TX_GZ_SG; end
                ST_TX_GZ_SG: begin if (!tx_busy_r) begin tx_data <= gz_sg;    tx_start <= 1'b1; state <= ST_W_GZ_SG; end end
                ST_W_GZ_SG:  begin if (tx_done) state <= ST_TX_GZ_D4; end
                ST_TX_GZ_D4: begin if (!tx_busy_r) begin tx_data <= d2a(gz_d4); tx_start <= 1'b1; state <= ST_W_GZ_D4; end end
                ST_W_GZ_D4:  begin if (tx_done) state <= ST_TX_GZ_D3; end
                ST_TX_GZ_D3: begin if (!tx_busy_r) begin tx_data <= d2a(gz_d3); tx_start <= 1'b1; state <= ST_W_GZ_D3; end end
                ST_W_GZ_D3:  begin if (tx_done) state <= ST_TX_GZ_D2; end
                ST_TX_GZ_D2: begin if (!tx_busy_r) begin tx_data <= d2a(gz_d2); tx_start <= 1'b1; state <= ST_W_GZ_D2; end end
                ST_W_GZ_D2:  begin if (tx_done) state <= ST_TX_GZ_D1; end
                ST_TX_GZ_D1: begin if (!tx_busy_r) begin tx_data <= d2a(gz_d1); tx_start <= 1'b1; state <= ST_W_GZ_D1; end end
                ST_W_GZ_D1:  begin if (tx_done) state <= ST_TX_GZ_D0; end
                ST_TX_GZ_D0: begin if (!tx_busy_r) begin tx_data <= d2a(gz_d0); tx_start <= 1'b1; state <= ST_W_GZ_D0; end end
                ST_W_GZ_D0:  begin if (tx_done) state <= ST_TX_AN_SP; end

                // ------ AN (angle) ------
                ST_TX_AN_SP: begin if (!tx_busy_r) begin tx_data <= " ";      tx_start <= 1'b1; state <= ST_W_AN_SP; end end
                ST_W_AN_SP:  begin if (tx_done) state <= ST_TX_AN_L1; end
                ST_TX_AN_L1: begin if (!tx_busy_r) begin tx_data <= "A";      tx_start <= 1'b1; state <= ST_W_AN_L1; end end
                ST_W_AN_L1:  begin if (tx_done) state <= ST_TX_AN_L2; end
                ST_TX_AN_L2: begin if (!tx_busy_r) begin tx_data <= "N";      tx_start <= 1'b1; state <= ST_W_AN_L2; end end
                ST_W_AN_L2:  begin if (tx_done) state <= ST_TX_AN_EQ; end
                ST_TX_AN_EQ: begin if (!tx_busy_r) begin tx_data <= "=";      tx_start <= 1'b1; state <= ST_W_AN_EQ; end end
                ST_W_AN_EQ:  begin if (tx_done) state <= ST_TX_AN_SG; end
                ST_TX_AN_SG: begin if (!tx_busy_r) begin tx_data <= an_sg;    tx_start <= 1'b1; state <= ST_W_AN_SG; end end
                ST_W_AN_SG:  begin if (tx_done) state <= ST_TX_AN_D4; end
                ST_TX_AN_D4: begin if (!tx_busy_r) begin tx_data <= d2a(an_d4); tx_start <= 1'b1; state <= ST_W_AN_D4; end end
                ST_W_AN_D4:  begin if (tx_done) state <= ST_TX_AN_D3; end
                ST_TX_AN_D3: begin if (!tx_busy_r) begin tx_data <= d2a(an_d3); tx_start <= 1'b1; state <= ST_W_AN_D3; end end
                ST_W_AN_D3:  begin if (tx_done) state <= ST_TX_AN_D2; end
                ST_TX_AN_D2: begin if (!tx_busy_r) begin tx_data <= d2a(an_d2); tx_start <= 1'b1; state <= ST_W_AN_D2; end end
                ST_W_AN_D2:  begin if (tx_done) state <= ST_TX_AN_D1; end
                ST_TX_AN_D1: begin if (!tx_busy_r) begin tx_data <= d2a(an_d1); tx_start <= 1'b1; state <= ST_W_AN_D1; end end
                ST_W_AN_D1:  begin if (tx_done) state <= ST_TX_AN_D0; end
                ST_TX_AN_D0: begin if (!tx_busy_r) begin tx_data <= d2a(an_d0); tx_start <= 1'b1; state <= ST_W_AN_D0; end end
                ST_W_AN_D0:  begin if (tx_done) state <= ST_TX_KP_SP; end

                // ------ KP ------
                ST_TX_KP_SP: begin if (!tx_busy_r) begin tx_data <= " ";        tx_start <= 1'b1; state <= ST_W_KP_SP; end end
                ST_W_KP_SP:  begin if (tx_done) state <= ST_TX_KP_L1; end
                ST_TX_KP_L1: begin if (!tx_busy_r) begin tx_data <= "K";        tx_start <= 1'b1; state <= ST_W_KP_L1; end end
                ST_W_KP_L1:  begin if (tx_done) state <= ST_TX_KP_L2; end
                ST_TX_KP_L2: begin if (!tx_busy_r) begin tx_data <= "P";        tx_start <= 1'b1; state <= ST_W_KP_L2; end end
                ST_W_KP_L2:  begin if (tx_done) state <= ST_TX_KP_EQ; end
                ST_TX_KP_EQ: begin if (!tx_busy_r) begin tx_data <= "=";        tx_start <= 1'b1; state <= ST_W_KP_EQ; end end
                ST_W_KP_EQ:  begin if (tx_done) state <= ST_TX_KP_D4; end
                ST_TX_KP_D4: begin if (!tx_busy_r) begin tx_data <= d2a(kp_d4); tx_start <= 1'b1; state <= ST_W_KP_D4; end end
                ST_W_KP_D4:  begin if (tx_done) state <= ST_TX_KP_D3; end
                ST_TX_KP_D3: begin if (!tx_busy_r) begin tx_data <= d2a(kp_d3); tx_start <= 1'b1; state <= ST_W_KP_D3; end end
                ST_W_KP_D3:  begin if (tx_done) state <= ST_TX_KP_D2; end
                ST_TX_KP_D2: begin if (!tx_busy_r) begin tx_data <= d2a(kp_d2); tx_start <= 1'b1; state <= ST_W_KP_D2; end end
                ST_W_KP_D2:  begin if (tx_done) state <= ST_TX_KP_D1; end
                ST_TX_KP_D1: begin if (!tx_busy_r) begin tx_data <= d2a(kp_d1); tx_start <= 1'b1; state <= ST_W_KP_D1; end end
                ST_W_KP_D1:  begin if (tx_done) state <= ST_TX_KP_D0; end
                ST_TX_KP_D0: begin if (!tx_busy_r) begin tx_data <= d2a(kp_d0); tx_start <= 1'b1; state <= ST_W_KP_D0; end end
                ST_W_KP_D0:  begin if (tx_done) state <= ST_TX_KI_SP; end

                // ------ KI ------
                ST_TX_KI_SP: begin if (!tx_busy_r) begin tx_data <= " ";        tx_start <= 1'b1; state <= ST_W_KI_SP; end end
                ST_W_KI_SP:  begin if (tx_done) state <= ST_TX_KI_L1; end
                ST_TX_KI_L1: begin if (!tx_busy_r) begin tx_data <= "K";        tx_start <= 1'b1; state <= ST_W_KI_L1; end end
                ST_W_KI_L1:  begin if (tx_done) state <= ST_TX_KI_L2; end
                ST_TX_KI_L2: begin if (!tx_busy_r) begin tx_data <= "I";        tx_start <= 1'b1; state <= ST_W_KI_L2; end end
                ST_W_KI_L2:  begin if (tx_done) state <= ST_TX_KI_EQ; end
                ST_TX_KI_EQ: begin if (!tx_busy_r) begin tx_data <= "=";        tx_start <= 1'b1; state <= ST_W_KI_EQ; end end
                ST_W_KI_EQ:  begin if (tx_done) state <= ST_TX_KI_D4; end
                ST_TX_KI_D4: begin if (!tx_busy_r) begin tx_data <= d2a(ki_d4); tx_start <= 1'b1; state <= ST_W_KI_D4; end end
                ST_W_KI_D4:  begin if (tx_done) state <= ST_TX_KI_D3; end
                ST_TX_KI_D3: begin if (!tx_busy_r) begin tx_data <= d2a(ki_d3); tx_start <= 1'b1; state <= ST_W_KI_D3; end end
                ST_W_KI_D3:  begin if (tx_done) state <= ST_TX_KI_D2; end
                ST_TX_KI_D2: begin if (!tx_busy_r) begin tx_data <= d2a(ki_d2); tx_start <= 1'b1; state <= ST_W_KI_D2; end end
                ST_W_KI_D2:  begin if (tx_done) state <= ST_TX_KI_D1; end
                ST_TX_KI_D1: begin if (!tx_busy_r) begin tx_data <= d2a(ki_d1); tx_start <= 1'b1; state <= ST_W_KI_D1; end end
                ST_W_KI_D1:  begin if (tx_done) state <= ST_TX_KI_D0; end
                ST_TX_KI_D0: begin if (!tx_busy_r) begin tx_data <= d2a(ki_d0); tx_start <= 1'b1; state <= ST_W_KI_D0; end end
                ST_W_KI_D0:  begin if (tx_done) state <= ST_TX_KD_SP; end

                // ------ KD ------
                ST_TX_KD_SP: begin if (!tx_busy_r) begin tx_data <= " ";        tx_start <= 1'b1; state <= ST_W_KD_SP; end end
                ST_W_KD_SP:  begin if (tx_done) state <= ST_TX_KD_L1; end
                ST_TX_KD_L1: begin if (!tx_busy_r) begin tx_data <= "K";        tx_start <= 1'b1; state <= ST_W_KD_L1; end end
                ST_W_KD_L1:  begin if (tx_done) state <= ST_TX_KD_L2; end
                ST_TX_KD_L2: begin if (!tx_busy_r) begin tx_data <= "D";        tx_start <= 1'b1; state <= ST_W_KD_L2; end end
                ST_W_KD_L2:  begin if (tx_done) state <= ST_TX_KD_EQ; end
                ST_TX_KD_EQ: begin if (!tx_busy_r) begin tx_data <= "=";        tx_start <= 1'b1; state <= ST_W_KD_EQ; end end
                ST_W_KD_EQ:  begin if (tx_done) state <= ST_TX_KD_D4; end
                ST_TX_KD_D4: begin if (!tx_busy_r) begin tx_data <= d2a(kd_d4); tx_start <= 1'b1; state <= ST_W_KD_D4; end end
                ST_W_KD_D4:  begin if (tx_done) state <= ST_TX_KD_D3; end
                ST_TX_KD_D3: begin if (!tx_busy_r) begin tx_data <= d2a(kd_d3); tx_start <= 1'b1; state <= ST_W_KD_D3; end end
                ST_W_KD_D3:  begin if (tx_done) state <= ST_TX_KD_D2; end
                ST_TX_KD_D2: begin if (!tx_busy_r) begin tx_data <= d2a(kd_d2); tx_start <= 1'b1; state <= ST_W_KD_D2; end end
                ST_W_KD_D2:  begin if (tx_done) state <= ST_TX_KD_D1; end
                ST_TX_KD_D1: begin if (!tx_busy_r) begin tx_data <= d2a(kd_d1); tx_start <= 1'b1; state <= ST_W_KD_D1; end end
                ST_W_KD_D1:  begin if (tx_done) state <= ST_TX_KD_D0; end
                ST_TX_KD_D0: begin if (!tx_busy_r) begin tx_data <= d2a(kd_d0); tx_start <= 1'b1; state <= ST_W_KD_D0; end end
                ST_W_KD_D0:  begin if (tx_done) state <= ST_TX_CR; end

                // ------ CR LF ------
                ST_TX_CR: begin if (!tx_busy_r) begin tx_data <= 8'h0D; tx_start <= 1'b1; state <= ST_W_CR; end end
                ST_W_CR:  begin if (tx_done) state <= ST_TX_LF; end
                ST_TX_LF: begin if (!tx_busy_r) begin tx_data <= 8'h0A; tx_start <= 1'b1; state <= ST_W_LF; end end
                ST_W_LF:  begin if (tx_done) state <= ST_IDLE; end

                default: state <= ST_IDLE;

            endcase
        end
    end

endmodule
