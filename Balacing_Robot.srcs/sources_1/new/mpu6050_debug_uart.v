
`timescale 1ns / 1ps
//==============================================================================
// mpu6050_debug_uart
// - MPU6050 6축 + 각도 + PID 이득값 + 엔코더 데이터 UART 출력
// - 출력 형식:
//   AX=+DDDDD AY=+DDDDD AZ=+DDDDD GX=+DDDDD GY=+DDDDD GZ=+DDDDD AN=+DDDDD
//   KP=DDDDD KI=DDDDD KD=DDDDD SP=+DDDDD VA=+DDDDD VB=+DDDDD EA=+DDDDD EB=+DDDDD\r\n

//   KP=DDDDD KI=DDDDD KD=DDDDD SP=+DDDDD OF=+DDDDD VA=+DDDDD VB=+DDDDD EA=+DDDDD EB=+DDDDD\r\n
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

    input  wire signed [15:0] angle,
    input  wire        [15:0] kp,
    input  wire        [15:0] ki,
    input  wire        [15:0] kd,
    input  wire signed [15:0] setpoint,
    //추가
    input  wire signed [15:0] angle_offset,



    // 엔코더 데이터
    input  wire signed [15:0] vel_a,
    input  wire signed [15:0] vel_b,
    input  wire signed [31:0] pos_a,
    input  wire signed [31:0] pos_b,

    output wire               uart_tx_o
);

    //--------------------------------------------------------------------------
    // 상태 정의 (9-bit, 0~511)
    //--------------------------------------------------------------------------
    localparam [8:0]
        ST_IDLE    = 9'd0,
        ST_CAPTURE = 9'd1,

        ST_CALC_10000 = 9'd200,
        ST_CALC_1000  = 9'd201,
        ST_CALC_100   = 9'd202,
        ST_CALC_10    = 9'd203,
        ST_CALC_1     = 9'd204,

        // AX (3~22, 부호 있음, 뒤에 space)
        ST_TX_AX_L1 = 9'd3,   ST_W_AX_L1 = 9'd4,
        ST_TX_AX_L2 = 9'd5,   ST_W_AX_L2 = 9'd6,
        ST_TX_AX_EQ = 9'd7,   ST_W_AX_EQ = 9'd8,
        ST_TX_AX_SG = 9'd9,   ST_W_AX_SG = 9'd10,
        ST_TX_AX_D4 = 9'd11,  ST_W_AX_D4 = 9'd12,
        ST_TX_AX_D3 = 9'd13,  ST_W_AX_D3 = 9'd14,
        ST_TX_AX_D2 = 9'd15,  ST_W_AX_D2 = 9'd16,
        ST_TX_AX_D1 = 9'd17,  ST_W_AX_D1 = 9'd18,
        ST_TX_AX_D0 = 9'd19,  ST_W_AX_D0 = 9'd20,
        ST_TX_AX_SP = 9'd21,  ST_W_AX_SP = 9'd22,

        // AY (23~42)
        ST_TX_AY_L1 = 9'd23,  ST_W_AY_L1 = 9'd24,
        ST_TX_AY_L2 = 9'd25,  ST_W_AY_L2 = 9'd26,
        ST_TX_AY_EQ = 9'd27,  ST_W_AY_EQ = 9'd28,
        ST_TX_AY_SG = 9'd29,  ST_W_AY_SG = 9'd30,
        ST_TX_AY_D4 = 9'd31,  ST_W_AY_D4 = 9'd32,
        ST_TX_AY_D3 = 9'd33,  ST_W_AY_D3 = 9'd34,
        ST_TX_AY_D2 = 9'd35,  ST_W_AY_D2 = 9'd36,
        ST_TX_AY_D1 = 9'd37,  ST_W_AY_D1 = 9'd38,
        ST_TX_AY_D0 = 9'd39,  ST_W_AY_D0 = 9'd40,
        ST_TX_AY_SP = 9'd41,  ST_W_AY_SP = 9'd42,

        // AZ (43~62)
        ST_TX_AZ_L1 = 9'd43,  ST_W_AZ_L1 = 9'd44,
        ST_TX_AZ_L2 = 9'd45,  ST_W_AZ_L2 = 9'd46,
        ST_TX_AZ_EQ = 9'd47,  ST_W_AZ_EQ = 9'd48,
        ST_TX_AZ_SG = 9'd49,  ST_W_AZ_SG = 9'd50,
        ST_TX_AZ_D4 = 9'd51,  ST_W_AZ_D4 = 9'd52,
        ST_TX_AZ_D3 = 9'd53,  ST_W_AZ_D3 = 9'd54,
        ST_TX_AZ_D2 = 9'd55,  ST_W_AZ_D2 = 9'd56,
        ST_TX_AZ_D1 = 9'd57,  ST_W_AZ_D1 = 9'd58,
        ST_TX_AZ_D0 = 9'd59,  ST_W_AZ_D0 = 9'd60,
        ST_TX_AZ_SP = 9'd61,  ST_W_AZ_SP = 9'd62,

        // GX (63~82)
        ST_TX_GX_L1 = 9'd63,  ST_W_GX_L1 = 9'd64,
        ST_TX_GX_L2 = 9'd65,  ST_W_GX_L2 = 9'd66,
        ST_TX_GX_EQ = 9'd67,  ST_W_GX_EQ = 9'd68,
        ST_TX_GX_SG = 9'd69,  ST_W_GX_SG = 9'd70,
        ST_TX_GX_D4 = 9'd71,  ST_W_GX_D4 = 9'd72,
        ST_TX_GX_D3 = 9'd73,  ST_W_GX_D3 = 9'd74,
        ST_TX_GX_D2 = 9'd75,  ST_W_GX_D2 = 9'd76,
        ST_TX_GX_D1 = 9'd77,  ST_W_GX_D1 = 9'd78,
        ST_TX_GX_D0 = 9'd79,  ST_W_GX_D0 = 9'd80,
        ST_TX_GX_SP = 9'd81,  ST_W_GX_SP = 9'd82,

        // GY (83~102)
        ST_TX_GY_L1 = 9'd83,  ST_W_GY_L1 = 9'd84,
        ST_TX_GY_L2 = 9'd85,  ST_W_GY_L2 = 9'd86,
        ST_TX_GY_EQ = 9'd87,  ST_W_GY_EQ = 9'd88,
        ST_TX_GY_SG = 9'd89,  ST_W_GY_SG = 9'd90,
        ST_TX_GY_D4 = 9'd91,  ST_W_GY_D4 = 9'd92,
        ST_TX_GY_D3 = 9'd93,  ST_W_GY_D3 = 9'd94,
        ST_TX_GY_D2 = 9'd95,  ST_W_GY_D2 = 9'd96,
        ST_TX_GY_D1 = 9'd97,  ST_W_GY_D1 = 9'd98,
        ST_TX_GY_D0 = 9'd99,  ST_W_GY_D0 = 9'd100,
        ST_TX_GY_SP = 9'd101, ST_W_GY_SP = 9'd102,

        // GZ (103~120, space 없음)
        ST_TX_GZ_L1 = 9'd103, ST_W_GZ_L1 = 9'd104,
        ST_TX_GZ_L2 = 9'd105, ST_W_GZ_L2 = 9'd106,
        ST_TX_GZ_EQ = 9'd107, ST_W_GZ_EQ = 9'd108,
        ST_TX_GZ_SG = 9'd109, ST_W_GZ_SG = 9'd110,
        ST_TX_GZ_D4 = 9'd111, ST_W_GZ_D4 = 9'd112,
        ST_TX_GZ_D3 = 9'd113, ST_W_GZ_D3 = 9'd114,
        ST_TX_GZ_D2 = 9'd115, ST_W_GZ_D2 = 9'd116,
        ST_TX_GZ_D1 = 9'd117, ST_W_GZ_D1 = 9'd118,
        ST_TX_GZ_D0 = 9'd119, ST_W_GZ_D0 = 9'd120,

        // CR LF
        ST_TX_CR = 9'd121, ST_W_CR = 9'd122,
        ST_TX_LF = 9'd123, ST_W_LF = 9'd124,

        // AN (125~144, 앞에 space)
        ST_TX_AN_SP = 9'd125, ST_W_AN_SP = 9'd126,
        ST_TX_AN_L1 = 9'd127, ST_W_AN_L1 = 9'd128,
        ST_TX_AN_L2 = 9'd129, ST_W_AN_L2 = 9'd130,
        ST_TX_AN_EQ = 9'd131, ST_W_AN_EQ = 9'd132,
        ST_TX_AN_SG = 9'd133, ST_W_AN_SG = 9'd134,
        ST_TX_AN_D4 = 9'd135, ST_W_AN_D4 = 9'd136,
        ST_TX_AN_D3 = 9'd137, ST_W_AN_D3 = 9'd138,
        ST_TX_AN_D2 = 9'd139, ST_W_AN_D2 = 9'd140,
        ST_TX_AN_D1 = 9'd141, ST_W_AN_D1 = 9'd142,
        ST_TX_AN_D0 = 9'd143, ST_W_AN_D0 = 9'd144,

        // KP (145~162, 앞에 space, 부호 없음)
        ST_TX_KP_SP = 9'd145, ST_W_KP_SP = 9'd146,
        ST_TX_KP_L1 = 9'd147, ST_W_KP_L1 = 9'd148,
        ST_TX_KP_L2 = 9'd149, ST_W_KP_L2 = 9'd150,
        ST_TX_KP_EQ = 9'd151, ST_W_KP_EQ = 9'd152,
        ST_TX_KP_D4 = 9'd153, ST_W_KP_D4 = 9'd154,
        ST_TX_KP_D3 = 9'd155, ST_W_KP_D3 = 9'd156,
        ST_TX_KP_D2 = 9'd157, ST_W_KP_D2 = 9'd158,
        ST_TX_KP_D1 = 9'd159, ST_W_KP_D1 = 9'd160,
        ST_TX_KP_D0 = 9'd161, ST_W_KP_D0 = 9'd162,

        // KI (163~180)
        ST_TX_KI_SP = 9'd163, ST_W_KI_SP = 9'd164,
        ST_TX_KI_L1 = 9'd165, ST_W_KI_L1 = 9'd166,
        ST_TX_KI_L2 = 9'd167, ST_W_KI_L2 = 9'd168,
        ST_TX_KI_EQ = 9'd169, ST_W_KI_EQ = 9'd170,
        ST_TX_KI_D4 = 9'd171, ST_W_KI_D4 = 9'd172,
        ST_TX_KI_D3 = 9'd173, ST_W_KI_D3 = 9'd174,
        ST_TX_KI_D2 = 9'd175, ST_W_KI_D2 = 9'd176,
        ST_TX_KI_D1 = 9'd177, ST_W_KI_D1 = 9'd178,
        ST_TX_KI_D0 = 9'd179, ST_W_KI_D0 = 9'd180,

        // KD (181~198)
        ST_TX_KD_SP = 9'd181, ST_W_KD_SP = 9'd182,
        ST_TX_KD_L1 = 9'd183, ST_W_KD_L1 = 9'd184,
        ST_TX_KD_L2 = 9'd185, ST_W_KD_L2 = 9'd186,
        ST_TX_KD_EQ = 9'd187, ST_W_KD_EQ = 9'd188,
        ST_TX_KD_D4 = 9'd189, ST_W_KD_D4 = 9'd190,
        ST_TX_KD_D3 = 9'd191, ST_W_KD_D3 = 9'd192,
        ST_TX_KD_D2 = 9'd193, ST_W_KD_D2 = 9'd194,
        ST_TX_KD_D1 = 9'd195, ST_W_KD_D1 = 9'd196,
        ST_TX_KD_D0 = 9'd197, ST_W_KD_D0 = 9'd198,

        // SP (205~224, 앞에 space, 부호 있음)
        ST_TX_SP_SP = 9'd205, ST_W_SP_SP = 9'd206,
        ST_TX_SP_L1 = 9'd207, ST_W_SP_L1 = 9'd208,
        ST_TX_SP_L2 = 9'd209, ST_W_SP_L2 = 9'd210,
        ST_TX_SP_EQ = 9'd211, ST_W_SP_EQ = 9'd212,
        ST_TX_SP_SG = 9'd213, ST_W_SP_SG = 9'd214,
        ST_TX_SP_D4 = 9'd215, ST_W_SP_D4 = 9'd216,
        ST_TX_SP_D3 = 9'd217, ST_W_SP_D3 = 9'd218,
        ST_TX_SP_D2 = 9'd219, ST_W_SP_D2 = 9'd220,
        ST_TX_SP_D1 = 9'd221, ST_W_SP_D1 = 9'd222,
        ST_TX_SP_D0 = 9'd223, ST_W_SP_D0 = 9'd224,

        // VA (225~244, 앞에 space, 부호 있음)
        ST_TX_VA_SP = 9'd225, ST_W_VA_SP = 9'd226,
        ST_TX_VA_L1 = 9'd227, ST_W_VA_L1 = 9'd228,
        ST_TX_VA_L2 = 9'd229, ST_W_VA_L2 = 9'd230,
        ST_TX_VA_EQ = 9'd231, ST_W_VA_EQ = 9'd232,
        ST_TX_VA_SG = 9'd233, ST_W_VA_SG = 9'd234,
        ST_TX_VA_D4 = 9'd235, ST_W_VA_D4 = 9'd236,
        ST_TX_VA_D3 = 9'd237, ST_W_VA_D3 = 9'd238,
        ST_TX_VA_D2 = 9'd239, ST_W_VA_D2 = 9'd240,
        ST_TX_VA_D1 = 9'd241, ST_W_VA_D1 = 9'd242,
        ST_TX_VA_D0 = 9'd243, ST_W_VA_D0 = 9'd244,

        // VB (245~264)
        ST_TX_VB_SP = 9'd245, ST_W_VB_SP = 9'd246,
        ST_TX_VB_L1 = 9'd247, ST_W_VB_L1 = 9'd248,
        ST_TX_VB_L2 = 9'd249, ST_W_VB_L2 = 9'd250,
        ST_TX_VB_EQ = 9'd251, ST_W_VB_EQ = 9'd252,
        ST_TX_VB_SG = 9'd253, ST_W_VB_SG = 9'd254,
        ST_TX_VB_D4 = 9'd255, ST_W_VB_D4 = 9'd256,
        ST_TX_VB_D3 = 9'd257, ST_W_VB_D3 = 9'd258,
        ST_TX_VB_D2 = 9'd259, ST_W_VB_D2 = 9'd260,
        ST_TX_VB_D1 = 9'd261, ST_W_VB_D1 = 9'd262,
        ST_TX_VB_D0 = 9'd263, ST_W_VB_D0 = 9'd264,

        // EA (265~284, 엔코더 위치 A, 부호 있음)
        ST_TX_EA_SP = 9'd265, ST_W_EA_SP = 9'd266,
        ST_TX_EA_L1 = 9'd267, ST_W_EA_L1 = 9'd268,
        ST_TX_EA_L2 = 9'd269, ST_W_EA_L2 = 9'd270,
        ST_TX_EA_EQ = 9'd271, ST_W_EA_EQ = 9'd272,
        ST_TX_EA_SG = 9'd273, ST_W_EA_SG = 9'd274,
        ST_TX_EA_D4 = 9'd275, ST_W_EA_D4 = 9'd276,
        ST_TX_EA_D3 = 9'd277, ST_W_EA_D3 = 9'd278,
        ST_TX_EA_D2 = 9'd279, ST_W_EA_D2 = 9'd280,
        ST_TX_EA_D1 = 9'd281, ST_W_EA_D1 = 9'd282,
        ST_TX_EA_D0 = 9'd283, ST_W_EA_D0 = 9'd284,

        // EB (285~304, 엔코더 위치 B, 부호 있음)
        ST_TX_EB_SP = 9'd285, ST_W_EB_SP = 9'd286,
        ST_TX_EB_L1 = 9'd287, ST_W_EB_L1 = 9'd288,
        ST_TX_EB_L2 = 9'd289, ST_W_EB_L2 = 9'd290,
        ST_TX_EB_EQ = 9'd291, ST_W_EB_EQ = 9'd292,
        ST_TX_EB_SG = 9'd293, ST_W_EB_SG = 9'd294,
        ST_TX_EB_D4 = 9'd295, ST_W_EB_D4 = 9'd296,
        ST_TX_EB_D3 = 9'd297, ST_W_EB_D3 = 9'd298,
        ST_TX_EB_D2 = 9'd299, ST_W_EB_D2 = 9'd300,
        ST_TX_EB_D1 = 9'd301, ST_W_EB_D1 = 9'd302,
        ST_TX_EB_D0 = 9'd303, ST_W_EB_D0 = 9'd304,


        // 추가
        // OF (245~264, 앞에 space, 부호 있음)
        ST_TX_OF_SP = 9'd305, ST_W_OF_SP = 9'd306,
        ST_TX_OF_L1 = 9'd307, ST_W_OF_L1 = 9'd308,
        ST_TX_OF_L2 = 9'd309, ST_W_OF_L2 = 9'd310,
        ST_TX_OF_EQ = 9'd311, ST_W_OF_EQ = 9'd312,
        ST_TX_OF_SG = 9'd313, ST_W_OF_SG = 9'd314,
        ST_TX_OF_D4 = 9'd315, ST_W_OF_D4 = 9'd316,
        ST_TX_OF_D3 = 9'd317, ST_W_OF_D3 = 9'd318,
        ST_TX_OF_D2 = 9'd319, ST_W_OF_D2 = 9'd320,
        ST_TX_OF_D1 = 9'd321, ST_W_OF_D1 = 9'd322,
        ST_TX_OF_D0 = 9'd323, ST_W_OF_D0 = 9'd324;

    reg [8:0] state;

    //--------------------------------------------------------------------------
    // UART 송신기
    //--------------------------------------------------------------------------
    reg       tx_start;
    reg [7:0] tx_data;
    reg       tx_busy_r;
    wire      tx_done;
    wire      b_tick;

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

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)        tx_busy_r <= 1'b0;
        else if (tx_start) tx_busy_r <= 1'b1;
        else if (tx_done)  tx_busy_r <= 1'b0;
    end

    //--------------------------------------------------------------------------
    // 0.5초 print tick
    //--------------------------------------------------------------------------
    localparam [31:0] PRINT_CNT_MAX = 32'd50_000_000;
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
    // sample_ready
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
    // 십진수 변환 결과
    //--------------------------------------------------------------------------
    reg [7:0] ax_sg; reg [3:0] ax_d4, ax_d3, ax_d2, ax_d1, ax_d0;
    reg [7:0] ay_sg; reg [3:0] ay_d4, ay_d3, ay_d2, ay_d1, ay_d0;
    reg [7:0] az_sg; reg [3:0] az_d4, az_d3, az_d2, az_d1, az_d0;
    reg [7:0] gx_sg; reg [3:0] gx_d4, gx_d3, gx_d2, gx_d1, gx_d0;
    reg [7:0] gy_sg; reg [3:0] gy_d4, gy_d3, gy_d2, gy_d1, gy_d0;
    reg [7:0] gz_sg; reg [3:0] gz_d4, gz_d3, gz_d2, gz_d1, gz_d0;
    reg [7:0] an_sg; reg [3:0] an_d4, an_d3, an_d2, an_d1, an_d0;
    reg [3:0] kp_d4, kp_d3, kp_d2, kp_d1, kp_d0;
    reg [3:0] ki_d4, ki_d3, ki_d2, ki_d1, ki_d0;
    reg [3:0] kd_d4, kd_d3, kd_d2, kd_d1, kd_d0;
    reg [7:0] sp_sg; reg [3:0] sp_d4, sp_d3, sp_d2, sp_d1, sp_d0;
    // 추가
    reg [7:0] of_sg; reg [3:0] of_d4, of_d3, of_d2, of_d1, of_d0;

    // 엔코더
    reg [7:0] va_sg; reg [3:0] va_d4, va_d3, va_d2, va_d1, va_d0;
    reg [7:0] vb_sg; reg [3:0] vb_d4, vb_d3, vb_d2, vb_d1, vb_d0;
    reg [7:0] ea_sg; reg [3:0] ea_d4, ea_d3, ea_d2, ea_d1, ea_d0;
    reg [7:0] eb_sg; reg [3:0] eb_d4, eb_d3, eb_d2, eb_d1, eb_d0;

    // 반복 뺄셈용 임시 변수
    reg [15:0] t_ax, t_ay, t_az, t_gx, t_gy, t_gz, t_an;
    reg [15:0] t_kp, t_ki, t_kd, t_sp, t_of;
    reg [15:0] t_va, t_vb;
    reg [16:0] t_ea, t_eb;  // 17-bit: 최대 99999 저장

    // 엔코더 위치 절댓값 (조합 논리)
    wire [31:0] pos_a_abs = pos_a[31] ? (~pos_a + 32'd1) : pos_a;
    wire [31:0] pos_b_abs = pos_b[31] ? (~pos_b + 32'd1) : pos_b;

    //--------------------------------------------------------------------------
    // digit → ASCII
    //--------------------------------------------------------------------------
    function [7:0] d2a;
        input [3:0] d;
        begin
            d2a = {4'b0011, d};
        end
    endfunction

    //--------------------------------------------------------------------------
    // UART 출력 FSM
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_start <= 1'b0;
            tx_data  <= 8'd0;

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
            sp_sg <= "+"; sp_d4 <= 0; sp_d3 <= 0; sp_d2 <= 0; sp_d1 <= 0; sp_d0 <= 0;
            va_sg <= "+"; va_d4 <= 0; va_d3 <= 0; va_d2 <= 0; va_d1 <= 0; va_d0 <= 0;
            vb_sg <= "+"; vb_d4 <= 0; vb_d3 <= 0; vb_d2 <= 0; vb_d1 <= 0; vb_d0 <= 0;
            ea_sg <= "+"; ea_d4 <= 0; ea_d3 <= 0; ea_d2 <= 0; ea_d1 <= 0; ea_d0 <= 0;
            eb_sg <= "+"; eb_d4 <= 0; eb_d3 <= 0; eb_d2 <= 0; eb_d1 <= 0; eb_d0 <= 0;
            // 추가
            of_sg <= "+"; of_d4 <= 0; of_d3 <= 0; of_d2 <= 0; of_d1 <= 0; of_d0 <= 0;


            t_ax <= 0; t_ay <= 0; t_az <= 0; t_gx <= 0; t_gy <= 0; t_gz <= 0; t_an <= 0;
            t_kp <= 0; t_ki <= 0; t_kd <= 0; t_sp <= 0; t_of <= 0;// 추가
            t_va <= 0; t_vb <= 0; t_ea <= 0; t_eb <= 0;

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
                    // 부호 판별
                    ax_sg <= accel_x[15] ? "-" : "+";
                    ay_sg <= accel_y[15] ? "-" : "+";
                    az_sg <= accel_z[15] ? "-" : "+";
                    gx_sg <= gyro_x[15]  ? "-" : "+";
                    gy_sg <= gyro_y[15]  ? "-" : "+";
                    gz_sg <= gyro_z[15]  ? "-" : "+";
                    an_sg <= angle[15]   ? "-" : "+";
                    sp_sg <= setpoint[15] ? "-" : "+";
                    va_sg <= vel_a[15]   ? "-" : "+";
                    vb_sg <= vel_b[15]   ? "-" : "+";
                    ea_sg <= pos_a[31]   ? "-" : "+";
                    eb_sg <= pos_b[31]   ? "-" : "+";
                    //추가
                    of_sg <= angle_offset[15] ? "-" : "+";

                    // 절댓값 저장
                    t_ax <= accel_x[15] ? (~accel_x + 1'b1) : accel_x;
                    t_ay <= accel_y[15] ? (~accel_y + 1'b1) : accel_y;
                    t_az <= accel_z[15] ? (~accel_z + 1'b1) : accel_z;
                    t_gx <= gyro_x[15]  ? (~gyro_x  + 1'b1) : gyro_x;
                    t_gy <= gyro_y[15]  ? (~gyro_y  + 1'b1) : gyro_y;
                    t_gz <= gyro_z[15]  ? (~gyro_z  + 1'b1) : gyro_z;
                    t_an <= angle[15]   ? (~angle   + 1'b1) : angle;
                    t_kp <= kp;
                    t_ki <= ki;
                    t_kd <= kd;
                    t_sp <= setpoint[15] ? (~setpoint + 1'b1) : setpoint;
                    // 추가
                    t_of <= angle_offset[15] ? (~angle_offset + 1'b1) : angle_offset;

                    t_va <= vel_a[15]   ? (~vel_a   + 1'b1) : vel_a;
                    t_vb <= vel_b[15]   ? (~vel_b   + 1'b1) : vel_b;
                    // 위치: 32비트 절댓값, 99999 상한 클램핑
                    t_ea <= (pos_a_abs > 32'd99999) ? 17'd99999 : pos_a_abs[16:0];
                    t_eb <= (pos_b_abs > 32'd99999) ? 17'd99999 : pos_b_abs[16:0];

                    // 자릿수 초기화
                    ax_d4<=0; ax_d3<=0; ax_d2<=0; ax_d1<=0; ax_d0<=0;
                    ay_d4<=0; ay_d3<=0; ay_d2<=0; ay_d1<=0; ay_d0<=0;
                    az_d4<=0; az_d3<=0; az_d2<=0; az_d1<=0; az_d0<=0;
                    gx_d4<=0; gx_d3<=0; gx_d2<=0; gx_d1<=0; gx_d0<=0;
                    gy_d4<=0; gy_d3<=0; gy_d2<=0; gy_d1<=0; gy_d0<=0;
                    gz_d4<=0; gz_d3<=0; gz_d2<=0; gz_d1<=0; gz_d0<=0;
                    an_d4<=0; an_d3<=0; an_d2<=0; an_d1<=0; an_d0<=0;
                    kp_d4<=0; kp_d3<=0; kp_d2<=0; kp_d1<=0; kp_d0<=0;
                    ki_d4<=0; ki_d3<=0; ki_d2<=0; ki_d1<=0; ki_d0<=0;
                    kd_d4<=0; kd_d3<=0; kd_d2<=0; kd_d1<=0; kd_d0<=0;
                    sp_d4<=0; sp_d3<=0; sp_d2<=0; sp_d1<=0; sp_d0<=0;
                    // 추가
                    of_d4<=0; of_d3<=0; of_d2<=0; of_d1<=0; of_d0<=0;

                    va_d4<=0; va_d3<=0; va_d2<=0; va_d1<=0; va_d0<=0;
                    vb_d4<=0; vb_d3<=0; vb_d2<=0; vb_d1<=0; vb_d0<=0;
                    ea_d4<=0; ea_d3<=0; ea_d2<=0; ea_d1<=0; ea_d0<=0;
                    eb_d4<=0; eb_d3<=0; eb_d2<=0; eb_d1<=0; eb_d0<=0;

                    state <= ST_CALC_10000;
                end

                ST_CALC_10000: begin
                    if (t_ax>=10000||t_ay>=10000||t_az>=10000||t_gx>=10000||t_gy>=10000||t_gz>=10000||
                        t_an>=10000||t_kp>=10000||t_ki>=10000||t_kd>=10000||t_sp>=10000||t_of>=10000||
                        t_va>=10000||t_vb>=10000||t_ea>=10000||t_eb>=10000) begin
                        if (t_ax>=10000) begin t_ax<=t_ax-10000; ax_d4<=ax_d4+4'd1; end
                        if (t_ay>=10000) begin t_ay<=t_ay-10000; ay_d4<=ay_d4+4'd1; end
                        if (t_az>=10000) begin t_az<=t_az-10000; az_d4<=az_d4+4'd1; end
                        if (t_gx>=10000) begin t_gx<=t_gx-10000; gx_d4<=gx_d4+4'd1; end
                        if (t_gy>=10000) begin t_gy<=t_gy-10000; gy_d4<=gy_d4+4'd1; end
                        if (t_gz>=10000) begin t_gz<=t_gz-10000; gz_d4<=gz_d4+4'd1; end
                        if (t_an>=10000) begin t_an<=t_an-10000; an_d4<=an_d4+4'd1; end
                        if (t_kp>=10000) begin t_kp<=t_kp-10000; kp_d4<=kp_d4+4'd1; end
                        if (t_ki>=10000) begin t_ki<=t_ki-10000; ki_d4<=ki_d4+4'd1; end
                        if (t_kd>=10000) begin t_kd<=t_kd-10000; kd_d4<=kd_d4+4'd1; end
                        if (t_sp>=10000) begin t_sp<=t_sp-10000; sp_d4<=sp_d4+4'd1; end
                        // 추가
                        if (t_of>=10000) begin t_of<=t_of-10000; of_d4<=of_d4+4'd1; end
                        if (t_va>=10000) begin t_va<=t_va-10000; va_d4<=va_d4+4'd1; end
                        if (t_vb>=10000) begin t_vb<=t_vb-10000; vb_d4<=vb_d4+4'd1; end
                        if (t_ea>=10000) begin t_ea<=t_ea-10000; ea_d4<=ea_d4+4'd1; end
                        if (t_eb>=10000) begin t_eb<=t_eb-10000; eb_d4<=eb_d4+4'd1; end
                    end else state <= ST_CALC_1000;
                end

                ST_CALC_1000: begin
                    if (t_ax>=1000||t_ay>=1000||t_az>=1000||t_gx>=1000||t_gy>=1000||t_gz>=1000||
                        t_an>=1000||t_kp>=1000||t_ki>=1000||t_kd>=1000||t_sp>=1000||t_of>=1000||
                        t_va>=1000||t_vb>=1000||t_ea>=1000||t_eb>=1000) begin
                        if (t_ax>=1000) begin t_ax<=t_ax-1000; ax_d3<=ax_d3+4'd1; end
                        if (t_ay>=1000) begin t_ay<=t_ay-1000; ay_d3<=ay_d3+4'd1; end
                        if (t_az>=1000) begin t_az<=t_az-1000; az_d3<=az_d3+4'd1; end
                        if (t_gx>=1000) begin t_gx<=t_gx-1000; gx_d3<=gx_d3+4'd1; end
                        if (t_gy>=1000) begin t_gy<=t_gy-1000; gy_d3<=gy_d3+4'd1; end
                        if (t_gz>=1000) begin t_gz<=t_gz-1000; gz_d3<=gz_d3+4'd1; end
                        if (t_an>=1000) begin t_an<=t_an-1000; an_d3<=an_d3+4'd1; end
                        if (t_kp>=1000) begin t_kp<=t_kp-1000; kp_d3<=kp_d3+4'd1; end
                        if (t_ki>=1000) begin t_ki<=t_ki-1000; ki_d3<=ki_d3+4'd1; end
                        if (t_kd>=1000) begin t_kd<=t_kd-1000; kd_d3<=kd_d3+4'd1; end
                        if (t_sp>=1000) begin t_sp<=t_sp-1000; sp_d3<=sp_d3+4'd1; end
                        // 추가
                        if (t_of>=1000) begin t_of<=t_of-1000; of_d3<=of_d3+4'd1; end

                        if (t_va>=1000) begin t_va<=t_va-1000; va_d3<=va_d3+4'd1; end
                        if (t_vb>=1000) begin t_vb<=t_vb-1000; vb_d3<=vb_d3+4'd1; end
                        if (t_ea>=1000) begin t_ea<=t_ea-1000; ea_d3<=ea_d3+4'd1; end
                        if (t_eb>=1000) begin t_eb<=t_eb-1000; eb_d3<=eb_d3+4'd1; end
                    end else state <= ST_CALC_100;
                end

                ST_CALC_100: begin
                    if (t_ax>=100||t_ay>=100||t_az>=100||t_gx>=100||t_gy>=100||t_gz>=100||
                        t_an>=100||t_kp>=100||t_ki>=100||t_kd>=100||t_sp>=100||t_of>=100||
                        t_va>=100||t_vb>=100||t_ea>=100||t_eb>=100) begin
                        if (t_ax>=100) begin t_ax<=t_ax-100; ax_d2<=ax_d2+4'd1; end
                        if (t_ay>=100) begin t_ay<=t_ay-100; ay_d2<=ay_d2+4'd1; end
                        if (t_az>=100) begin t_az<=t_az-100; az_d2<=az_d2+4'd1; end
                        if (t_gx>=100) begin t_gx<=t_gx-100; gx_d2<=gx_d2+4'd1; end
                        if (t_gy>=100) begin t_gy<=t_gy-100; gy_d2<=gy_d2+4'd1; end
                        if (t_gz>=100) begin t_gz<=t_gz-100; gz_d2<=gz_d2+4'd1; end
                        if (t_an>=100) begin t_an<=t_an-100; an_d2<=an_d2+4'd1; end
                        if (t_kp>=100) begin t_kp<=t_kp-100; kp_d2<=kp_d2+4'd1; end
                        if (t_ki>=100) begin t_ki<=t_ki-100; ki_d2<=ki_d2+4'd1; end
                        if (t_kd>=100) begin t_kd<=t_kd-100; kd_d2<=kd_d2+4'd1; end
                        if (t_sp>=100) begin t_sp<=t_sp-100; sp_d2<=sp_d2+4'd1; end
                        // 추가
                        if (t_of>=100) begin t_of<=t_of-100; of_d2<=of_d2+4'd1; end

                        if (t_va>=100) begin t_va<=t_va-100; va_d2<=va_d2+4'd1; end
                        if (t_vb>=100) begin t_vb<=t_vb-100; vb_d2<=vb_d2+4'd1; end
                        if (t_ea>=100) begin t_ea<=t_ea-100; ea_d2<=ea_d2+4'd1; end
                        if (t_eb>=100) begin t_eb<=t_eb-100; eb_d2<=eb_d2+4'd1; end
                    end else state <= ST_CALC_10;
                end

                ST_CALC_10: begin
                    if (t_ax>=10||t_ay>=10||t_az>=10||t_gx>=10||t_gy>=10||t_gz>=10||
                        t_an>=10||t_kp>=10||t_ki>=10||t_kd>=10||t_sp>=10||t_of>=10||
                        t_va>=10||t_vb>=10||t_ea>=10||t_eb>=10) begin
                        if (t_ax>=10) begin t_ax<=t_ax-10; ax_d1<=ax_d1+4'd1; end
                        if (t_ay>=10) begin t_ay<=t_ay-10; ay_d1<=ay_d1+4'd1; end
                        if (t_az>=10) begin t_az<=t_az-10; az_d1<=az_d1+4'd1; end
                        if (t_gx>=10) begin t_gx<=t_gx-10; gx_d1<=gx_d1+4'd1; end
                        if (t_gy>=10) begin t_gy<=t_gy-10; gy_d1<=gy_d1+4'd1; end
                        if (t_gz>=10) begin t_gz<=t_gz-10; gz_d1<=gz_d1+4'd1; end
                        if (t_an>=10) begin t_an<=t_an-10; an_d1<=an_d1+4'd1; end
                        if (t_kp>=10) begin t_kp<=t_kp-10; kp_d1<=kp_d1+4'd1; end
                        if (t_ki>=10) begin t_ki<=t_ki-10; ki_d1<=ki_d1+4'd1; end
                        if (t_kd>=10) begin t_kd<=t_kd-10; kd_d1<=kd_d1+4'd1; end
                        if (t_sp>=10) begin t_sp<=t_sp-10; sp_d1<=sp_d1+4'd1; end
                        // 추가
                        if (t_of>=10) begin t_of<=t_of-10; of_d1<=of_d1+4'd1; end

                        if (t_va>=10) begin t_va<=t_va-10; va_d1<=va_d1+4'd1; end
                        if (t_vb>=10) begin t_vb<=t_vb-10; vb_d1<=vb_d1+4'd1; end
                        if (t_ea>=10) begin t_ea<=t_ea-10; ea_d1<=ea_d1+4'd1; end
                        if (t_eb>=10) begin t_eb<=t_eb-10; eb_d1<=eb_d1+4'd1; end
                    end else state <= ST_CALC_1;
                end

                ST_CALC_1: begin
                    ax_d0<=t_ax[3:0]; ay_d0<=t_ay[3:0]; az_d0<=t_az[3:0];
                    gx_d0<=t_gx[3:0]; gy_d0<=t_gy[3:0]; gz_d0<=t_gz[3:0];
                    an_d0<=t_an[3:0];
                    kp_d0<=t_kp[3:0]; ki_d0<=t_ki[3:0]; kd_d0<=t_kd[3:0];
                    sp_d0<=t_sp[3:0];
                    // 추가
                    of_d0<=t_of[3:0];
                    
                    va_d0<=t_va[3:0]; vb_d0<=t_vb[3:0];
                    ea_d0<=t_ea[3:0]; eb_d0<=t_eb[3:0];
                    state <= ST_TX_AX_L1;
                end

                // ------ AX ------
                ST_TX_AX_L1: begin if (!tx_busy_r) begin tx_data<="A"; tx_start<=1'b1; state<=ST_W_AX_L1; end end
                ST_W_AX_L1:  begin if (tx_done) state<=ST_TX_AX_L2; end
                ST_TX_AX_L2: begin if (!tx_busy_r) begin tx_data<="X"; tx_start<=1'b1; state<=ST_W_AX_L2; end end
                ST_W_AX_L2:  begin if (tx_done) state<=ST_TX_AX_EQ; end
                ST_TX_AX_EQ: begin if (!tx_busy_r) begin tx_data<="="; tx_start<=1'b1; state<=ST_W_AX_EQ; end end
                ST_W_AX_EQ:  begin if (tx_done) state<=ST_TX_AX_SG; end
                ST_TX_AX_SG: begin if (!tx_busy_r) begin tx_data<=ax_sg; tx_start<=1'b1; state<=ST_W_AX_SG; end end
                ST_W_AX_SG:  begin if (tx_done) state<=ST_TX_AX_D4; end
                ST_TX_AX_D4: begin if (!tx_busy_r) begin tx_data<=d2a(ax_d4); tx_start<=1'b1; state<=ST_W_AX_D4; end end
                ST_W_AX_D4:  begin if (tx_done) state<=ST_TX_AX_D3; end
                ST_TX_AX_D3: begin if (!tx_busy_r) begin tx_data<=d2a(ax_d3); tx_start<=1'b1; state<=ST_W_AX_D3; end end
                ST_W_AX_D3:  begin if (tx_done) state<=ST_TX_AX_D2; end
                ST_TX_AX_D2: begin if (!tx_busy_r) begin tx_data<=d2a(ax_d2); tx_start<=1'b1; state<=ST_W_AX_D2; end end
                ST_W_AX_D2:  begin if (tx_done) state<=ST_TX_AX_D1; end
                ST_TX_AX_D1: begin if (!tx_busy_r) begin tx_data<=d2a(ax_d1); tx_start<=1'b1; state<=ST_W_AX_D1; end end
                ST_W_AX_D1:  begin if (tx_done) state<=ST_TX_AX_D0; end
                ST_TX_AX_D0: begin if (!tx_busy_r) begin tx_data<=d2a(ax_d0); tx_start<=1'b1; state<=ST_W_AX_D0; end end
                ST_W_AX_D0:  begin if (tx_done) state<=ST_TX_AX_SP; end
                ST_TX_AX_SP: begin if (!tx_busy_r) begin tx_data<=" "; tx_start<=1'b1; state<=ST_W_AX_SP; end end
                ST_W_AX_SP:  begin if (tx_done) state<=ST_TX_AY_L1; end

                // ------ AY ------
                ST_TX_AY_L1: begin if (!tx_busy_r) begin tx_data<="A"; tx_start<=1'b1; state<=ST_W_AY_L1; end end
                ST_W_AY_L1:  begin if (tx_done) state<=ST_TX_AY_L2; end
                ST_TX_AY_L2: begin if (!tx_busy_r) begin tx_data<="Y"; tx_start<=1'b1; state<=ST_W_AY_L2; end end
                ST_W_AY_L2:  begin if (tx_done) state<=ST_TX_AY_EQ; end
                ST_TX_AY_EQ: begin if (!tx_busy_r) begin tx_data<="="; tx_start<=1'b1; state<=ST_W_AY_EQ; end end
                ST_W_AY_EQ:  begin if (tx_done) state<=ST_TX_AY_SG; end
                ST_TX_AY_SG: begin if (!tx_busy_r) begin tx_data<=ay_sg; tx_start<=1'b1; state<=ST_W_AY_SG; end end
                ST_W_AY_SG:  begin if (tx_done) state<=ST_TX_AY_D4; end
                ST_TX_AY_D4: begin if (!tx_busy_r) begin tx_data<=d2a(ay_d4); tx_start<=1'b1; state<=ST_W_AY_D4; end end
                ST_W_AY_D4:  begin if (tx_done) state<=ST_TX_AY_D3; end
                ST_TX_AY_D3: begin if (!tx_busy_r) begin tx_data<=d2a(ay_d3); tx_start<=1'b1; state<=ST_W_AY_D3; end end
                ST_W_AY_D3:  begin if (tx_done) state<=ST_TX_AY_D2; end
                ST_TX_AY_D2: begin if (!tx_busy_r) begin tx_data<=d2a(ay_d2); tx_start<=1'b1; state<=ST_W_AY_D2; end end
                ST_W_AY_D2:  begin if (tx_done) state<=ST_TX_AY_D1; end
                ST_TX_AY_D1: begin if (!tx_busy_r) begin tx_data<=d2a(ay_d1); tx_start<=1'b1; state<=ST_W_AY_D1; end end
                ST_W_AY_D1:  begin if (tx_done) state<=ST_TX_AY_D0; end
                ST_TX_AY_D0: begin if (!tx_busy_r) begin tx_data<=d2a(ay_d0); tx_start<=1'b1; state<=ST_W_AY_D0; end end
                ST_W_AY_D0:  begin if (tx_done) state<=ST_TX_AY_SP; end
                ST_TX_AY_SP: begin if (!tx_busy_r) begin tx_data<=" "; tx_start<=1'b1; state<=ST_W_AY_SP; end end
                ST_W_AY_SP:  begin if (tx_done) state<=ST_TX_AZ_L1; end

                // ------ AZ ------
                ST_TX_AZ_L1: begin if (!tx_busy_r) begin tx_data<="A"; tx_start<=1'b1; state<=ST_W_AZ_L1; end end
                ST_W_AZ_L1:  begin if (tx_done) state<=ST_TX_AZ_L2; end
                ST_TX_AZ_L2: begin if (!tx_busy_r) begin tx_data<="Z"; tx_start<=1'b1; state<=ST_W_AZ_L2; end end
                ST_W_AZ_L2:  begin if (tx_done) state<=ST_TX_AZ_EQ; end
                ST_TX_AZ_EQ: begin if (!tx_busy_r) begin tx_data<="="; tx_start<=1'b1; state<=ST_W_AZ_EQ; end end
                ST_W_AZ_EQ:  begin if (tx_done) state<=ST_TX_AZ_SG; end
                ST_TX_AZ_SG: begin if (!tx_busy_r) begin tx_data<=az_sg; tx_start<=1'b1; state<=ST_W_AZ_SG; end end
                ST_W_AZ_SG:  begin if (tx_done) state<=ST_TX_AZ_D4; end
                ST_TX_AZ_D4: begin if (!tx_busy_r) begin tx_data<=d2a(az_d4); tx_start<=1'b1; state<=ST_W_AZ_D4; end end
                ST_W_AZ_D4:  begin if (tx_done) state<=ST_TX_AZ_D3; end
                ST_TX_AZ_D3: begin if (!tx_busy_r) begin tx_data<=d2a(az_d3); tx_start<=1'b1; state<=ST_W_AZ_D3; end end
                ST_W_AZ_D3:  begin if (tx_done) state<=ST_TX_AZ_D2; end
                ST_TX_AZ_D2: begin if (!tx_busy_r) begin tx_data<=d2a(az_d2); tx_start<=1'b1; state<=ST_W_AZ_D2; end end
                ST_W_AZ_D2:  begin if (tx_done) state<=ST_TX_AZ_D1; end
                ST_TX_AZ_D1: begin if (!tx_busy_r) begin tx_data<=d2a(az_d1); tx_start<=1'b1; state<=ST_W_AZ_D1; end end
                ST_W_AZ_D1:  begin if (tx_done) state<=ST_TX_AZ_D0; end
                ST_TX_AZ_D0: begin if (!tx_busy_r) begin tx_data<=d2a(az_d0); tx_start<=1'b1; state<=ST_W_AZ_D0; end end
                ST_W_AZ_D0:  begin if (tx_done) state<=ST_TX_AZ_SP; end
                ST_TX_AZ_SP: begin if (!tx_busy_r) begin tx_data<=" "; tx_start<=1'b1; state<=ST_W_AZ_SP; end end
                ST_W_AZ_SP:  begin if (tx_done) state<=ST_TX_GX_L1; end

                // ------ GX ------
                ST_TX_GX_L1: begin if (!tx_busy_r) begin tx_data<="G"; tx_start<=1'b1; state<=ST_W_GX_L1; end end
                ST_W_GX_L1:  begin if (tx_done) state<=ST_TX_GX_L2; end
                ST_TX_GX_L2: begin if (!tx_busy_r) begin tx_data<="X"; tx_start<=1'b1; state<=ST_W_GX_L2; end end
                ST_W_GX_L2:  begin if (tx_done) state<=ST_TX_GX_EQ; end
                ST_TX_GX_EQ: begin if (!tx_busy_r) begin tx_data<="="; tx_start<=1'b1; state<=ST_W_GX_EQ; end end
                ST_W_GX_EQ:  begin if (tx_done) state<=ST_TX_GX_SG; end
                ST_TX_GX_SG: begin if (!tx_busy_r) begin tx_data<=gx_sg; tx_start<=1'b1; state<=ST_W_GX_SG; end end
                ST_W_GX_SG:  begin if (tx_done) state<=ST_TX_GX_D4; end
                ST_TX_GX_D4: begin if (!tx_busy_r) begin tx_data<=d2a(gx_d4); tx_start<=1'b1; state<=ST_W_GX_D4; end end
                ST_W_GX_D4:  begin if (tx_done) state<=ST_TX_GX_D3; end
                ST_TX_GX_D3: begin if (!tx_busy_r) begin tx_data<=d2a(gx_d3); tx_start<=1'b1; state<=ST_W_GX_D3; end end
                ST_W_GX_D3:  begin if (tx_done) state<=ST_TX_GX_D2; end
                ST_TX_GX_D2: begin if (!tx_busy_r) begin tx_data<=d2a(gx_d2); tx_start<=1'b1; state<=ST_W_GX_D2; end end
                ST_W_GX_D2:  begin if (tx_done) state<=ST_TX_GX_D1; end
                ST_TX_GX_D1: begin if (!tx_busy_r) begin tx_data<=d2a(gx_d1); tx_start<=1'b1; state<=ST_W_GX_D1; end end
                ST_W_GX_D1:  begin if (tx_done) state<=ST_TX_GX_D0; end
                ST_TX_GX_D0: begin if (!tx_busy_r) begin tx_data<=d2a(gx_d0); tx_start<=1'b1; state<=ST_W_GX_D0; end end
                ST_W_GX_D0:  begin if (tx_done) state<=ST_TX_GX_SP; end
                ST_TX_GX_SP: begin if (!tx_busy_r) begin tx_data<=" "; tx_start<=1'b1; state<=ST_W_GX_SP; end end
                ST_W_GX_SP:  begin if (tx_done) state<=ST_TX_GY_L1; end

                // ------ GY ------
                ST_TX_GY_L1: begin if (!tx_busy_r) begin tx_data<="G"; tx_start<=1'b1; state<=ST_W_GY_L1; end end
                ST_W_GY_L1:  begin if (tx_done) state<=ST_TX_GY_L2; end
                ST_TX_GY_L2: begin if (!tx_busy_r) begin tx_data<="Y"; tx_start<=1'b1; state<=ST_W_GY_L2; end end
                ST_W_GY_L2:  begin if (tx_done) state<=ST_TX_GY_EQ; end
                ST_TX_GY_EQ: begin if (!tx_busy_r) begin tx_data<="="; tx_start<=1'b1; state<=ST_W_GY_EQ; end end
                ST_W_GY_EQ:  begin if (tx_done) state<=ST_TX_GY_SG; end
                ST_TX_GY_SG: begin if (!tx_busy_r) begin tx_data<=gy_sg; tx_start<=1'b1; state<=ST_W_GY_SG; end end
                ST_W_GY_SG:  begin if (tx_done) state<=ST_TX_GY_D4; end
                ST_TX_GY_D4: begin if (!tx_busy_r) begin tx_data<=d2a(gy_d4); tx_start<=1'b1; state<=ST_W_GY_D4; end end
                ST_W_GY_D4:  begin if (tx_done) state<=ST_TX_GY_D3; end
                ST_TX_GY_D3: begin if (!tx_busy_r) begin tx_data<=d2a(gy_d3); tx_start<=1'b1; state<=ST_W_GY_D3; end end
                ST_W_GY_D3:  begin if (tx_done) state<=ST_TX_GY_D2; end
                ST_TX_GY_D2: begin if (!tx_busy_r) begin tx_data<=d2a(gy_d2); tx_start<=1'b1; state<=ST_W_GY_D2; end end
                ST_W_GY_D2:  begin if (tx_done) state<=ST_TX_GY_D1; end
                ST_TX_GY_D1: begin if (!tx_busy_r) begin tx_data<=d2a(gy_d1); tx_start<=1'b1; state<=ST_W_GY_D1; end end
                ST_W_GY_D1:  begin if (tx_done) state<=ST_TX_GY_D0; end
                ST_TX_GY_D0: begin if (!tx_busy_r) begin tx_data<=d2a(gy_d0); tx_start<=1'b1; state<=ST_W_GY_D0; end end
                ST_W_GY_D0:  begin if (tx_done) state<=ST_TX_GY_SP; end
                ST_TX_GY_SP: begin if (!tx_busy_r) begin tx_data<=" "; tx_start<=1'b1; state<=ST_W_GY_SP; end end
                ST_W_GY_SP:  begin if (tx_done) state<=ST_TX_GZ_L1; end

                // ------ GZ ------
                ST_TX_GZ_L1: begin if (!tx_busy_r) begin tx_data<="G"; tx_start<=1'b1; state<=ST_W_GZ_L1; end end
                ST_W_GZ_L1:  begin if (tx_done) state<=ST_TX_GZ_L2; end
                ST_TX_GZ_L2: begin if (!tx_busy_r) begin tx_data<="Z"; tx_start<=1'b1; state<=ST_W_GZ_L2; end end
                ST_W_GZ_L2:  begin if (tx_done) state<=ST_TX_GZ_EQ; end
                ST_TX_GZ_EQ: begin if (!tx_busy_r) begin tx_data<="="; tx_start<=1'b1; state<=ST_W_GZ_EQ; end end
                ST_W_GZ_EQ:  begin if (tx_done) state<=ST_TX_GZ_SG; end
                ST_TX_GZ_SG: begin if (!tx_busy_r) begin tx_data<=gz_sg; tx_start<=1'b1; state<=ST_W_GZ_SG; end end
                ST_W_GZ_SG:  begin if (tx_done) state<=ST_TX_GZ_D4; end
                ST_TX_GZ_D4: begin if (!tx_busy_r) begin tx_data<=d2a(gz_d4); tx_start<=1'b1; state<=ST_W_GZ_D4; end end
                ST_W_GZ_D4:  begin if (tx_done) state<=ST_TX_GZ_D3; end
                ST_TX_GZ_D3: begin if (!tx_busy_r) begin tx_data<=d2a(gz_d3); tx_start<=1'b1; state<=ST_W_GZ_D3; end end
                ST_W_GZ_D3:  begin if (tx_done) state<=ST_TX_GZ_D2; end
                ST_TX_GZ_D2: begin if (!tx_busy_r) begin tx_data<=d2a(gz_d2); tx_start<=1'b1; state<=ST_W_GZ_D2; end end
                ST_W_GZ_D2:  begin if (tx_done) state<=ST_TX_GZ_D1; end
                ST_TX_GZ_D1: begin if (!tx_busy_r) begin tx_data<=d2a(gz_d1); tx_start<=1'b1; state<=ST_W_GZ_D1; end end
                ST_W_GZ_D1:  begin if (tx_done) state<=ST_TX_GZ_D0; end
                ST_TX_GZ_D0: begin if (!tx_busy_r) begin tx_data<=d2a(gz_d0); tx_start<=1'b1; state<=ST_W_GZ_D0; end end
                ST_W_GZ_D0:  begin if (tx_done) state<=ST_TX_AN_SP; end

                // ------ AN ------
                ST_TX_AN_SP: begin if (!tx_busy_r) begin tx_data<=" "; tx_start<=1'b1; state<=ST_W_AN_SP; end end
                ST_W_AN_SP:  begin if (tx_done) state<=ST_TX_AN_L1; end
                ST_TX_AN_L1: begin if (!tx_busy_r) begin tx_data<="A"; tx_start<=1'b1; state<=ST_W_AN_L1; end end
                ST_W_AN_L1:  begin if (tx_done) state<=ST_TX_AN_L2; end
                ST_TX_AN_L2: begin if (!tx_busy_r) begin tx_data<="N"; tx_start<=1'b1; state<=ST_W_AN_L2; end end
                ST_W_AN_L2:  begin if (tx_done) state<=ST_TX_AN_EQ; end
                ST_TX_AN_EQ: begin if (!tx_busy_r) begin tx_data<="="; tx_start<=1'b1; state<=ST_W_AN_EQ; end end
                ST_W_AN_EQ:  begin if (tx_done) state<=ST_TX_AN_SG; end
                ST_TX_AN_SG: begin if (!tx_busy_r) begin tx_data<=an_sg; tx_start<=1'b1; state<=ST_W_AN_SG; end end
                ST_W_AN_SG:  begin if (tx_done) state<=ST_TX_AN_D4; end
                ST_TX_AN_D4: begin if (!tx_busy_r) begin tx_data<=d2a(an_d4); tx_start<=1'b1; state<=ST_W_AN_D4; end end
                ST_W_AN_D4:  begin if (tx_done) state<=ST_TX_AN_D3; end
                ST_TX_AN_D3: begin if (!tx_busy_r) begin tx_data<=d2a(an_d3); tx_start<=1'b1; state<=ST_W_AN_D3; end end
                ST_W_AN_D3:  begin if (tx_done) state<=ST_TX_AN_D2; end
                ST_TX_AN_D2: begin if (!tx_busy_r) begin tx_data<=d2a(an_d2); tx_start<=1'b1; state<=ST_W_AN_D2; end end
                ST_W_AN_D2:  begin if (tx_done) state<=ST_TX_AN_D1; end
                ST_TX_AN_D1: begin if (!tx_busy_r) begin tx_data<=d2a(an_d1); tx_start<=1'b1; state<=ST_W_AN_D1; end end
                ST_W_AN_D1:  begin if (tx_done) state<=ST_TX_AN_D0; end
                ST_TX_AN_D0: begin if (!tx_busy_r) begin tx_data<=d2a(an_d0); tx_start<=1'b1; state<=ST_W_AN_D0; end end
                ST_W_AN_D0:  begin if (tx_done) state<=ST_TX_KP_SP; end

                // ------ KP ------
                ST_TX_KP_SP: begin if (!tx_busy_r) begin tx_data<=" "; tx_start<=1'b1; state<=ST_W_KP_SP; end end
                ST_W_KP_SP:  begin if (tx_done) state<=ST_TX_KP_L1; end
                ST_TX_KP_L1: begin if (!tx_busy_r) begin tx_data<="K"; tx_start<=1'b1; state<=ST_W_KP_L1; end end
                ST_W_KP_L1:  begin if (tx_done) state<=ST_TX_KP_L2; end
                ST_TX_KP_L2: begin if (!tx_busy_r) begin tx_data<="P"; tx_start<=1'b1; state<=ST_W_KP_L2; end end
                ST_W_KP_L2:  begin if (tx_done) state<=ST_TX_KP_EQ; end
                ST_TX_KP_EQ: begin if (!tx_busy_r) begin tx_data<="="; tx_start<=1'b1; state<=ST_W_KP_EQ; end end
                ST_W_KP_EQ:  begin if (tx_done) state<=ST_TX_KP_D4; end
                ST_TX_KP_D4: begin if (!tx_busy_r) begin tx_data<=d2a(kp_d4); tx_start<=1'b1; state<=ST_W_KP_D4; end end
                ST_W_KP_D4:  begin if (tx_done) state<=ST_TX_KP_D3; end
                ST_TX_KP_D3: begin if (!tx_busy_r) begin tx_data<=d2a(kp_d3); tx_start<=1'b1; state<=ST_W_KP_D3; end end
                ST_W_KP_D3:  begin if (tx_done) state<=ST_TX_KP_D2; end
                ST_TX_KP_D2: begin if (!tx_busy_r) begin tx_data<=d2a(kp_d2); tx_start<=1'b1; state<=ST_W_KP_D2; end end
                ST_W_KP_D2:  begin if (tx_done) state<=ST_TX_KP_D1; end
                ST_TX_KP_D1: begin if (!tx_busy_r) begin tx_data<=d2a(kp_d1); tx_start<=1'b1; state<=ST_W_KP_D1; end end
                ST_W_KP_D1:  begin if (tx_done) state<=ST_TX_KP_D0; end
                ST_TX_KP_D0: begin if (!tx_busy_r) begin tx_data<=d2a(kp_d0); tx_start<=1'b1; state<=ST_W_KP_D0; end end
                ST_W_KP_D0:  begin if (tx_done) state<=ST_TX_KI_SP; end

                // ------ KI ------
                ST_TX_KI_SP: begin if (!tx_busy_r) begin tx_data<=" "; tx_start<=1'b1; state<=ST_W_KI_SP; end end
                ST_W_KI_SP:  begin if (tx_done) state<=ST_TX_KI_L1; end
                ST_TX_KI_L1: begin if (!tx_busy_r) begin tx_data<="K"; tx_start<=1'b1; state<=ST_W_KI_L1; end end
                ST_W_KI_L1:  begin if (tx_done) state<=ST_TX_KI_L2; end
                ST_TX_KI_L2: begin if (!tx_busy_r) begin tx_data<="I"; tx_start<=1'b1; state<=ST_W_KI_L2; end end
                ST_W_KI_L2:  begin if (tx_done) state<=ST_TX_KI_EQ; end
                ST_TX_KI_EQ: begin if (!tx_busy_r) begin tx_data<="="; tx_start<=1'b1; state<=ST_W_KI_EQ; end end
                ST_W_KI_EQ:  begin if (tx_done) state<=ST_TX_KI_D4; end
                ST_TX_KI_D4: begin if (!tx_busy_r) begin tx_data<=d2a(ki_d4); tx_start<=1'b1; state<=ST_W_KI_D4; end end
                ST_W_KI_D4:  begin if (tx_done) state<=ST_TX_KI_D3; end
                ST_TX_KI_D3: begin if (!tx_busy_r) begin tx_data<=d2a(ki_d3); tx_start<=1'b1; state<=ST_W_KI_D3; end end
                ST_W_KI_D3:  begin if (tx_done) state<=ST_TX_KI_D2; end
                ST_TX_KI_D2: begin if (!tx_busy_r) begin tx_data<=d2a(ki_d2); tx_start<=1'b1; state<=ST_W_KI_D2; end end
                ST_W_KI_D2:  begin if (tx_done) state<=ST_TX_KI_D1; end
                ST_TX_KI_D1: begin if (!tx_busy_r) begin tx_data<=d2a(ki_d1); tx_start<=1'b1; state<=ST_W_KI_D1; end end
                ST_W_KI_D1:  begin if (tx_done) state<=ST_TX_KI_D0; end
                ST_TX_KI_D0: begin if (!tx_busy_r) begin tx_data<=d2a(ki_d0); tx_start<=1'b1; state<=ST_W_KI_D0; end end
                ST_W_KI_D0:  begin if (tx_done) state<=ST_TX_KD_SP; end

                // ------ KD ------
                ST_TX_KD_SP: begin if (!tx_busy_r) begin tx_data<=" "; tx_start<=1'b1; state<=ST_W_KD_SP; end end
                ST_W_KD_SP:  begin if (tx_done) state<=ST_TX_KD_L1; end
                ST_TX_KD_L1: begin if (!tx_busy_r) begin tx_data<="K"; tx_start<=1'b1; state<=ST_W_KD_L1; end end
                ST_W_KD_L1:  begin if (tx_done) state<=ST_TX_KD_L2; end
                ST_TX_KD_L2: begin if (!tx_busy_r) begin tx_data<="D"; tx_start<=1'b1; state<=ST_W_KD_L2; end end
                ST_W_KD_L2:  begin if (tx_done) state<=ST_TX_KD_EQ; end
                ST_TX_KD_EQ: begin if (!tx_busy_r) begin tx_data<="="; tx_start<=1'b1; state<=ST_W_KD_EQ; end end
                ST_W_KD_EQ:  begin if (tx_done) state<=ST_TX_KD_D4; end
                ST_TX_KD_D4: begin if (!tx_busy_r) begin tx_data<=d2a(kd_d4); tx_start<=1'b1; state<=ST_W_KD_D4; end end
                ST_W_KD_D4:  begin if (tx_done) state<=ST_TX_KD_D3; end
                ST_TX_KD_D3: begin if (!tx_busy_r) begin tx_data<=d2a(kd_d3); tx_start<=1'b1; state<=ST_W_KD_D3; end end
                ST_W_KD_D3:  begin if (tx_done) state<=ST_TX_KD_D2; end
                ST_TX_KD_D2: begin if (!tx_busy_r) begin tx_data<=d2a(kd_d2); tx_start<=1'b1; state<=ST_W_KD_D2; end end
                ST_W_KD_D2:  begin if (tx_done) state<=ST_TX_KD_D1; end
                ST_TX_KD_D1: begin if (!tx_busy_r) begin tx_data<=d2a(kd_d1); tx_start<=1'b1; state<=ST_W_KD_D1; end end
                ST_W_KD_D1:  begin if (tx_done) state<=ST_TX_KD_D0; end
                ST_TX_KD_D0: begin if (!tx_busy_r) begin tx_data<=d2a(kd_d0); tx_start<=1'b1; state<=ST_W_KD_D0; end end
                ST_W_KD_D0:  begin if (tx_done) state<=ST_TX_SP_SP; end

                // ------ SP ------
                ST_TX_SP_SP: begin if (!tx_busy_r) begin tx_data<=" "; tx_start<=1'b1; state<=ST_W_SP_SP; end end
                ST_W_SP_SP:  begin if (tx_done) state<=ST_TX_SP_L1; end
                ST_TX_SP_L1: begin if (!tx_busy_r) begin tx_data<="S"; tx_start<=1'b1; state<=ST_W_SP_L1; end end
                ST_W_SP_L1:  begin if (tx_done) state<=ST_TX_SP_L2; end
                ST_TX_SP_L2: begin if (!tx_busy_r) begin tx_data<="P"; tx_start<=1'b1; state<=ST_W_SP_L2; end end
                ST_W_SP_L2:  begin if (tx_done) state<=ST_TX_SP_EQ; end
                ST_TX_SP_EQ: begin if (!tx_busy_r) begin tx_data<="="; tx_start<=1'b1; state<=ST_W_SP_EQ; end end
                ST_W_SP_EQ:  begin if (tx_done) state<=ST_TX_SP_SG; end
                ST_TX_SP_SG: begin if (!tx_busy_r) begin tx_data<=sp_sg; tx_start<=1'b1; state<=ST_W_SP_SG; end end
                ST_W_SP_SG:  begin if (tx_done) state<=ST_TX_SP_D4; end
                ST_TX_SP_D4: begin if (!tx_busy_r) begin tx_data<=d2a(sp_d4); tx_start<=1'b1; state<=ST_W_SP_D4; end end
                ST_W_SP_D4:  begin if (tx_done) state<=ST_TX_SP_D3; end
                ST_TX_SP_D3: begin if (!tx_busy_r) begin tx_data<=d2a(sp_d3); tx_start<=1'b1; state<=ST_W_SP_D3; end end
                ST_W_SP_D3:  begin if (tx_done) state<=ST_TX_SP_D2; end
                ST_TX_SP_D2: begin if (!tx_busy_r) begin tx_data<=d2a(sp_d2); tx_start<=1'b1; state<=ST_W_SP_D2; end end
                ST_W_SP_D2:  begin if (tx_done) state<=ST_TX_SP_D1; end
                ST_TX_SP_D1: begin if (!tx_busy_r) begin tx_data<=d2a(sp_d1); tx_start<=1'b1; state<=ST_W_SP_D1; end end
                ST_W_SP_D1:  begin if (tx_done) state<=ST_TX_SP_D0; end
                ST_TX_SP_D0: begin if (!tx_busy_r) begin tx_data<=d2a(sp_d0); tx_start<=1'b1; state<=ST_W_SP_D0; end end
                ST_W_SP_D0:  begin if (tx_done) state<=ST_TX_VA_SP; end   // → VA

                // 추가
                // ------ OF ------
                ST_TX_OF_SP: begin if (!tx_busy_r) begin tx_data<=" "; tx_start<=1'b1; state<=ST_W_OF_SP; end end
                ST_W_OF_SP:  begin if (tx_done) state<=ST_TX_OF_L1; end
                ST_TX_OF_L1: begin if (!tx_busy_r) begin tx_data<="O"; tx_start<=1'b1; state<=ST_W_OF_L1; end end
                ST_W_OF_L1:  begin if (tx_done) state<=ST_TX_OF_L2; end
                ST_TX_OF_L2: begin if (!tx_busy_r) begin tx_data<="F"; tx_start<=1'b1; state<=ST_W_OF_L2; end end
                ST_W_OF_L2:  begin if (tx_done) state<=ST_TX_OF_EQ; end
                ST_TX_OF_EQ: begin if (!tx_busy_r) begin tx_data<="="; tx_start<=1'b1; state<=ST_W_OF_EQ; end end
                ST_W_OF_EQ:  begin if (tx_done) state<=ST_TX_OF_SG; end
                ST_TX_OF_SG: begin if (!tx_busy_r) begin tx_data<=of_sg; tx_start<=1'b1; state<=ST_W_OF_SG; end end
                ST_W_OF_SG:  begin if (tx_done) state<=ST_TX_OF_D4; end
                ST_TX_OF_D4: begin if (!tx_busy_r) begin tx_data<=d2a(of_d4); tx_start<=1'b1; state<=ST_W_OF_D4; end end
                ST_W_OF_D4:  begin if (tx_done) state<=ST_TX_OF_D3; end
                ST_TX_OF_D3: begin if (!tx_busy_r) begin tx_data<=d2a(of_d3); tx_start<=1'b1; state<=ST_W_OF_D3; end end
                ST_W_OF_D3:  begin if (tx_done) state<=ST_TX_OF_D2; end
                ST_TX_OF_D2: begin if (!tx_busy_r) begin tx_data<=d2a(of_d2); tx_start<=1'b1; state<=ST_W_OF_D2; end end
                ST_W_OF_D2:  begin if (tx_done) state<=ST_TX_OF_D1; end
                ST_TX_OF_D1: begin if (!tx_busy_r) begin tx_data<=d2a(of_d1); tx_start<=1'b1; state<=ST_W_OF_D1; end end
                ST_W_OF_D1:  begin if (tx_done) state<=ST_TX_OF_D0; end
                ST_TX_OF_D0: begin if (!tx_busy_r) begin tx_data<=d2a(of_d0); tx_start<=1'b1; state<=ST_W_OF_D0; end end
                ST_W_OF_D0:  begin if (tx_done) state<=ST_TX_CR; end        

                // ------ VA (엔코더 속도 A) ------
                ST_TX_VA_SP: begin if (!tx_busy_r) begin tx_data<=" "; tx_start<=1'b1; state<=ST_W_VA_SP; end end
                ST_W_VA_SP:  begin if (tx_done) state<=ST_TX_VA_L1; end
                ST_TX_VA_L1: begin if (!tx_busy_r) begin tx_data<="V"; tx_start<=1'b1; state<=ST_W_VA_L1; end end
                ST_W_VA_L1:  begin if (tx_done) state<=ST_TX_VA_L2; end
                ST_TX_VA_L2: begin if (!tx_busy_r) begin tx_data<="A"; tx_start<=1'b1; state<=ST_W_VA_L2; end end
                ST_W_VA_L2:  begin if (tx_done) state<=ST_TX_VA_EQ; end
                ST_TX_VA_EQ: begin if (!tx_busy_r) begin tx_data<="="; tx_start<=1'b1; state<=ST_W_VA_EQ; end end
                ST_W_VA_EQ:  begin if (tx_done) state<=ST_TX_VA_SG; end
                ST_TX_VA_SG: begin if (!tx_busy_r) begin tx_data<=va_sg; tx_start<=1'b1; state<=ST_W_VA_SG; end end
                ST_W_VA_SG:  begin if (tx_done) state<=ST_TX_VA_D4; end
                ST_TX_VA_D4: begin if (!tx_busy_r) begin tx_data<=d2a(va_d4); tx_start<=1'b1; state<=ST_W_VA_D4; end end
                ST_W_VA_D4:  begin if (tx_done) state<=ST_TX_VA_D3; end
                ST_TX_VA_D3: begin if (!tx_busy_r) begin tx_data<=d2a(va_d3); tx_start<=1'b1; state<=ST_W_VA_D3; end end
                ST_W_VA_D3:  begin if (tx_done) state<=ST_TX_VA_D2; end
                ST_TX_VA_D2: begin if (!tx_busy_r) begin tx_data<=d2a(va_d2); tx_start<=1'b1; state<=ST_W_VA_D2; end end
                ST_W_VA_D2:  begin if (tx_done) state<=ST_TX_VA_D1; end
                ST_TX_VA_D1: begin if (!tx_busy_r) begin tx_data<=d2a(va_d1); tx_start<=1'b1; state<=ST_W_VA_D1; end end
                ST_W_VA_D1:  begin if (tx_done) state<=ST_TX_VA_D0; end
                ST_TX_VA_D0: begin if (!tx_busy_r) begin tx_data<=d2a(va_d0); tx_start<=1'b1; state<=ST_W_VA_D0; end end
                ST_W_VA_D0:  begin if (tx_done) state<=ST_TX_VB_SP; end

                // ------ VB (엔코더 속도 B) ------
                ST_TX_VB_SP: begin if (!tx_busy_r) begin tx_data<=" "; tx_start<=1'b1; state<=ST_W_VB_SP; end end
                ST_W_VB_SP:  begin if (tx_done) state<=ST_TX_VB_L1; end
                ST_TX_VB_L1: begin if (!tx_busy_r) begin tx_data<="V"; tx_start<=1'b1; state<=ST_W_VB_L1; end end
                ST_W_VB_L1:  begin if (tx_done) state<=ST_TX_VB_L2; end
                ST_TX_VB_L2: begin if (!tx_busy_r) begin tx_data<="B"; tx_start<=1'b1; state<=ST_W_VB_L2; end end
                ST_W_VB_L2:  begin if (tx_done) state<=ST_TX_VB_EQ; end
                ST_TX_VB_EQ: begin if (!tx_busy_r) begin tx_data<="="; tx_start<=1'b1; state<=ST_W_VB_EQ; end end
                ST_W_VB_EQ:  begin if (tx_done) state<=ST_TX_VB_SG; end
                ST_TX_VB_SG: begin if (!tx_busy_r) begin tx_data<=vb_sg; tx_start<=1'b1; state<=ST_W_VB_SG; end end
                ST_W_VB_SG:  begin if (tx_done) state<=ST_TX_VB_D4; end
                ST_TX_VB_D4: begin if (!tx_busy_r) begin tx_data<=d2a(vb_d4); tx_start<=1'b1; state<=ST_W_VB_D4; end end
                ST_W_VB_D4:  begin if (tx_done) state<=ST_TX_VB_D3; end
                ST_TX_VB_D3: begin if (!tx_busy_r) begin tx_data<=d2a(vb_d3); tx_start<=1'b1; state<=ST_W_VB_D3; end end
                ST_W_VB_D3:  begin if (tx_done) state<=ST_TX_VB_D2; end
                ST_TX_VB_D2: begin if (!tx_busy_r) begin tx_data<=d2a(vb_d2); tx_start<=1'b1; state<=ST_W_VB_D2; end end
                ST_W_VB_D2:  begin if (tx_done) state<=ST_TX_VB_D1; end
                ST_TX_VB_D1: begin if (!tx_busy_r) begin tx_data<=d2a(vb_d1); tx_start<=1'b1; state<=ST_W_VB_D1; end end
                ST_W_VB_D1:  begin if (tx_done) state<=ST_TX_VB_D0; end
                ST_TX_VB_D0: begin if (!tx_busy_r) begin tx_data<=d2a(vb_d0); tx_start<=1'b1; state<=ST_W_VB_D0; end end
                ST_W_VB_D0:  begin if (tx_done) state<=ST_TX_EA_SP; end

                // ------ EA (엔코더 위치 A) ------
                ST_TX_EA_SP: begin if (!tx_busy_r) begin tx_data<=" "; tx_start<=1'b1; state<=ST_W_EA_SP; end end
                ST_W_EA_SP:  begin if (tx_done) state<=ST_TX_EA_L1; end
                ST_TX_EA_L1: begin if (!tx_busy_r) begin tx_data<="E"; tx_start<=1'b1; state<=ST_W_EA_L1; end end
                ST_W_EA_L1:  begin if (tx_done) state<=ST_TX_EA_L2; end
                ST_TX_EA_L2: begin if (!tx_busy_r) begin tx_data<="A"; tx_start<=1'b1; state<=ST_W_EA_L2; end end
                ST_W_EA_L2:  begin if (tx_done) state<=ST_TX_EA_EQ; end
                ST_TX_EA_EQ: begin if (!tx_busy_r) begin tx_data<="="; tx_start<=1'b1; state<=ST_W_EA_EQ; end end
                ST_W_EA_EQ:  begin if (tx_done) state<=ST_TX_EA_SG; end
                ST_TX_EA_SG: begin if (!tx_busy_r) begin tx_data<=ea_sg; tx_start<=1'b1; state<=ST_W_EA_SG; end end
                ST_W_EA_SG:  begin if (tx_done) state<=ST_TX_EA_D4; end
                ST_TX_EA_D4: begin if (!tx_busy_r) begin tx_data<=d2a(ea_d4); tx_start<=1'b1; state<=ST_W_EA_D4; end end
                ST_W_EA_D4:  begin if (tx_done) state<=ST_TX_EA_D3; end
                ST_TX_EA_D3: begin if (!tx_busy_r) begin tx_data<=d2a(ea_d3); tx_start<=1'b1; state<=ST_W_EA_D3; end end
                ST_W_EA_D3:  begin if (tx_done) state<=ST_TX_EA_D2; end
                ST_TX_EA_D2: begin if (!tx_busy_r) begin tx_data<=d2a(ea_d2); tx_start<=1'b1; state<=ST_W_EA_D2; end end
                ST_W_EA_D2:  begin if (tx_done) state<=ST_TX_EA_D1; end
                ST_TX_EA_D1: begin if (!tx_busy_r) begin tx_data<=d2a(ea_d1); tx_start<=1'b1; state<=ST_W_EA_D1; end end
                ST_W_EA_D1:  begin if (tx_done) state<=ST_TX_EA_D0; end
                ST_TX_EA_D0: begin if (!tx_busy_r) begin tx_data<=d2a(ea_d0); tx_start<=1'b1; state<=ST_W_EA_D0; end end
                ST_W_EA_D0:  begin if (tx_done) state<=ST_TX_EB_SP; end

                // ------ EB (엔코더 위치 B) ------
                ST_TX_EB_SP: begin if (!tx_busy_r) begin tx_data<=" "; tx_start<=1'b1; state<=ST_W_EB_SP; end end
                ST_W_EB_SP:  begin if (tx_done) state<=ST_TX_EB_L1; end
                ST_TX_EB_L1: begin if (!tx_busy_r) begin tx_data<="E"; tx_start<=1'b1; state<=ST_W_EB_L1; end end
                ST_W_EB_L1:  begin if (tx_done) state<=ST_TX_EB_L2; end
                ST_TX_EB_L2: begin if (!tx_busy_r) begin tx_data<="B"; tx_start<=1'b1; state<=ST_W_EB_L2; end end
                ST_W_EB_L2:  begin if (tx_done) state<=ST_TX_EB_EQ; end
                ST_TX_EB_EQ: begin if (!tx_busy_r) begin tx_data<="="; tx_start<=1'b1; state<=ST_W_EB_EQ; end end
                ST_W_EB_EQ:  begin if (tx_done) state<=ST_TX_EB_SG; end
                ST_TX_EB_SG: begin if (!tx_busy_r) begin tx_data<=eb_sg; tx_start<=1'b1; state<=ST_W_EB_SG; end end
                ST_W_EB_SG:  begin if (tx_done) state<=ST_TX_EB_D4; end
                ST_TX_EB_D4: begin if (!tx_busy_r) begin tx_data<=d2a(eb_d4); tx_start<=1'b1; state<=ST_W_EB_D4; end end
                ST_W_EB_D4:  begin if (tx_done) state<=ST_TX_EB_D3; end
                ST_TX_EB_D3: begin if (!tx_busy_r) begin tx_data<=d2a(eb_d3); tx_start<=1'b1; state<=ST_W_EB_D3; end end
                ST_W_EB_D3:  begin if (tx_done) state<=ST_TX_EB_D2; end
                ST_TX_EB_D2: begin if (!tx_busy_r) begin tx_data<=d2a(eb_d2); tx_start<=1'b1; state<=ST_W_EB_D2; end end
                ST_W_EB_D2:  begin if (tx_done) state<=ST_TX_EB_D1; end
                ST_TX_EB_D1: begin if (!tx_busy_r) begin tx_data<=d2a(eb_d1); tx_start<=1'b1; state<=ST_W_EB_D1; end end
                ST_W_EB_D1:  begin if (tx_done) state<=ST_TX_EB_D0; end
                ST_TX_EB_D0: begin if (!tx_busy_r) begin tx_data<=d2a(eb_d0); tx_start<=1'b1; state<=ST_W_EB_D0; end end
                // ST_W_EB_D0:  begin if (tx_done) state<=ST_TX_CR; end   // → CR LF
                // 수정
                ST_W_EB_D0:  begin if (tx_done) state<=ST_TX_OF_SP; end 

                // ------ CR LF ------
                ST_TX_CR: begin if (!tx_busy_r) begin tx_data<=8'h0D; tx_start<=1'b1; state<=ST_W_CR; end end
                ST_W_CR:  begin if (tx_done) state<=ST_TX_LF; end
                ST_TX_LF: begin if (!tx_busy_r) begin tx_data<=8'h0A; tx_start<=1'b1; state<=ST_W_LF; end end
                ST_W_LF:  begin if (tx_done) state<=ST_IDLE; end

                default: state <= ST_IDLE;

            endcase
        end
    end

endmodule
