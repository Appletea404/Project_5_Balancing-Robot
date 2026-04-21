`timescale 1ns / 1ps
//==============================================================================
// angle_calc — 상보 필터(Complementary Filter) [파이프라인 적용 버전]
//==============================================================================

module angle_calc
(
    input  wire               clk,
    input  wire               rst_n,

    input  wire               data_valid,    // mpu6050_ctrl에서 오는 10ms 주기 펄스

    input  wire signed [15:0] accel_x,       // bias 보정됨, 16384 = 1g
    input  wire signed [15:0] accel_z,       // 현재 미사용
    input  wire signed [15:0] gyro_x,        // bias 보정됨, 131 LSB/°/s

    output reg  signed [15:0] angle,         // 기울기 각도 [0.01°], 직립=0
    output reg                angle_valid    // 각도 갱신 완료 1클럭 펄스
);

    //--------------------------------------------------------------------------
    // 파라미터 (튜닝 가능)
    //--------------------------------------------------------------------------
    parameter signed [31:0] ALPHA_NUM  = 32'sd249;  // α = 249/256 ≈ 0.97
    parameter signed [31:0] ALPHA_COMP = 32'sd7;    // 1-α = 7/256 ≈ 0.03

    //--------------------------------------------------------------------------
    // 내부 레지스터
    //--------------------------------------------------------------------------
    reg signed [31:0] angle_q8;

    // 파이프라인(Pipeline) 분리용 레지스터
    reg signed [31:0] accel_q8_reg;
    reg signed [31:0] gyro_q8_reg;
    reg               calc_step2;


    // top.v에서 accel_z는 bias를 뺀 값이므로 직립 시 0 근처가 된다.
    // 따라서 accel_z 크기로 gain을 바꾸면 거의 항상 최저 gain만 선택된다.
    // 현재 배치/로그 기준에서는 고정 gain 쪽이 더 예측 가능하다.
    localparam signed [31:0] ACCEL_GAIN = 32'sd60;
    localparam signed [31:0] GYRO_GAIN  = 32'sd250;

    // 다음 클럭의 상보 필터 계산 결과를 담을 wire (1클럭 지연 없는 출력을 위함)
    wire signed [31:0] next_angle_q8;
    assign next_angle_q8 = (ALPHA_NUM  * (angle_q8 + gyro_q8_reg) + 
                            ALPHA_COMP * accel_q8_reg) >>> 8;

    //--------------------------------------------------------------------------
    // 상보 필터 FSM (2-Stage 파이프라인)
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            angle_q8     <= 32'sd0;
            angle        <= 16'sd0;
            angle_valid  <= 1'b0;
            
            accel_q8_reg <= 32'sd0;
            gyro_q8_reg  <= 32'sd0;
            calc_step2   <= 1'b0;

        end
        else begin
            // 펄스성 신호 기본값 초기화
            angle_valid <= 1'b0;
            calc_step2  <= 1'b0;



            // 수정 전
            // // [Stage 1] 데이터 수신 및 1차 곱셈 (무거운 연산 먼저 수행)
            // if (data_valid) begin
            //     // 수정 전
            //     // accel_q8_reg <= $signed(accel_x) * 32'sd90;
            //     // 수정 후
            //     accel_q8_reg <= $signed(accel_x) * 32'sd60;
                
            //     gyro_q8_reg  <= ($signed(gyro_x) * 32'sd500) >>> 8;                
            //     calc_step2   <= 1'b1; // 다음 클럭에서 2단계 진행하도록 플래그 세움
            // end


            // 수정 후
            // // [Stage 1] 데이터 수신 및 1차 곱셈 (무거운 연산 먼저 수행)            
            if (data_valid) begin
                accel_q8_reg <= $signed(accel_x) * ACCEL_GAIN;
                // mpu6050_ctrl read period is 5ms, so the gyro integration gain
                // is halved from the old 10ms setting.
                gyro_q8_reg  <= ($signed(gyro_x) * GYRO_GAIN) >>> 8;
                calc_step2   <= 1'b1; // 다음 클럭에서 2단계 진행하도록 플래그 세움
            end

            // [Stage 2] 1차 연산된 레지스터 값을 이용해 덧셈 및 최종 출력
            if (calc_step2) begin
                angle_q8    <= next_angle_q8;
                angle       <= next_angle_q8[23:8]; // Q8 정규화된 실제 각도 추출
                angle_valid <= 1'b1;
            end
        end
    end

endmodule
