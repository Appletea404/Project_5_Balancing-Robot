`timescale 1ns / 1ps
//================================================================================
// pid -> 균형 로봇을 위한 PID 제어 모듈
//
// [ 동작 원리 ]
//   error     = setpoint - angle_in          (목표각 - 현재각)
//   P_term    = Kp * error
//   I_term   += Ki * error  (매 tick마다 누적, anti-windup 적용)
//   D_term    = Kd * (error - prev_error)    (전 tick과의 차분)
//   pid_out   = P_term + I_term + D_term
//
// [ 고정소수점 표현: Q8 포맷 ]
//   이득 값(Kp, Ki, Kd)은 실수 * 256으로 스케일링하여 정수로 표현
//   예) Kp=1.5 → KP = 384 (= 1.5 * 256)
//   최종 출력 시 >> 8 하여 소수 부분 제거
//
// [ 포트 ]
//   angle_in  : MPU6050에서 넘어오는 현재 기울기 (16비트 부호있는 정수)
//   setpoint  : 목표 각도 (보통 0)
//   tick      : clk_divider에서 오는 느린 펄스 (PID 계산 타이밍)
//   pwm_duty  : L298N으로 보내는 PWM 듀티값 (0 ~ MAX_DUTY)
//   dir       : 모터 방향 (0: 전진, 1: 후진)
//================================================================================

module pid
(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        tick,                    // PID 업데이트 타이밍 (clk_divider tick)

    input  wire signed [15:0] angle_in,         // 현재 기울기 (MPU6050 raw 또는 변환값)
    input  wire signed [15:0] setpoint,         // 목표 각도 (보통 16'd0)

    output reg  [15:0] pwm_duty,                // PWM 듀티 (0 ~ MAX_DUTY)
    output reg         dir                      // 방향: 0 = 전진, 1 = 후진
);

//================================================================================
// 파라미터: 이득값 (Q8 고정소수점: 실제값 * 256)
// 튜닝 시 이 값만 수정
//================================================================================
parameter signed [15:0] KP         = 16'd256;  // Kp = 1.0
parameter signed [15:0] KI         = 16'd26;   // Ki ≈ 0.1
parameter signed [15:0] KD         = 16'd512;  // Kd = 2.0

parameter [15:0] MAX_DUTY          = 16'd1000; // PWM 최대 듀티 (L298N에 맞게 조정)
parameter signed [31:0] I_MAX      = 32'sd500000; // 적분 항 상한 (anti-windup)
parameter signed [31:0] I_MIN      = -32'sd500000; // 적분 항 하한

//================================================================================
// 내부 신호
//================================================================================
reg  signed [15:0] error;           // 현재 오차
reg  signed [15:0] prev_error;      // 이전 틱의 오차 (D항 계산용)
reg  signed [31:0] integral;        // 누적 오차 (I항)
wire signed [15:0] d_error;         // 오차 변화량 (D항)

wire signed [31:0] p_term;          // P 항 (Q8)
wire signed [31:0] i_term;          // I 항 (Q8)
wire signed [31:0] d_term;          // D 항 (Q8)
wire signed [31:0] pid_sum;         // PID 합산 (Q8)
wire signed [31:0] pid_out;         // PID 출력 (Q8 >> 8 = 실제값)
wire signed [31:0] neg_pid_out;     // pid_out 부호 반전 (후진 시 절댓값용)

//================================================================================
// 오차 및 D항 계산
//================================================================================
assign d_error = error - prev_error;

//================================================================================
// PID 각 항 계산 (Q8 스케일)
//================================================================================
assign p_term  = KP * error;
assign i_term  = integral;          // 이미 Ki가 곱해진 누적값
assign d_term  = KD * d_error;

assign pid_sum = p_term + i_term + d_term;

// Q8 >> 8: 소수 부분 제거하여 실제 제어값 도출
assign pid_out     = pid_sum >>> 8;
assign neg_pid_out = -pid_out;

//================================================================================
// 순서형 로직: tick마다 PID 상태 갱신
//================================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        error      <= 16'sd0;
        prev_error <= 16'sd0;
        integral   <= 32'sd0;
        pwm_duty   <= 16'd0;
        dir        <= 1'b0;
    end

    else if (tick) begin
        //----------------------------------------------------------------------
        // 1. 오차 계산
        //----------------------------------------------------------------------
        prev_error <= error;
        error      <= setpoint - angle_in;

        //----------------------------------------------------------------------
        // 2. 적분 누적 (anti-windup: 범위 초과 시 클램핑)
        //----------------------------------------------------------------------
        if (integral + KI * error > I_MAX)
            integral <= I_MAX;
        else if (integral + KI * error < I_MIN)
            integral <= I_MIN;
        else
            integral <= integral + KI * error;

        //----------------------------------------------------------------------
        // 3. PWM 듀티 및 방향 출력
        //    pid_out > 0: 앞으로 기울어짐 → 전진
        //    pid_out < 0: 뒤로 기울어짐  → 후진
        //    절댓값을 듀티로, MAX_DUTY로 상한 클램핑
        //----------------------------------------------------------------------
        if (pid_out >= 32'sd0) begin
            dir <= 1'b0;    // 전진
            if (pid_out > MAX_DUTY)
                pwm_duty <= MAX_DUTY;
            else
                pwm_duty <= pid_out[15:0];
        end
        else begin
            dir <= 1'b1;    // 후진
            if (-pid_out > MAX_DUTY)
                pwm_duty <= MAX_DUTY;
            else
                pwm_duty <= neg_pid_out[15:0];
        end
    end
end

endmodule
