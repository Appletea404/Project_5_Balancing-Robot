`timescale 1ns / 1ps
//================================================================================
// pid -> 균형 로봇을 위한 PID 제어 모듈 (FSM 기반 타이밍 최적화 버전)
//================================================================================

module pid
(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        en,                      // 10ms 주기 업데이트 트리거

    input  wire signed [15:0] angle_in,
    input  wire signed [15:0] setpoint,

    input  wire [15:0] kp,
    input  wire [15:0] ki,
    input  wire [15:0] kd,

    output reg  [15:0] pwm_duty,
    output reg         dir
);

    parameter [15:0] MAX_DUTY          = 16'd1000;
    parameter signed [31:0] I_MAX      = 32'sd500000;
    parameter signed [31:0] I_MIN      = -32'sd500000;

    //================================================================================
    // FSM 상태 정의
    //================================================================================
    localparam [2:0]
        ST_IDLE   = 3'd0,
        ST_TERMS  = 3'd1,
        ST_SUM    = 3'd2,
        ST_OUT    = 3'd3;

    reg [2:0] state;

    //================================================================================
    // 내부 상태 레지스터
    //================================================================================
    reg signed [15:0] error;
    reg signed [15:0] prev_error;
    reg signed [31:0] integral;

    // 중간 연산 보관 레지스터
    reg signed [31:0] p_term_reg;
    reg signed [31:0] d_term_reg;
    reg signed [31:0] pid_sum_reg;

    //================================================================================
    // 출력 계산용 조합 논리 (ST_OUT 에서만 사용)
    //================================================================================
    wire signed [31:0] pid_out     = pid_sum_reg >>> 8;
    wire signed [31:0] neg_pid_out = -pid_out;
    localparam [15:0] MIN_DUTY = 16'd50;


    //================================================================================
    // FSM PID 로직
    //================================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= ST_IDLE;
            error       <= 16'sd0;
            prev_error  <= 16'sd0;
            integral    <= 32'sd0;
            
            p_term_reg  <= 32'sd0;
            d_term_reg  <= 32'sd0;
            pid_sum_reg <= 32'sd0;

            pwm_duty    <= 16'd0;
            dir         <= 1'b0;
        end
        else begin
            case (state)
                //------------------------------------------------------------------
                // [Stage 0] 오차 갱신 및 대기
                //------------------------------------------------------------------
                ST_IDLE: begin
                    if (en) begin
                        prev_error <= error;
                        error      <= setpoint - angle_in;
                        state      <= ST_TERMS; // 다음 연산 단계로 이동
                    end
                end

                //------------------------------------------------------------------
                // [Stage 1] 가장 무거운 곱셈 및 적분 누적
                //------------------------------------------------------------------
                ST_TERMS: begin
                    p_term_reg <= $signed(kp) * error;
                    d_term_reg <= $signed(kd) * (error - prev_error);

                    if (integral + $signed(ki) * error > I_MAX)
                        integral <= I_MAX;
                    else if (integral + $signed(ki) * error < I_MIN)
                        integral <= I_MIN;
                    else
                        integral <= integral + $signed(ki) * error;

                    state <= ST_SUM;
                end

                //------------------------------------------------------------------
                // [Stage 2] PID 항 합산
                //------------------------------------------------------------------
                ST_SUM: begin
                    pid_sum_reg <= p_term_reg + integral + d_term_reg;
                    state       <= ST_OUT;
                end

                //------------------------------------------------------------------
                // [Stage 3] 스케일링(>>8) 및 출력 클램핑
                //------------------------------------------------------------------
                //------------------------------------------------------------------
                // [Stage 3] 스케일링(>>8) 및 출력 클램핑 + 데드존 보상
                //------------------------------------------------------------------
                ST_OUT: begin
                    // ★ 튜닝 포인트: 모터가 윙 소리만 내고 돌지 않는 최대 PWM 값
                    // 이 값을 50, 80, 100 등으로 바꿔가며 기어 마찰력을 상쇄시킵니다.

                    if (pid_out > 32'sd0) begin
                        dir <= 1'b0;    // 전진
                        // 계산값에 최소 듀티(MIN_DUTY)를 무조건 더해서 출력
                        if (pid_out + MIN_DUTY > MAX_DUTY)
                            pwm_duty <= MAX_DUTY;
                        else
                            pwm_duty <= pid_out[15:0] + MIN_DUTY;
                    end
                    else if (pid_out < 32'sd0) begin
                        dir <= 1'b1;    // 후진
                        if (neg_pid_out + MIN_DUTY > MAX_DUTY)
                            pwm_duty <= MAX_DUTY;
                        else
                            pwm_duty <= neg_pid_out[15:0] + MIN_DUTY;
                    end
                    else begin
                        // 오차가 완벽히 0일 때는 모터 정지
                        dir <= 1'b0;
                        pwm_duty <= 16'd0;
                    end
                    
                    state <= ST_IDLE; // 모든 연산을 마치고 다시 대기 상태로
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule