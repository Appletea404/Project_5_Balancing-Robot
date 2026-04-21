`timescale 1ns / 1ps
//================================================================================
// 모듈명: pid.v
// 기능: 밸런싱 로봇을 위한 FSM 기반 파이프라인 PID 제어기
// 특징: Center Hold (미세진동 제거), Start Boost (정지마찰 극복), Anti-Windup 적용
//================================================================================

module pid
(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        en,                      // 5ms 주기 제어 루프 트리거
    input  wire        i_clear,                 // 적분항 초기화 펄스 (1클럭)

    input  wire signed [15:0] angle_in,         // 현재 로봇 기울기 각도
    input  wire signed [15:0] setpoint,         // 목표 각도 (원격 주행 시 변경됨)
    input  wire signed [15:0] vel_in,           // 양쪽 바퀴의 속도 피드백
    input  wire signed [15:0] gyro_in,          // 현재 각속도 (D 항 계산용)

    input  wire [15:0] kp,                      // P 게인 (비례)
    input  wire [15:0] ki,                      // I 게인 (적분)
    input  wire [15:0] kd,                      // D 게인 (미분)

    output reg  [15:0] pwm_duty,                // 최종 출력 PWM 듀티 (0 ~ 1000)
    output reg         dir,                     // 모터 회전 방향 (0: 전진, 1: 후진)
    
    // 디버깅 및 모니터링 출력
    output wire signed [15:0] pid_out_dbg,
    output reg         sat_flag,
    output reg         active_min_applied,
    output reg  [6:0]  motor_duty_dbg,
    output reg         boost_active_dbg,
    output wire        center_hold_dbg
);

    //================================================================================
    // 1. 하드웨어 및 제어 파라미터 튜닝 설정
    //================================================================================
    
    // 1-1. 적분항 (I) 제한
    parameter signed [31:0] I_MAX      = 32'sd500000;
    parameter signed [31:0] I_MIN      = -32'sd500000;
    localparam signed [15:0] I_ERR_DEAD = 16'sd8;       // 오차가 이 값 이하이면 적분값을 서서히 감소시킴

    // 1-2. 모터 구동 한계치 (JGB37-520 모터 기준)
    parameter [6:0] DUTY_ACTIVE_MIN    = 7'd20;         // 정지마찰력을 극복하기 위한 최소 PWM 듀티
    parameter [6:0] DUTY_MAX_MOTOR     = 7'd75;         // 모터 하드웨어 보호용 최대 PWM 듀티 (75%)
    parameter signed [15:0] PID_OUT_CLAMP = 16'sd900;   // PID 연산 총합의 물리적 한계치
    
    // 1-3. D 항 (미분) 제어 제한
    parameter signed [15:0] GYRO_D_DEAD = 16'sd96;      // 자이로 노이즈 무시 구간
    parameter signed [15:0] GYRO_D_LIM  = 16'sd1024;    // 너무 큰 미분값에 의한 발작 방지 
    parameter signed [31:0] D_TERM_SUM_CLAMP = 32'sd30720;
    localparam integer GYRO_D_SHIFT = 2;                // D 게인 민감도 축소 (x 0.25)
    localparam integer D_TERM_POST_SHIFT = 2;

    // 1-4. 비선형 출력 매핑 (오차 크기에 따른 출력 강도 조절)
    parameter signed [15:0] MAP_SMALL_THR = 16'sd160;
    parameter signed [15:0] MAP_MID_THR   = 16'sd260;
    parameter signed [15:0] SMALL_MIN_OUT_THR = 16'sd56;

    // 1-5. Center Hold (수직 정지 유지) 파라미터
    parameter signed [15:0] HOLD_ENTER_THR = 16'sd6;    // 각도 오차가 이 값보다 작아지면 모터 차단 준비
    parameter signed [15:0] HOLD_EXIT_THR  = 16'sd10;   // 각도 오차가 이 값보다 커지면 모터 재가동
    parameter signed [15:0] SMALL_VEL_THR  = 16'sd8;    // 속도가 이 값보다 작아야 정지한 것으로 간주
    parameter signed [15:0] HOLD_EXIT_VEL_THR = 16'sd8; // 밀려서 속도가 생기면 즉시 재가동
    parameter [2:0] HOLD_RELEASE_SOFT_CYCLES = 3'd4;    // 정지 해제 시 갑자기 튀는 것을 막는 소프트 스타트
    parameter [6:0] HOLD_RELEASE_DUTY_MAX    = 7'd16;

    // 1-6. 방향 전환 히스테리시스 (0 근처에서 방향이 미친 듯이 바뀌는 현상 방지)
    parameter signed [15:0] DIR_ENTER_THR = 16'sd4;
    parameter signed [15:0] DIR_EXIT_THR  = 16'sd2;

    // 1-7. Start Boost (초기 기동 펄스)
    parameter        START_BOOST_ENABLE      = 1'b1;
    parameter signed [15:0] BOOST_ERR_MIN    = 16'sd96;
    parameter signed [15:0] BOOST_REST_VEL_THR = 16'sd6;
    parameter [1:0]  BOOST_DIR_STABLE_CYCLES = 2'd2;
    parameter [2:0]  START_BOOST_HOLD_CYCLES = 3'd1;
    parameter [6:0]  START_BOOST_DUTY        = 7'd22;

    //================================================================================
    // 2. FSM 상태 및 내부 레지스터 정의
    //================================================================================
    localparam [2:0]
        ST_IDLE   = 3'd0,  // 입력 대기 및 갱신
        ST_TERMS  = 3'd1,  // P, I, D 곱셈 연산
        ST_INTEG  = 3'd2,  // 적분 누적 및 안티와인드업
        ST_SUM    = 3'd3,  // PID 총합 계산
        ST_OUT    = 3'd4,  // 방향 및 절댓값 도출, Center Hold 판별
        ST_APPLY  = 3'd5,  // 모터 듀티 매핑 및 부스트 적용
        ST_PWM    = 3'd6;  // 최종 PWM 스케일링

    reg [2:0] state;

    reg signed [15:0] error;
    reg signed [15:0] prev_error;
    reg signed [31:0] integral;

    // 연산 파이프라인 레지스터
    reg signed [31:0] p_term_reg;
    reg signed [31:0] ki_term_reg;
    reg signed [31:0] d_term_reg;
    reg signed [31:0] pid_sum_reg;

    // 모터 제어 및 상태 보존 레지스터
    reg [6:0]         motor_duty_sel_reg;
    reg signed [15:0] pid_out_abs_reg;
    reg signed [15:0] error_abs_reg;
    reg signed [15:0] gyro_d_reg;
    reg signed [15:0] vel_abs_reg;
    reg               pid_active_reg;
    reg               center_hold_reg;
    reg               hold_prev_reg;
    reg               dir_prev_reg;
    reg [1:0]         dir_stable_cnt;
    reg [2:0]         hold_release_soft_cnt;
    reg [2:0]         boost_hold_cnt;

    //================================================================================
    // 3. FSM 내부 조합 논리 (ST_OUT 및 ST_APPLY 단계용)
    //================================================================================
    wire signed [31:0] pid_out     = pid_sum_reg >>> 8;
    wire signed [15:0] error_next_w = setpoint - angle_in;
    wire signed [15:0] gyro_abs_w  = gyro_in[15] ? -gyro_in : gyro_in;
    wire signed [15:0] vel_abs_w   = vel_in[15]  ? -vel_in  : vel_in;

    // 자이로 데드존 및 클램프 처리
    wire signed [15:0] gyro_d_used_w =
        (gyro_abs_w <= GYRO_D_DEAD) ? 16'sd0 :
        (gyro_in >  GYRO_D_LIM)     ? GYRO_D_LIM :
        (gyro_in < -GYRO_D_LIM)     ? -GYRO_D_LIM : gyro_in;

    // 미분항 클램프 처리
    wire signed [31:0] d_term_used_w =
        (d_term_reg >  D_TERM_SUM_CLAMP) ? D_TERM_SUM_CLAMP :
        (d_term_reg < -D_TERM_SUM_CLAMP) ? -D_TERM_SUM_CLAMP : d_term_reg;
    wire signed [31:0] d_term_shaped_w = d_term_used_w >>> D_TERM_POST_SHIFT;

    // 총합 출력 클램프 처리
    wire signed [15:0] pid_out_used_w =
        (pid_out >  PID_OUT_CLAMP) ? PID_OUT_CLAMP :
        (pid_out < -PID_OUT_CLAMP) ? -PID_OUT_CLAMP : pid_out[15:0];
    wire pid_out_clamped_w = (pid_out > PID_OUT_CLAMP) || (pid_out < -PID_OUT_CLAMP);

    wire signed [15:0] error_abs = error[15] ? -error : error;

    // 비선형 출력 매핑 계산 (구간별 강도 조절)
    wire [15:0] mid_delta_w   = pid_out_abs_reg[15:0] - MAP_SMALL_THR[15:0];
    wire [15:0] large_delta_w = pid_out_abs_reg[15:0] - MAP_MID_THR[15:0];
    wire [6:0] duty_small_w = (pid_out_abs_reg[15:0] >> 4);
    wire [6:0] duty_mid_w   = 7'd24 + (mid_delta_w >> 10);
    wire [6:0] duty_large_w = 7'd52 + (large_delta_w >> 6);
    wire [6:0] duty_piecewise_w =
        (pid_out_abs_reg <= MAP_SMALL_THR) ? duty_small_w :
        (pid_out_abs_reg <= MAP_MID_THR)   ? duty_mid_w   : duty_large_w;

    // Active Min 램프 계산 (점진적 구동력 상승)
    wire [15:0] active_min_delta_w =
        (pid_out_abs_reg > SMALL_MIN_OUT_THR) ? (pid_out_abs_reg - SMALL_MIN_OUT_THR) : 16'd0;
    wire [8:0] active_min_ramp_ext_w = {3'b000, active_min_delta_w[15:6]};
    wire [6:0] active_min_ramp_w =
        active_min_ramp_ext_w[8] ? DUTY_ACTIVE_MIN :
        (active_min_ramp_ext_w[6:0] > DUTY_ACTIVE_MIN) ? DUTY_ACTIVE_MIN : active_min_ramp_ext_w[6:0];
    
    wire active_min_need_w =
        (pid_out_abs_reg >= SMALL_MIN_OUT_THR) &&
        (error_abs_reg > HOLD_EXIT_THR) &&
        (active_min_ramp_w > duty_piecewise_w);

    wire [6:0] duty_pre_sat_w = active_min_need_w ? active_min_ramp_w : duty_piecewise_w;
    wire sat_flag_w = (duty_pre_sat_w > DUTY_MAX_MOTOR) || pid_out_clamped_w;
    wire [6:0] duty_final_w = sat_flag_w ? DUTY_MAX_MOTOR : duty_pre_sat_w;

    // Start Boost 논리 계산
    wire boost_rest_w = (vel_abs_reg <= BOOST_REST_VEL_THR);
    wire boost_dir_stable_w = (dir_stable_cnt >= BOOST_DIR_STABLE_CYCLES);
    wire boost_req_w =
        START_BOOST_ENABLE && active_min_need_w &&
        (error_abs_reg >= BOOST_ERR_MIN) && boost_rest_w && boost_dir_stable_w;
    wire [6:0] boost_floor_w = (START_BOOST_DUTY > duty_final_w) ? START_BOOST_DUTY : duty_final_w;
    wire boost_sat_w = (boost_floor_w > DUTY_MAX_MOTOR);
    wire [6:0] boost_duty_w = boost_sat_w ? DUTY_MAX_MOTOR : boost_floor_w;

    // Center Hold (정지 유지) 진입 및 해제 판별
    wire hold_enter_w = (error_abs < HOLD_ENTER_THR) && (vel_abs_reg < SMALL_VEL_THR);
    wire hold_exit_w  = (error_abs > HOLD_EXIT_THR)  || (vel_abs_reg > HOLD_EXIT_VEL_THR);
    wire center_hold_next_w = center_hold_reg ? !hold_exit_w : hold_enter_w;
    wire hold_apply_w = center_hold_next_w;
    wire just_released_hold_w = hold_prev_reg && !hold_apply_w;
    wire hold_release_soft_w = (hold_release_soft_cnt != 3'd0);
    wire hold_release_apply_w = hold_release_soft_w || just_released_hold_w;

    wire [6:0] duty_after_soft_w = (duty_final_w > HOLD_RELEASE_DUTY_MAX) ? HOLD_RELEASE_DUTY_MAX : duty_final_w;
    wire [6:0] boost_after_soft_w = (boost_duty_w > HOLD_RELEASE_DUTY_MAX) ? HOLD_RELEASE_DUTY_MAX : boost_duty_w;

    assign pid_out_dbg =
        (pid_out >  32'sd32767) ? 16'sd32767 :
        (pid_out < -32'sd32768) ? -16'sd32768 : pid_out[15:0];
    assign center_hold_dbg = center_hold_reg;

    //================================================================================
    // 4. 메인 FSM 제어 루프
    //================================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= ST_IDLE;
            error       <= 16'sd0;
            prev_error  <= 16'sd0;
            integral    <= 32'sd0;
            p_term_reg  <= 32'sd0;
            ki_term_reg <= 32'sd0;
            d_term_reg  <= 32'sd0;
            pid_sum_reg <= 32'sd0;
            motor_duty_sel_reg <= 7'd0;
            pid_out_abs_reg <= 16'sd0;
            error_abs_reg <= 16'sd0;
            gyro_d_reg <= 16'sd0;
            vel_abs_reg <= 16'sd0;
            pid_active_reg <= 1'b0;
            center_hold_reg <= 1'b0;
            hold_prev_reg <= 1'b0;
            dir_prev_reg <= 1'b0;
            dir_stable_cnt <= 2'd0;
            hold_release_soft_cnt <= 3'd0;
            pwm_duty    <= 16'd0;
            dir         <= 1'b0;
            sat_flag    <= 1'b0;
            active_min_applied <= 1'b0;
            motor_duty_dbg <= 7'd0;
            boost_active_dbg <= 1'b0;
            boost_hold_cnt <= 3'd0;
        end
        else begin
            // 터미널 입력 등에 의한 강제 적분항 초기화
            if (i_clear) begin
                integral    <= 32'sd0;
                prev_error  <= 16'sd0;
                state       <= ST_IDLE;
            end
            
            case (state)
                //------------------------------------------------------------------
                // [Stage 0] 데이터 샘플링 대기 및 갱신
                //------------------------------------------------------------------
                ST_IDLE: begin
                    if (en) begin
                        prev_error <= error;
                        error      <= error_next_w;
                        gyro_d_reg <= gyro_d_used_w;
                        vel_abs_reg <= vel_abs_w;
                        state      <= ST_TERMS;
                    end
                end

                //------------------------------------------------------------------
                // [Stage 1] P, I, D 게인 곱셈 연산
                //------------------------------------------------------------------
                ST_TERMS: begin
                    p_term_reg  <= $signed(kp) * error;
                    ki_term_reg <= $signed(ki) * error;
                    // D 항은 각도 차분이 아닌 자이로 값을 직접 곱하여 응답성 확보
                    d_term_reg  <= ($signed(kd) * $signed(gyro_d_reg)) >>> GYRO_D_SHIFT;
                    state <= ST_INTEG;
                end

                //------------------------------------------------------------------
                // [Stage 2] 적분항 누적 및 안티와인드업 처리
                //------------------------------------------------------------------
                ST_INTEG: begin
                    if ((error > I_ERR_DEAD) || (error < -I_ERR_DEAD)) begin
                        if (integral + ki_term_reg > I_MAX)
                            integral <= I_MAX;
                        else if (integral + ki_term_reg < I_MIN)
                            integral <= I_MIN;
                        else
                            integral <= integral + ki_term_reg;
                    end
                    else begin
                        // 중심 근처에서는 쌓인 적분값을 천천히 감소시켜 밀림 방지
                        if (integral > 32'sd0)
                            integral <= integral - 32'sd1;
                        else if (integral < 32'sd0)
                            integral <= integral + 32'sd1;
                        else
                            integral <= integral;
                    end
                    state <= ST_SUM;
                end

                //------------------------------------------------------------------
                // [Stage 3] PID 제어 총합 계산
                //------------------------------------------------------------------
                ST_SUM: begin
                    pid_sum_reg <= p_term_reg + integral - d_term_shaped_w;
                    state       <= ST_OUT;
                end

                //------------------------------------------------------------------
                // [Stage 4] 방향 결정 및 Center Hold 판별
                //------------------------------------------------------------------
                ST_OUT: begin
                    hold_prev_reg <= center_hold_reg;
                    center_hold_reg <= center_hold_next_w;

                    if (hold_apply_w) begin
                        // 완벽하게 중심을 잡은 상태: 출력을 0으로 강제
                        pid_out_abs_reg <= 16'sd0;
                        error_abs_reg <= error_abs;
                        pid_active_reg <= 1'b0;
                    end
                    else if (dir) begin
                        // 히스테리시스를 적용하여 방향 전환 시 덜덜거림 방지
                        if (pid_out_used_w > DIR_ENTER_THR) begin
                            dir <= 1'b0;
                            pid_out_abs_reg <= pid_out_used_w;
                            error_abs_reg <= error_abs;
                            pid_active_reg <= 1'b1;
                        end
                        else if (pid_out_used_w < -DIR_EXIT_THR) begin
                            dir <= 1'b1;
                            pid_out_abs_reg <= -pid_out_used_w;
                            error_abs_reg <= error_abs;
                            pid_active_reg <= 1'b1;
                        end
                        else begin
                            dir <= 1'b1;
                            pid_out_abs_reg <= 16'sd0;
                            error_abs_reg <= error_abs;
                            pid_active_reg <= 1'b0;
                        end
                    end
                    else begin
                        if (pid_out_used_w < -DIR_ENTER_THR) begin
                            dir <= 1'b1;
                            pid_out_abs_reg <= -pid_out_used_w;
                            error_abs_reg <= error_abs;
                            pid_active_reg <= 1'b1;
                        end
                        else if (pid_out_used_w > DIR_EXIT_THR) begin
                            dir <= 1'b0;
                            pid_out_abs_reg <= pid_out_used_w;
                            error_abs_reg <= error_abs;
                            pid_active_reg <= 1'b1;
                        end
                        else begin
                            dir <= 1'b0;
                            pid_out_abs_reg <= 16'sd0;
                            error_abs_reg <= error_abs;
                            pid_active_reg <= 1'b0;
                        end
                    end
                    state <= ST_APPLY;
                end

                //------------------------------------------------------------------
                // [Stage 5] 모터 Duty 매핑 및 부스트 적용
                //------------------------------------------------------------------
                ST_APPLY: begin
                    sat_flag <= 1'b0;
                    active_min_applied <= 1'b0;
                    motor_duty_dbg <= 7'd0;
                    motor_duty_sel_reg <= 7'd0;
                    boost_active_dbg <= 1'b0;

                    // 정지 해제 후 일시적으로 부드럽게 토크를 올리는 램프 처리
                    if (just_released_hold_w)
                        hold_release_soft_cnt <= HOLD_RELEASE_SOFT_CYCLES;
                    else if (hold_release_soft_cnt != 3'd0)
                        hold_release_soft_cnt <= hold_release_soft_cnt - 3'd1;

                    // 회전 방향이 안정적으로 유지될 때만 부스트 허용
                    if (!pid_active_reg || hold_apply_w) begin
                        dir_prev_reg <= dir;
                        dir_stable_cnt <= 2'd0;
                    end
                    else if (dir != dir_prev_reg) begin
                        dir_prev_reg <= dir;
                        dir_stable_cnt <= 2'd0;
                    end
                    else if (dir_stable_cnt != 2'b11) begin
                        dir_stable_cnt <= dir_stable_cnt + 2'd1;
                    end

                    if (hold_apply_w) begin
                        boost_hold_cnt <= 3'd0;
                    end
                    else if (pid_active_reg) begin
                        if ((boost_hold_cnt != 3'd0) && boost_req_w) begin
                            sat_flag <= boost_sat_w;
                            active_min_applied <= 1'b1;
                            motor_duty_dbg <= hold_release_apply_w ? boost_after_soft_w : boost_duty_w;
                            motor_duty_sel_reg <= hold_release_apply_w ? boost_after_soft_w : boost_duty_w;
                            boost_active_dbg <= 1'b1;
                            boost_hold_cnt <= boost_hold_cnt - 3'd1;
                        end
                        else begin
                            sat_flag <= sat_flag_w;
                            active_min_applied <= active_min_need_w;
                            motor_duty_dbg <= hold_release_apply_w ? duty_after_soft_w : duty_final_w;
                            motor_duty_sel_reg <= hold_release_apply_w ? duty_after_soft_w : duty_final_w;
                            if (boost_req_w && (START_BOOST_HOLD_CYCLES != 3'd0))
                                boost_hold_cnt <= START_BOOST_HOLD_CYCLES - 3'd1;
                            else
                                boost_hold_cnt <= 3'd0;
                        end
                    end
                    else begin
                        boost_hold_cnt <= 3'd0;
                    end
                    state <= ST_PWM;
                end

                //------------------------------------------------------------------
                // [Stage 6] 최종 PWM 스케일링 (곱셈 대신 Shift-Add 활용)
                //------------------------------------------------------------------
                ST_PWM: begin
                    // duty 값에 10을 곱하여(x8 + x2) 0~1000 범위의 PWM 생성
                    pwm_duty <= {6'd0, motor_duty_sel_reg, 3'b000} + {8'd0, motor_duty_sel_reg, 1'b0};
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule