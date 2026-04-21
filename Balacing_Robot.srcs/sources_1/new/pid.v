`timescale 1ns / 1ps
//================================================================================
// pid -> 균형 로봇을 위한 PID 제어 모듈 (FSM 기반 타이밍 최적화 버전)
//        + velocity damping 추가
//================================================================================

module pid
(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        en,                      // 5ms 주기 업데이트 트리거
    input  wire        i_clear,                 // 적분 초기화 (1클럭 펄스)

    input  wire signed [15:0] angle_in,
    input  wire signed [15:0] setpoint,

    //--------------------------------------------------------------------------
    // 추가:
    // 전진/후진 축 속도 피드백
    // 큰 외란에서 중심을 통과할 때 속도를 줄여
    // 반대편으로 다시 넘어가는 오버슈트를 줄이기 위한 입력
    //--------------------------------------------------------------------------
    input  wire signed [15:0] vel_in,
    input  wire signed [15:0] gyro_in,

    input  wire [15:0] kp,
    input  wire [15:0] ki,
    input  wire [15:0] kd,

    output reg  [15:0] pwm_duty,
    output reg         dir,
    output wire signed [15:0] pid_out_dbg,
    output reg         sat_flag,
    output reg         active_min_applied,
    output reg  [6:0]  motor_duty_dbg,
    output reg         boost_active_dbg,
    output wire        center_hold_dbg
);

    parameter signed [31:0] I_MAX      = 32'sd500000;
    parameter signed [31:0] I_MIN      = -32'sd500000;
    // Tune these against the actual JGB37-520 drivetrain.
    parameter [6:0] DUTY_ACTIVE_MIN    = 7'd20;
    parameter [6:0] DUTY_MAX_MOTOR     = 7'd75;
    parameter signed [31:0] OUT_DEAD   = 32'sd4;
    parameter signed [15:0] GYRO_D_DEAD = 16'sd96;
    parameter signed [15:0] GYRO_D_LIM  = 16'sd1024;
    parameter signed [15:0] PID_OUT_CLAMP = 16'sd900;
    parameter signed [31:0] D_TERM_SUM_CLAMP = 32'sd30720;
    parameter signed [15:0] MAP_SMALL_THR = 16'sd160;
    parameter signed [15:0] MAP_MID_THR   = 16'sd260;
    parameter signed [15:0] SMALL_MIN_OUT_THR = 16'sd56;
    // Center-hold lock state:
    // once the robot is both near upright and nearly stationary, force the
    // motor command to zero so residual PWM does not keep exciting oscillation.
    // Hysteresis is required so the controller does not chatter between
    // "hold" and "recover" on every small sensor fluctuation.
    parameter signed [15:0] HOLD_ENTER_THR = 16'sd6;
    parameter signed [15:0] HOLD_EXIT_THR  = 16'sd10;
    parameter signed [15:0] SMALL_VEL_THR  = 16'sd8;
    parameter signed [15:0] HOLD_EXIT_VEL_THR = 16'sd8;
    // Direction hysteresis reduces rapid forward/reverse flipping when the
    // controller is close to zero and the signed PID output dithers.
    parameter signed [15:0] DIR_ENTER_THR = 16'sd4;
    parameter signed [15:0] DIR_EXIT_THR  = 16'sd2;
    // Only allow boost once the robot is clearly away from center and the
    // wheel axis still looks close to rest.
    parameter signed [15:0] BOOST_ERR_MIN      = 16'sd96;
    parameter signed [15:0] BOOST_REST_VEL_THR = 16'sd6;
    parameter [1:0]         BOOST_DIR_STABLE_CYCLES = 2'd2;
    // Apply the strongest velocity damping only near center where overshoot
    // is most harmful. At larger angles it is reduced so recovery stays strong.
    parameter signed [15:0] VEL_DAMP_FULL_ERR_THR = 16'sd128;
    // After center hold releases, keep the next couple of control updates
    // gentle so recovery ramps up instead of kicking immediately.
    parameter [2:0]         HOLD_RELEASE_SOFT_CYCLES = 3'd4;
    parameter [6:0]         HOLD_RELEASE_DUTY_MAX    = 7'd16;
    // Start boost helps the 12V JGB37-520 overcome static friction.
    // Tune with care: too large or too long will make the robot kick too hard.
    parameter        START_BOOST_ENABLE      = 1'b1;
    parameter [2:0]  START_BOOST_HOLD_CYCLES = 3'd1;
    parameter [6:0]  START_BOOST_DUTY        = 7'd22;

    //--------------------------------------------------------------------------
    // wheel velocity damping
    // 바로 세우는 반응을 먼저 보기 위해 기본값은 끈다.
    // 필요하면 1~2 정도로 다시 키워가면 된다.
    //--------------------------------------------------------------------------
    localparam signed [15:0] KV_DAMP = 16'sd0;
    // These local polarity options let us verify velocity / gyro sign safely
    // without changing module ports. Defaults preserve current top-level wiring.
    localparam                VEL_SIGN_INV  = 1'b0;
    localparam                GYRO_SIGN_INV = 1'b0;

    // D항은 P300~P400 영역을 망치지 않도록 더 보수적으로 사용한다.
    // 작은 D 값이 곧바로 saturation을 만들지 않게 추가 축소를 둔다.
    localparam integer GYRO_D_SHIFT = 2;
    localparam integer D_TERM_POST_SHIFT = 2;

    //================================================================================
    // FSM 상태 정의
    //================================================================================
    localparam [2:0]
        ST_IDLE   = 3'd0,
        ST_TERMS  = 3'd1,
        ST_INTEG  = 3'd2,
        ST_SUM    = 3'd3,
        ST_OUT    = 3'd4,
        ST_APPLY  = 3'd5,
        ST_PWM    = 3'd6;

    reg [2:0] state;

    //================================================================================
    // 내부 상태 레지스터
    //================================================================================
    reg signed [15:0] error;
    reg signed [15:0] prev_error;
    reg signed [31:0] integral;

    // 중간 연산 보관 레지스터
    reg signed [31:0] p_term_reg;
    reg signed [31:0] ki_term_reg;
    reg signed [31:0] d_term_reg;
    reg signed [31:0] v_term_reg;
    reg signed [31:0] pid_sum_reg;
    reg [6:0]         motor_duty_sel_reg;
    reg signed [15:0] pid_out_abs_reg;
    reg signed [15:0] error_abs_reg;
    reg signed [15:0] gyro_d_reg;
    reg signed [15:0] vel_abs_reg;
    reg               vel_same_dir_reg;
    reg               pid_active_reg;
    reg               center_hold_reg;
    reg               hold_prev_reg;
    reg               dir_prev_reg;
    reg [1:0]         dir_stable_cnt;
    reg [2:0]         hold_release_soft_cnt;

    //================================================================================
    // 출력 계산용 조합 논리 (ST_OUT 에서만 사용)
    //================================================================================
    wire signed [31:0] pid_out     = pid_sum_reg >>> 8;
    // angle_in must already be in the single corrected control frame.
    // Do not re-apply any captured offset inside pid.v.
    wire signed [15:0] error_next_w = setpoint - angle_in;
    // Keep sign handling explicit so D/velocity damping can be verified
    // without silently relying on top-level negations.
    wire signed [15:0] gyro_aligned_w = GYRO_SIGN_INV ? -gyro_in : gyro_in;
    wire signed [15:0] vel_aligned_w  = VEL_SIGN_INV  ? -vel_in  : vel_in;
    wire signed [15:0] gyro_abs_w = gyro_aligned_w[15] ? -gyro_aligned_w : gyro_aligned_w;
    wire signed [15:0] vel_abs_w  = vel_aligned_w[15]  ? -vel_aligned_w  : vel_aligned_w;
    wire signed [15:0] gyro_d_used_w =
        (gyro_abs_w <= GYRO_D_DEAD) ? 16'sd0 :
        (gyro_aligned_w >  GYRO_D_LIM) ? GYRO_D_LIM :
        (gyro_aligned_w < -GYRO_D_LIM) ? -GYRO_D_LIM :
                                         gyro_aligned_w;
    wire signed [31:0] d_term_used_w =
        (d_term_reg >  D_TERM_SUM_CLAMP) ?  D_TERM_SUM_CLAMP :
        (d_term_reg < -D_TERM_SUM_CLAMP) ? -D_TERM_SUM_CLAMP :
                                           d_term_reg;
    wire signed [31:0] d_term_shaped_w = d_term_used_w >>> D_TERM_POST_SHIFT;
    wire signed [31:0] v_term_used_w =
        !vel_same_dir_reg ? 32'sd0 :
        (error_abs_reg <= VEL_DAMP_FULL_ERR_THR) ? v_term_reg :
                                                   (v_term_reg >>> 1);
    wire signed [15:0] pid_out_used_w =
        (pid_out >  PID_OUT_CLAMP) ?  PID_OUT_CLAMP :
        (pid_out < -PID_OUT_CLAMP) ? -PID_OUT_CLAMP :
                                     pid_out[15:0];
    wire pid_out_clamped_w = (pid_out > PID_OUT_CLAMP) || (pid_out < -PID_OUT_CLAMP);

    // 적분항을 계속 쌓지 않을 작은 오차 구간
    localparam signed [15:0] I_ERR_DEAD = 16'sd8;
    wire signed [15:0] error_abs = error[15] ? -error : error;
    // Keep the near-zero region quiet, but make the practical recovery range
    // noticeably stronger by steepening the existing small/mid mapping.
    // The large-error region stays compressed so max duty is reached later.
    wire [15:0] mid_delta_w   = pid_out_abs_reg[15:0] - MAP_SMALL_THR[15:0];
    wire [15:0] large_delta_w = pid_out_abs_reg[15:0] - MAP_MID_THR[15:0];
    wire [6:0] duty_small_w =
        (pid_out_abs_reg[15:0] >> 4);
    // Keep the existing piecewise shape, but let small/mid/large each cover
    // a different part of the stand-up problem:
    // - small : quiet near center
    // - mid   : smooth practical recovery
    // - large : strong recovery without an immediate jump to max duty
    wire [6:0] duty_mid_w =
        7'd24 + (mid_delta_w >> 10);
    wire [6:0] duty_large_w =
        7'd52 + (large_delta_w >> 6);
    wire [6:0] duty_piecewise_w =
        (pid_out_abs_reg <= MAP_SMALL_THR) ? duty_small_w :
        (pid_out_abs_reg <= MAP_MID_THR)   ? duty_mid_w   :
                                             duty_large_w;
    // Replace the old hard 0->ACTIVE_MIN jump with a gradual floor ramp.
    // This removes the sharp duty step that was kicking the robot through center.
    wire [15:0] active_min_delta_w =
        (pid_out_abs_reg > SMALL_MIN_OUT_THR) ? (pid_out_abs_reg - SMALL_MIN_OUT_THR) : 16'd0;
    wire [8:0] active_min_ramp_ext_w = {3'b000, active_min_delta_w[15:6]};
    wire [6:0] active_min_ramp_w =
        active_min_ramp_ext_w[8] ? DUTY_ACTIVE_MIN :
        (active_min_ramp_ext_w[6:0] > DUTY_ACTIVE_MIN) ? DUTY_ACTIVE_MIN :
                                                         active_min_ramp_ext_w[6:0];
    wire active_min_need_w =
        (pid_out_abs_reg >= SMALL_MIN_OUT_THR) &&
        (error_abs_reg > HOLD_EXIT_THR) &&
        (active_min_ramp_w > duty_piecewise_w);
    wire [6:0] duty_pre_sat_w =
        active_min_need_w ? active_min_ramp_w : duty_piecewise_w;
    wire sat_flag_w = (duty_pre_sat_w > DUTY_MAX_MOTOR) || pid_out_clamped_w;
    wire [6:0] duty_final_w = sat_flag_w ? DUTY_MAX_MOTOR : duty_pre_sat_w;
    // Boost is only useful when starting from near rest and holding a stable
    // direction for a few control ticks. This suppresses repeated kick-like
    // boost pulses during standing oscillation.
    wire boost_rest_w = (vel_abs_reg <= BOOST_REST_VEL_THR);
    wire boost_dir_stable_w = (dir_stable_cnt >= BOOST_DIR_STABLE_CYCLES);
    wire boost_req_w =
        START_BOOST_ENABLE &&
        active_min_need_w &&
        (error_abs_reg >= BOOST_ERR_MIN) &&
        boost_rest_w &&
        boost_dir_stable_w;
    wire [6:0] boost_floor_w =
        (START_BOOST_DUTY > duty_final_w) ? START_BOOST_DUTY : duty_final_w;
    wire boost_sat_w = (boost_floor_w > DUTY_MAX_MOTOR);
    wire [6:0] boost_duty_w = boost_sat_w ? DUTY_MAX_MOTOR : boost_floor_w;
    // Hold entry uses both angle and velocity so we only lock when the robot
    // is upright *and* already almost stationary. Exit also uses velocity so
    // the controller releases immediately when the robot starts falling.
    wire hold_enter_w =
        (error_abs < HOLD_ENTER_THR) &&
        (vel_abs_reg < SMALL_VEL_THR);
    wire hold_exit_w =
        (error_abs > HOLD_EXIT_THR) ||
        (vel_abs_reg > HOLD_EXIT_VEL_THR);
    wire center_hold_next_w = center_hold_reg ? !hold_exit_w : hold_enter_w;
    // Apply the current-cycle hold decision immediately to the output logic,
    // while still storing the state in center_hold_reg for hysteresis memory.
    wire hold_apply_w = center_hold_next_w;
    wire just_released_hold_w = hold_prev_reg && !hold_apply_w;
    wire hold_release_soft_w = (hold_release_soft_cnt != 3'd0);
    wire hold_release_apply_w = hold_release_soft_w || just_released_hold_w;
    wire [6:0] duty_after_soft_w =
        (duty_final_w > HOLD_RELEASE_DUTY_MAX) ? HOLD_RELEASE_DUTY_MAX : duty_final_w;
    wire [6:0] boost_after_soft_w =
        (boost_duty_w > HOLD_RELEASE_DUTY_MAX) ? HOLD_RELEASE_DUTY_MAX : boost_duty_w;
    assign pid_out_dbg =
        (pid_out >  32'sd32767) ? 16'sd32767 :
        (pid_out < -32'sd32768) ? -16'sd32768 :
        pid_out[15:0];
    assign center_hold_dbg = center_hold_reg;

    reg [2:0] boost_hold_cnt;

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
            ki_term_reg <= 32'sd0;
            d_term_reg  <= 32'sd0;
            v_term_reg  <= 32'sd0;
            pid_sum_reg <= 32'sd0;
            motor_duty_sel_reg <= 7'd0;
            pid_out_abs_reg <= 16'sd0;
            error_abs_reg <= 16'sd0;
            gyro_d_reg <= 16'sd0;
            vel_abs_reg <= 16'sd0;
            vel_same_dir_reg <= 1'b0;
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
            if (i_clear) begin
                integral    <= 32'sd0;
                prev_error  <= 16'sd0;
                state       <= ST_IDLE;
            end
            case (state)
                //------------------------------------------------------------------
                // [Stage 0] 오차 갱신 및 대기
                //------------------------------------------------------------------
                ST_IDLE: begin
                    if (en) begin
                        prev_error <= error;
                        error      <= error_next_w;
                        gyro_d_reg <= gyro_d_used_w;
                        vel_abs_reg <= vel_abs_w;
                        // Only damp when wheel motion already matches the
                        // requested correction direction. Otherwise the outer
                        // velocity signal can fight recovery instead of damping.
                        vel_same_dir_reg <=
                            (error_next_w != 16'sd0) &&
                            (vel_aligned_w != 16'sd0) &&
                            (error_next_w[15] == vel_aligned_w[15]);
                        state      <= ST_TERMS;
                    end
                end

                //------------------------------------------------------------------
                // [Stage 1] 가장 무거운 곱셈 및 적분 누적
                //------------------------------------------------------------------
                ST_TERMS: begin
                    p_term_reg <= $signed(kp) * error;
                    ki_term_reg <= $signed(ki) * error;

                    //------------------------------------------------------------------
                    // D항은 angle 차분 대신 gyro를 직접 사용해서
                    // 로봇이 급하게 넘어질 때 바로 받아치도록 만든다.
                    // top.v에서 angle 부호를 뒤집어 넣고 있으므로 gyro도 같은 부호로 쓴다.
                    //------------------------------------------------------------------
                    d_term_reg <= ($signed(kd) * $signed(gyro_d_reg)) >>> GYRO_D_SHIFT;

                    //------------------------------------------------------------------
                    // 추가:
                    // 속도가 클수록 PID 출력을 줄이는 damping 항
                    // 중심 근처에서 너무 세게 밀어 반대편으로 넘어가는 것을 줄임
                    //------------------------------------------------------------------
                    // Store damping magnitude only. Sign handling is done
                    // structurally in ST_SUM so damping can never become
                    // positive feedback by accident.
                    v_term_reg <= $signed(KV_DAMP) * $signed(vel_abs_reg);

                    // 정지 근처에서는 적분항을 더 쌓지 않고,
                    // 이미 쌓인 적분은 천천히 줄여 잔류 출력 때문에
                    // 계속 밀어버리는 현상을 줄인다.
                    state <= ST_INTEG;
                end

                //------------------------------------------------------------------
                // [Stage 2] 적분항 누적 / 감쇠 분리
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
                        // 작은 오차 구간에서는 적분항을 천천히 원점으로 복귀
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
                // [Stage 3] PID 항 합산
                //------------------------------------------------------------------
                ST_SUM: begin
                    // D-term and velocity feedback are both applied as damping.
                    // If either sign path is wrong, adding them would amplify
                    // motion. Subtracting the shaped terms makes the intent explicit.
                    pid_sum_reg <= p_term_reg + integral - d_term_shaped_w - v_term_used_w;
                    state       <= ST_OUT;
                end

                //------------------------------------------------------------------
                // [Stage 4] 출력 방향/절대값 분리
                //------------------------------------------------------------------
                ST_OUT: begin
                    hold_prev_reg <= center_hold_reg;
                    // Single source of truth for hold:
                    // update the lock state only here, from the current-cycle
                    // latched error/velocity signals.
                    center_hold_reg <= center_hold_next_w;

                    // Center hold overrides direction selection, active-min and
                    // boost logic. This intentionally outputs zero torque in the
                    // upright/stationary region to kill micro-oscillation.
                    if (hold_apply_w) begin
                        pid_out_abs_reg <= 16'sd0;
                        error_abs_reg <= error_abs;
                        pid_active_reg <= 1'b0;
                    end
                    // Direction hysteresis prevents rapid 0/1 toggling when the
                    // signed PID result dithers around zero during standing hold.
                    else if (dir) begin
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
                // [Stage 5] duty 선택 / boost / saturation
                //------------------------------------------------------------------
                ST_APPLY: begin
                    sat_flag <= 1'b0;
                    active_min_applied <= 1'b0;
                    motor_duty_dbg <= 7'd0;
                    motor_duty_sel_reg <= 7'd0;
                    boost_active_dbg <= 1'b0;

                    // Hold exit should recover immediately, but not with a kick.
                    // Clamp the first 1~2 commands after hold release so duty
                    // ramps back in smoothly.
                    if (just_released_hold_w)
                        hold_release_soft_cnt <= HOLD_RELEASE_SOFT_CYCLES;
                    else if (hold_release_soft_cnt != 3'd0)
                        hold_release_soft_cnt <= hold_release_soft_cnt - 3'd1;

                    // Track whether the requested direction has stayed stable.
                    // Boost is armed only after the command stops dithering.
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

                    // 1) dead zone -> duty 0
                    // 2) small error -> quiet piecewise duty
                    // 3) if that is still too weak, smoothly ramp a floor duty
                    // 4) only from rest and with stable direction, allow one-shot boost
                    // 5) clamp at DUTY_MAX_MOTOR
                    if (hold_apply_w) begin
                        // Hard override: no active-min ramp, no boost, no duty.
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
                // [Stage 6] 최종 PWM duty 생성
                // shift-add로 x10을 만들어 곱셈 경로를 피한다.
                //------------------------------------------------------------------
                ST_PWM: begin
                    pwm_duty <= {6'd0, motor_duty_sel_reg, 3'b000} +
                                {8'd0, motor_duty_sel_reg, 1'b0};
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
