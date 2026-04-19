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
    output reg         dir
);

    parameter [15:0] MAX_DUTY          = 16'd1000;
    parameter signed [31:0] I_MAX      = 32'sd500000;
    parameter signed [31:0] I_MIN      = -32'sd500000;

    //--------------------------------------------------------------------------
    // wheel velocity damping
    // 바로 세우는 반응을 먼저 보기 위해 기본값은 끈다.
    // 필요하면 1~2 정도로 다시 키워가면 된다.
    //--------------------------------------------------------------------------
    localparam signed [15:0] KV_DAMP = 16'sd0;

    // raw gyro를 D항에 바로 쓰면 너무 커지므로 적당히 축소해서 사용
    // 기존보다 한 단계 더 강하게 반응하도록 조정
    localparam integer GYRO_D_SHIFT = 2;

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
    reg signed [31:0] v_term_reg;
    reg signed [31:0] pid_sum_reg;

    //================================================================================
    // 출력 계산용 조합 논리 (ST_OUT 에서만 사용)
    //================================================================================
    wire signed [31:0] pid_out     = pid_sum_reg >>> 8;
    wire signed [31:0] neg_pid_out = -pid_out;

    // 최소 듀티 보상
    // 기존 4% 수준으로는 모터가 천천히만 반응할 가능성이 커서
    // 기본 킥을 높이고, 기울기가 커질 때는 더 세게 시작한다.
    localparam  [15:0] MIN_DUTY_BASE = 16'd220;
    localparam  [15:0] MIN_DUTY_MID  = 16'd320;
    localparam  [15:0] MIN_DUTY_HIGH = 16'd420;

    // 제자리 잔떨림 방지용 작은 출력 deadband
    localparam signed [31:0] OUT_DEAD = 32'sd4;

    // 적분항을 계속 쌓지 않을 작은 오차 구간
    localparam signed [15:0] I_ERR_DEAD = 16'sd8;

    // 기울기 크기에 따라 시작 듀티를 더 세게 준다.
    localparam signed [15:0] ANGLE_BOOST_MID  = 16'sd80;
    localparam signed [15:0] ANGLE_BOOST_HIGH = 16'sd160;
    wire signed [15:0] error_abs = error[15] ? -error : error;
    wire [15:0] min_duty_eff =
        (error_abs > ANGLE_BOOST_HIGH) ? MIN_DUTY_HIGH :
        (error_abs > ANGLE_BOOST_MID)  ? MIN_DUTY_MID  :
                                         MIN_DUTY_BASE;

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
            v_term_reg  <= 32'sd0;
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
                        state      <= ST_TERMS;
                    end
                end

                //------------------------------------------------------------------
                // [Stage 1] 가장 무거운 곱셈 및 적분 누적
                //------------------------------------------------------------------
                ST_TERMS: begin
                    p_term_reg <= $signed(kp) * error;

                    //------------------------------------------------------------------
                    // D항은 angle 차분 대신 gyro를 직접 사용해서
                    // 로봇이 급하게 넘어질 때 바로 받아치도록 만든다.
                    // top.v에서 angle 부호를 뒤집어 넣고 있으므로 gyro도 같은 부호로 쓴다.
                    //------------------------------------------------------------------
                    d_term_reg <= ($signed(kd) * $signed(gyro_in)) >>> GYRO_D_SHIFT;

                    //------------------------------------------------------------------
                    // 추가:
                    // 속도가 클수록 PID 출력을 줄이는 damping 항
                    // 중심 근처에서 너무 세게 밀어 반대편으로 넘어가는 것을 줄임
                    //------------------------------------------------------------------
                    v_term_reg <= $signed(KV_DAMP) * $signed(vel_in);

                    // 정지 근처에서는 적분항을 더 쌓지 않고,
                    // 이미 쌓인 적분은 천천히 줄여 잔류 출력 때문에
                    // 계속 밀어버리는 현상을 줄인다.
                    if ((error > I_ERR_DEAD) || (error < -I_ERR_DEAD)) begin
                        if (integral + $signed(ki) * error > I_MAX)
                            integral <= I_MAX;
                        else if (integral + $signed(ki) * error < I_MIN)
                            integral <= I_MIN;
                        else
                            integral <= integral + $signed(ki) * error;
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
                // [Stage 2] PID 항 합산
                //------------------------------------------------------------------
                ST_SUM: begin
                    // 기존 PID 합에 velocity damping 항을 뺀다.
                    // vel_in 방향으로 속도가 커질수록 출력이 줄어드는 구조
                    pid_sum_reg <= p_term_reg + integral + d_term_reg - v_term_reg;
                    state       <= ST_OUT;
                end

                //------------------------------------------------------------------
                // [Stage 3] 스케일링(>>8) 및 출력 클램핑 + 데드존 보상
                //------------------------------------------------------------------
                ST_OUT: begin
                    // 작은 출력은 deadband 안으로 보고 모터를 멈춘다.
                    // 그래야 미세 오차/센서 잡음/바닥 기울기 때문에
                    // 계속 최소 듀티로 밀어버리는 현상을 줄일 수 있다.
                    if (pid_out > OUT_DEAD) begin
                        dir <= 1'b0;    // 전진

                        if (pid_out + $signed({1'b0, min_duty_eff}) > $signed({1'b0, MAX_DUTY}))
                            pwm_duty <= MAX_DUTY;
                        else
                            pwm_duty <= pid_out[15:0] + min_duty_eff;
                    end
                    else if (pid_out < -OUT_DEAD) begin
                        dir <= 1'b1;    // 후진

                        if (neg_pid_out + $signed({1'b0, min_duty_eff}) > $signed({1'b0, MAX_DUTY}))
                            pwm_duty <= MAX_DUTY;
                        else
                            pwm_duty <= neg_pid_out[15:0] + min_duty_eff;
                    end
                    else begin
                        // deadband 안에서는 정지
                        dir <= 1'b0;
                        pwm_duty <= 16'd0;
                    end

                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule

// `timescale 1ns / 1ps
// //================================================================================
// // pid -> 균형 로봇을 위한 PID 제어 모듈 (FSM 기반 타이밍 최적화 버전)
// //================================================================================

// module pid
// (
//     input  wire        clk,
//     input  wire        rst_n,
//     input  wire        en,                      // 10ms 주기 업데이트 트리거

//     input  wire signed [15:0] angle_in,
//     input  wire signed [15:0] setpoint,

//     input  wire [15:0] kp,
//     input  wire [15:0] ki,
//     input  wire [15:0] kd,

//     output reg  [15:0] pwm_duty,
//     output reg         dir
// );

//     parameter [15:0] MAX_DUTY          = 16'd1000;
//     parameter signed [31:0] I_MAX      = 32'sd500000;
//     parameter signed [31:0] I_MIN      = -32'sd500000;

//     //================================================================================
//     // FSM 상태 정의
//     //================================================================================
//     localparam [2:0]
//         ST_IDLE   = 3'd0,
//         ST_TERMS  = 3'd1,
//         ST_SUM    = 3'd2,
//         ST_OUT    = 3'd3;

//     reg [2:0] state;

//     //================================================================================
//     // 내부 상태 레지스터
//     //================================================================================
//     reg signed [15:0] error;
//     reg signed [15:0] prev_error;
//     reg signed [31:0] integral;

//     // 중간 연산 보관 레지스터
//     reg signed [31:0] p_term_reg;
//     reg signed [31:0] d_term_reg;
//     reg signed [31:0] pid_sum_reg;

//     //================================================================================
//     // 출력 계산용 조합 논리 (ST_OUT 에서만 사용)
//     //================================================================================
//     wire signed [31:0] pid_out     = pid_sum_reg >>> 8;
//     wire signed [31:0] neg_pid_out = -pid_out;

//     // 수정 전
//     // localparam  [15:0] MIN_DUTY    = 16'd40;
//     // localparam signed [31:0] OUT_DEAD = 32'sd10;

//     // 수정 후
//     // 최소 듀티 보상
//     localparam  [15:0] MIN_DUTY    = 16'd40;

//     // 제자리 잔떨림 방지용 작은 출력 deadband
//     // 기존 10보다 조금 키워서 작은 오차에는 아예 정지시키는 방향
//     localparam signed [31:0] OUT_DEAD = 32'sd18;

//     // 적분항을 계속 쌓지 않을 작은 오차 구간
//     // 정지 근처에서 적분 때문에 계속 미는 현상을 줄이기 위한 값
//     localparam signed [15:0] I_ERR_DEAD = 16'sd8;

//     //================================================================================
//     // FSM PID 로직
//     //================================================================================
//     always @(posedge clk or negedge rst_n) begin
//         if (!rst_n) begin
//             state       <= ST_IDLE;
//             error       <= 16'sd0;
//             prev_error  <= 16'sd0;
//             integral    <= 32'sd0;
            
//             p_term_reg  <= 32'sd0;
//             d_term_reg  <= 32'sd0;
//             pid_sum_reg <= 32'sd0;

//             pwm_duty    <= 16'd0;
//             dir         <= 1'b0;
//         end
//         else begin
//             case (state)
//                 //------------------------------------------------------------------
//                 // [Stage 0] 오차 갱신 및 대기
//                 //------------------------------------------------------------------
//                 ST_IDLE: begin
//                     if (en) begin
//                         prev_error <= error;
//                         error      <= setpoint - angle_in;
//                         state      <= ST_TERMS; // 다음 연산 단계로 이동
//                     end
//                 end

//                 //------------------------------------------------------------------
//                 // [Stage 1] 가장 무거운 곱셈 및 적분 누적
//                 //------------------------------------------------------------------
//                 ST_TERMS: begin
//                     p_term_reg <= $signed(kp) * error;
//                     d_term_reg <= $signed(kd) * (error - prev_error);

//                     // 수정 전
//                     // if (integral + $signed(ki) * error > I_MAX)
//                     //     integral <= I_MAX;
//                     // else if (integral + $signed(ki) * error < I_MIN)
//                     //     integral <= I_MIN;
//                     // else
//                     //     integral <= integral + $signed(ki) * error;

//                     // 수정 후
//                     // 정지 근처에서는 적분항을 더 쌓지 않고,
//                     // 이미 쌓인 적분은 천천히 줄여서 잔류 출력 때문에
//                     // 계속 밀어버리는 현상을 줄인다.
//                     if ((error > I_ERR_DEAD) || (error < -I_ERR_DEAD)) begin
//                         if (integral + $signed(ki) * error > I_MAX)
//                             integral <= I_MAX;
//                         else if (integral + $signed(ki) * error < I_MIN)
//                             integral <= I_MIN;
//                         else
//                             integral <= integral + $signed(ki) * error;
//                     end
//                     else begin
//                         // 작은 오차 구간에서는 적분항을 천천히 원점으로 복귀
//                         if (integral > 32'sd0)
//                             integral <= integral - 32'sd1;
//                         else if (integral < 32'sd0)
//                             integral <= integral + 32'sd1;
//                         else
//                             integral <= integral;
//                     end

//                     state <= ST_SUM;
//                 end

//                 //------------------------------------------------------------------
//                 // [Stage 2] PID 항 합산
//                 //------------------------------------------------------------------
//                 ST_SUM: begin
//                     pid_sum_reg <= p_term_reg + integral + d_term_reg;
//                     state       <= ST_OUT;
//                 end

//                 //------------------------------------------------------------------
//                 // [Stage 3] 스케일링(>>8) 및 출력 클램핑 + 데드존 보상
//                 //------------------------------------------------------------------
//                 ST_OUT: begin
//                     // 작은 출력은 deadband 안으로 보고 모터를 멈춘다.
//                     // 그래야 미세 오차/센서 잡음/바닥 기울기 때문에
//                     // 계속 최소 듀티로 밀어버리는 현상을 줄일 수 있다.
//                     if (pid_out > OUT_DEAD) begin
//                         dir <= 1'b0;    // 전진

//                         if (pid_out + $signed({1'b0, MIN_DUTY}) > $signed({1'b0, MAX_DUTY}))
//                             pwm_duty <= MAX_DUTY;
//                         else
//                             pwm_duty <= pid_out[15:0] + MIN_DUTY;
//                     end
//                     else if (pid_out < -OUT_DEAD) begin
//                         dir <= 1'b1;    // 후진

//                         if (neg_pid_out + $signed({1'b0, MIN_DUTY}) > $signed({1'b0, MAX_DUTY}))
//                             pwm_duty <= MAX_DUTY;
//                         else
//                             pwm_duty <= neg_pid_out[15:0] + MIN_DUTY;
//                     end
//                     else begin
//                         // deadband 안에서는 정지
//                         dir <= 1'b0;
//                         pwm_duty <= 16'd0;
//                     end

//                     state <= ST_IDLE;
//                 end
    
//                 default: state <= ST_IDLE;
//             endcase
//         end
//     end

// endmodule
