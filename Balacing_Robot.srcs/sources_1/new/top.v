
`timescale 1ns / 1ps

module top
(
    input  wire        clk,
    input  wire        reset,   // active-high (BTNC, U18)

    inout  wire        i2c_sda,
    inout  wire        i2c_scl,

    input  wire        uart_rx,      // PC → FPGA (USB-UART RX, B18)
    output wire        uart_tx,

    // TB6612FNG 모터 드라이버 출력
    output wire        PWMA_out,
    output wire        AIN1_out,
    output wire        AIN2_out,
    output wire        PWMB_out,
    output wire        BIN1_out,
    output wire        BIN2_out,
    output wire        STBY_out,       // TB6612FNG STBY (SW0 ON=활성, OFF=정지)

    // STBY 제어 스위치
    input  wire        sw_stby,        // SW0 (V17)

    // AB 홀 엔코더 입력
    input  wire        encA_a,          // 모터 A 채널 A (JC1, K17)
    input  wire        encA_b,          // 모터 A 채널 B (JC2, M18)
    input  wire        encB_a,          // 모터 B 채널 A (JC3, N17)
    input  wire        encB_b           // 모터 B 채널 B (JC4, P18)
);

    // active-low reset (내부 로직용)
    wire rst_n = ~reset;

    //--------------------------------------------------------------------------
    // 공통 tick
    //--------------------------------------------------------------------------
    wire         tick;

    //--------------------------------------------------------------------------
    // mpu6050_ctrl <-> i2c_master 제어 신호
    //--------------------------------------------------------------------------
    wire         start_req;
    wire         rw;
    wire [7:0]   reg_addr;
    wire [7:0]   tx_data;
    wire [7:0]   burst_len;

    //--------------------------------------------------------------------------
    // i2c_master 상태 신호
    //--------------------------------------------------------------------------
    wire         busy;
    wire         done;
    wire         ack_ok;
    wire [2:0]   err_code;
    wire [7:0]   rx_data;
    wire [127:0] rx_buf;
    wire [7:0]   rx_count;

    //--------------------------------------------------------------------------
    // open-drain 제어
    //--------------------------------------------------------------------------
    wire         sda_enable;
    wire         scl_enable;

    //--------------------------------------------------------------------------
    // MPU raw 데이터
    //--------------------------------------------------------------------------
    wire signed [15:0] ax_raw;
    wire signed [15:0] ay_raw;
    wire signed [15:0] az_raw;
    wire signed [15:0] gx_raw;
    wire signed [15:0] gy_raw;
    wire signed [15:0] gz_raw;

    //--------------------------------------------------------------------------
    // 보정 후 데이터
    //--------------------------------------------------------------------------
    reg                bias_done;

    reg signed [15:0]  ax_bias;
    reg signed [15:0]  ay_bias;
    reg signed [15:0]  az_bias;
    reg signed [15:0]  gx_bias;
    reg signed [15:0]  gy_bias;
    reg signed [15:0]  gz_bias;

    reg signed [15:0]  ax_corr;
    reg signed [15:0]  ay_corr;
    reg signed [15:0]  az_corr;
    reg signed [15:0]  gx_corr;
    reg signed [15:0]  gy_corr;
    reg signed [15:0]  gz_corr;

    //--------------------------------------------------------------------------
    // bias 계산용 누적합
    //--------------------------------------------------------------------------
    reg [7:0]          cal_cnt;
    reg signed [24:0]  ax_sum;
    reg signed [24:0]  ay_sum;
    reg signed [24:0]  az_sum;
    reg signed [24:0]  gx_sum;
    reg signed [24:0]  gy_sum;
    reg signed [24:0]  gz_sum;

    //--------------------------------------------------------------------------
    // 상위 상태
    //--------------------------------------------------------------------------
    wire               init_done;
    wire               data_valid;
    wire [2:0]         last_err;

    //--------------------------------------------------------------------------
    // UART 출력선
    //--------------------------------------------------------------------------
    wire               uart_tx_wire;

    //--------------------------------------------------------------------------
    // 각도 출력
    //--------------------------------------------------------------------------
    wire signed [15:0] angle;        // 0.01° 단위 (raw)
    wire               angle_valid;

    // S 커맨드로 캡처한 균형 기준각 (초기값 0)
    reg signed [15:0]  angle_offset;
    // 보정된 각도: S 눌렀을 때 0이 됨
    wire signed [15:0] angle_adj = angle - angle_offset;

    //--------------------------------------------------------------------------
    // PID 출력 / 모터 제어 신호
    //--------------------------------------------------------------------------
    wire [15:0]  pwm_duty;           // PID 출력 (0~1000)
    wire         pid_dir;            // PID 방향 (0=전진, 1=후진)
    wire         pid_en;             // PID 업데이트 트리거

    // pwm_duty(0~1000) → dutyA/B(0~100) 변환
    wire [6:0]   motor_duty;
    // pid_dir(1bit) → dirA/B_cmd(2bit) 변환 (CW=2'b10, CCW=2'b01)
    wire [1:0]   motor_dir_cmd;

    //--------------------------------------------------------------------------
    // 엔코더 출력
    //--------------------------------------------------------------------------
    wire signed [31:0] enc_pos_a, enc_pos_b;
    wire signed [15:0] enc_vel_a, enc_vel_b;

    //--------------------------------------------------------------------------
    // 엔코더 외부 루프 (위치 유지)
    //--------------------------------------------------------------------------
    reg signed [31:0] enc_ref_a, enc_ref_b;   // S 커맨드 시 기준 위치 캡처
    reg        [15:0] kv_reg;                  // 외부 루프 이득 (기본 0=비활성)

    //--------------------------------------------------------------------------
    // 런타임 PID 이득값 레지스터 (UART RX로 업데이트)
    // 기본값: KP=256(1.0), KI=26(≈0.1), KD=512(2.0) — Q8 포맷
    //--------------------------------------------------------------------------
    reg [15:0]        kp_reg;
    reg [15:0]        ki_reg;
    reg [15:0]        kd_reg;
    reg signed [15:0] setpoint_reg;  // PID 목표 각도 (0.01° 단위, 기본 0)

    //--------------------------------------------------------------------------
    // UART RX 파서용 신호
    //--------------------------------------------------------------------------
    wire         rx_done;            // uart_rx에서 1바이트 수신 완료
    wire [7:0]   rx_byte;            // 수신된 바이트

    // 파서 FSM 상태
    localparam [1:0] RX_IDLE  = 2'd0,
                     RX_DIGIT = 2'd1;
    reg [1:0]    rx_state;
    reg [2:0]    rx_cmd;             // 0=KP, 1=KI, 2=KD, 3=Setpoint
    reg [15:0]   rx_acc;             // 수신 중 숫자 누적 레지스터
    reg [3:0]    rx_cnt;             // 수신 자릿수 카운터 (최대 5자리)
    reg          rx_sign;            // 1 = 음수 (앞에 '-' 수신됨)

    //--------------------------------------------------------------------------
    // 엔코더 외부 루프: 위치 오차 → 각도 setpoint 보정
    // enc_pos_err = (pos_a - ref_a) + (pos_b - ref_b) 합산
    // enc_correction (0.01° 단위) = enc_pos_err * kv_reg (양수→앞으로 간 경우 뒤로 기움)
    //--------------------------------------------------------------------------
    // 1. 단순 뺄셈 연산 (조합 논리 유지)
    wire signed [31:0] enc_delta_a = enc_pos_a - enc_ref_a;
    wire signed [31:0] enc_delta_b = enc_pos_b - enc_ref_b;

    // -------------------------------------------------------------------------
    // 파이프라인 레지스터 선언
    // -------------------------------------------------------------------------
    reg signed [31:0] enc_pos_err_reg;
    reg signed [31:0] enc_corr_q8_reg;
    reg signed [15:0] pid_setpoint;

    // -------------------------------------------------------------------------
    // 타이밍 에러 해결을 위한 3-Stage 파이프라인 연산
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            enc_pos_err_reg <= 32'sd0;
            enc_corr_q8_reg <= 32'sd0;
            pid_setpoint    <= 16'sd0;
        end else begin
            // [Stage 1] 위치 오차 뺄셈
            enc_pos_err_reg <= enc_delta_a - enc_delta_b;

            // [Stage 2] 클램핑 및 KV 게인 곱셈
            enc_corr_q8_reg <= $signed(
                (enc_pos_err_reg >  32'sd32767) ? 16'sd32767 :
                (enc_pos_err_reg < -32'sd32768) ? -16'sd32768 :
                enc_pos_err_reg[15:0]
            ) * $signed({1'b0, kv_reg});

            // [Stage 3] 보정값 2차 클램핑 및 Setpoint 최종 덧셈
            pid_setpoint <= setpoint_reg - (
                (enc_corr_q8_reg >  32'sd128000) ? 16'sd500 :
                (enc_corr_q8_reg < -32'sd128000) ? -16'sd500 :
                enc_corr_q8_reg[23:8]
            );
        end
    end

    //--------------------------------------------------------------------------
    // 방향 판단 threshold
    //--------------------------------------------------------------------------
    localparam signed [15:0] TILT_TH = 16'sd1000;
    localparam signed [15:0] Z_TH    = 16'sd12000;

    //--------------------------------------------------------------------------
    // 방향 표시 신호
    //--------------------------------------------------------------------------
    wire dir_front;
    wire dir_back;
    wire dir_left;
    wire dir_right;
    wire dir_up;
    wire dir_down;

    //--------------------------------------------------------------------------
    // PID 트리거: angle_valid & init_done & bias_done 모두 참일 때만 업데이트
    //--------------------------------------------------------------------------
    assign pid_en = angle_valid & init_done & bias_done;

    //--------------------------------------------------------------------------
    // 듀티 스케일링: 0~1000 → 0~100 (TB6612FNG는 7비트 0~100%)
    //--------------------------------------------------------------------------
    assign motor_duty = (pwm_duty >= 16'd1000) ? 7'd100 : pwm_duty[9:0] / 10;

    //--------------------------------------------------------------------------
    // 방향 변환: pid_dir 0→CW(전진), 1→CCW(후진)
    //--------------------------------------------------------------------------
    assign motor_dir_cmd = pid_dir ? 2'b01 : 2'b10;

    //--------------------------------------------------------------------------
    // 방향 판정
    //--------------------------------------------------------------------------
    assign dir_front = (ax_corr >  TILT_TH);
    assign dir_back  = (ax_corr < -TILT_TH);

    assign dir_right = (ay_corr >  TILT_TH);
    assign dir_left  = (ay_corr < -TILT_TH);

    assign dir_up    = (az_corr >  Z_TH);
    assign dir_down  = (az_corr < -Z_TH);

    //==========================================================================
    // AB 홀 엔코더 (4x 쿼드러처, 10ms 속도 샘플링)
    //==========================================================================
    encoder u_encoder
    (
        .clk        (clk),
        .rst_n      (rst_n),
        .encA_a     (encA_a),
        .encA_b     (encA_b),
        .encB_a     (encB_a),
        .encB_b     (encB_b),
        .vel_period (24'd1_000_000),   // 10ms @ 100MHz
        .pos_a      (enc_pos_a),
        .pos_b      (enc_pos_b),
        .vel_a      (enc_vel_a),
        .vel_b      (enc_vel_b)
    );

    //==========================================================================
    // tick 생성
    //==========================================================================
    clk_divider u_clk_divider
    (
        .clk   (clk),
        .rst_n (rst_n),
        .tick  (tick)
    );

    //==========================================================================
    // MPU6050 상위 제어
    //==========================================================================
    mpu6050_ctrl u_mpu6050_ctrl
    (
        .clk        (clk),
        .rst_n      (rst_n),
        .tick       (tick),

        .busy       (busy),
        .done       (done),
        .ack_ok     (ack_ok),
        .err_code   (err_code),
        .rx_buf     (rx_buf),
        .rx_count   (rx_count),

        .start_req  (start_req),
        .rw         (rw),
        .reg_addr   (reg_addr),
        .tx_data    (tx_data),
        .burst_len  (burst_len),

        .init_done  (init_done),
        .data_valid (data_valid),
        .last_err   (last_err),

        .ax         (ax_raw),
        .ay         (ay_raw),
        .az         (az_raw),
        .gx         (gx_raw),
        .gy         (gy_raw),
        .gz         (gz_raw)
    );

    //==========================================================================
    // I2C master
    //==========================================================================
    i2c_master u_i2c_master
    (
        .clk        (clk),
        .rst_n      (rst_n),
        .tick       (tick),

        .start_req  (start_req),
        .rw         (rw),
        .reg_addr   (reg_addr),
        .tx_data    (tx_data),
        .burst_len  (burst_len),
        .sda_in     (i2c_sda),

        .busy       (busy),
        .done       (done),
        .ack_ok     (ack_ok),
        .err_code   (err_code),
        .rx_data    (rx_data),
        .rx_buf     (rx_buf),
        .rx_count   (rx_count),

        .sda_enable (sda_enable),
        .scl_enable (scl_enable)
    );

    //==========================================================================
    // bias 계산 + corrected 값 생성
    //==========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bias_done <= 1'b0;
            cal_cnt   <= 8'd0;

            ax_sum    <= 25'sd0;
            ay_sum    <= 25'sd0;
            az_sum    <= 25'sd0;
            gx_sum    <= 25'sd0;
            gy_sum    <= 25'sd0;
            gz_sum    <= 25'sd0;

            ax_bias   <= 16'sd0;
            ay_bias   <= 16'sd0;
            az_bias   <= 16'sd0;
            gx_bias   <= 16'sd0;
            gy_bias   <= 16'sd0;
            gz_bias   <= 16'sd0;

            ax_corr   <= 16'sd0;
            ay_corr   <= 16'sd0;
            az_corr   <= 16'sd0;
            gx_corr   <= 16'sd0;
            gy_corr   <= 16'sd0;
            gz_corr   <= 16'sd0;
        end
        else begin
            //------------------------------------------------------------------
            // bias 계산 전
            //------------------------------------------------------------------
            if (!bias_done) begin
                if (data_valid) begin
                    ax_sum <= ax_sum + ax_raw;
                    ay_sum <= ay_sum + ay_raw;
                    az_sum <= az_sum + az_raw;
                    gx_sum <= gx_sum + gx_raw;
                    gy_sum <= gy_sum + gy_raw;
                    gz_sum <= gz_sum + gz_raw;

                    if (cal_cnt == 8'd31) begin
                        // 평균 = 합 / 256 = >>> 8
                        ax_bias   <= ax_sum >>> 8;
                        ay_bias   <= ay_sum >>> 8;
                        az_bias   <= az_sum >>> 8;
                        gx_bias   <= gx_sum >>> 8;
                        gy_bias   <= gy_sum >>> 8;
                        gz_bias   <= gz_sum >>> 8;
                        bias_done <= 1'b1;
                    end
                    else begin
                        cal_cnt <= cal_cnt + 8'd1;
                    end
                end

                // bias 계산 전에는 raw 그대로 출력
                ax_corr <= ax_raw;
                ay_corr <= ay_raw;
                az_corr <= az_raw;
                gx_corr <= gx_raw;
                gy_corr <= gy_raw;
                gz_corr <= gz_raw;
            end
            //------------------------------------------------------------------
            // bias 계산 후
            //------------------------------------------------------------------
            else begin
                // AX, AY는 offset 제거
                ax_corr <= ax_raw - ax_bias;
                ay_corr <= ay_raw - ay_bias;

                // AZ는 초기 단계에서는 raw 유지
                az_corr <= az_raw;

                // 자이로 bias 제거
                gx_corr <= gx_raw - gx_bias;
                gy_corr <= gy_raw - gy_bias;
                gz_corr <= gz_raw - gz_bias;
            end
        end
    end

    //==========================================================================
    // 각도 계산 (상보 필터)
    //==========================================================================
    angle_calc u_angle_calc
    (
        .clk         (clk),
        .rst_n       (rst_n),
        .data_valid  (data_valid),
    // y축으로 수정함 !!
        .accel_x     (ay_corr),
        .accel_z     (az_corr),
        .gyro_x      (gx_corr),

        .angle       (angle),
        .angle_valid (angle_valid)
    );

    //==========================================================================
    // UART RX (PC → FPGA, 9600bps, active-high reset)
    //==========================================================================
    uart_rx u_uart_rx (
        .clk     (clk),
        .reset   (reset),
        .rx_pin  (uart_rx),
        .rx_data (rx_byte),
        .rx_done (rx_done)
    );

    //==========================================================================
    // PID K값 레지스터 초기화 + UART RX 파서
    // 프로토콜: 'P'<숫자>\n → KP, 'I'<숫자>\n → KI, 'D'<숫자>\n → KD
    // 예) "P384\n" → KP=384 (Kp=1.5)
    //==========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            kp_reg       <= 16'd200;
            ki_reg       <= 16'd0;
            kd_reg       <= 16'd0;
            setpoint_reg <= 16'sd0;
            angle_offset <= 16'sd0;
            kv_reg       <= 16'd0;
            enc_ref_a    <= 32'sd0;
            enc_ref_b    <= 32'sd0;
            rx_state     <= RX_IDLE;
            rx_cmd       <= 3'd0;
            rx_acc       <= 16'd0;
            rx_cnt       <= 4'd0;
            rx_sign      <= 1'b0;
        end
        else if (rx_done) begin
            case (rx_state)
                RX_IDLE: begin
                    rx_acc  <= 16'd0;
                    rx_cnt  <= 4'd0;
                    rx_sign <= 1'b0;
                    if      (rx_byte == "P") begin rx_cmd <= 3'd0; rx_state <= RX_DIGIT; end
                    else if (rx_byte == "I") begin rx_cmd <= 3'd1; rx_state <= RX_DIGIT; end
                    else if (rx_byte == "D") begin rx_cmd <= 3'd2; rx_state <= RX_DIGIT; end
                    else if (rx_byte == "S") begin rx_cmd <= 3'd3; rx_state <= RX_DIGIT; end
                    else if (rx_byte == "V") begin rx_cmd <= 3'd5; rx_state <= RX_DIGIT; end
                end

                RX_DIGIT: begin
                    if (rx_byte == "-" && rx_cnt == 4'd0) begin
                        // 첫 문자가 '-': 음수 플래그만 세우고 숫자 대기
                        rx_sign <= 1'b1;
                    end
                    else if (rx_byte >= "0" && rx_byte <= "9" && rx_cnt < 4'd5) begin
                        rx_acc <= rx_acc * 10 + (rx_byte - "0");
                        rx_cnt <= rx_cnt + 4'd1;
                    end
                    else if (rx_byte == 8'h0D || rx_byte == 8'h0A) begin
                        // 줄 끝 (\r 또는 \n): 값 커밋
                        if (rx_cmd == 3'd3) begin
                            // S 커맨드: 각도 영점 + 인코더 기준 위치 캡처
                            angle_offset <= angle;
                            setpoint_reg <= 16'sd0;
                            enc_ref_a    <= enc_pos_a;
                            enc_ref_b    <= enc_pos_b;
                        end
                        else if (rx_cnt > 4'd0) begin
                            if      (rx_cmd == 3'd0) kp_reg <= rx_acc;
                            else if (rx_cmd == 3'd1) ki_reg <= rx_acc;
                            else if (rx_cmd == 3'd2) kd_reg <= rx_acc;
                            else if (rx_cmd == 3'd5) kv_reg <= rx_acc;
                        end
                        rx_state <= RX_IDLE;
                    end
                    else begin
                        // 숫자/줄끝 이외 문자: 파싱 중단
                        rx_state <= RX_IDLE;
                    end
                end

                default: rx_state <= RX_IDLE;
            endcase
        end
    end

    //==========================================================================
    // PID 제어
    //==========================================================================
    pid u_pid
    (
        .clk       (clk),
        .rst_n     (rst_n),
        .en        (pid_en),

        .angle_in  (-angle_adj),    // 보정된 각도 사용 (S 후 0 기준)
        .setpoint  (pid_setpoint),  // 수동 setpoint + 인코더 외부 루프 보정

        .kp        (kp_reg),
        .ki        (ki_reg),
        .kd        (kd_reg),

        .pwm_duty  (pwm_duty),
        .dir       (pid_dir)
    );

    //==========================================================================
    // TB6612FNG 모터 드라이버
    //==========================================================================
    TB6612FNG u_tb6612fng
    (
        .clk      (clk),
        .reset    (reset),          // active-high

        .dirA_cmd (motor_dir_cmd),
        .dutyA    (motor_duty),

        .dirB_cmd (~motor_dir_cmd),  // 모터 B 반대 방향 장착 → 방향 반전
        .dutyB    (motor_duty),

        .PWMA     (PWMA_out),
        .AIN1     (AIN1_out),
        .AIN2     (AIN2_out),

        .PWMB     (PWMB_out),
        .BIN1     (BIN1_out),
        .BIN2     (BIN2_out)
    );

    //==========================================================================
    // UART debug
    //==========================================================================
    mpu6050_debug_uart u_debug_uart
    (
        .clk        (clk),
        .rst_n      (rst_n),
        .init_done  (init_done),
        .data_valid (data_valid),

        .accel_x    (ax_corr),
        .accel_y    (ay_corr),
        .accel_z    (az_corr),
        .gyro_x     (gx_corr),
        .gyro_y     (gy_corr),
        .gyro_z     (gz_corr),

        .angle      (angle_adj),    // 보정된 각도 표시 (S 후 0)

        .kp         (kp_reg),
        .ki         (ki_reg),
        .kd         (kd_reg),
        .setpoint   (angle_offset), // SP= 에 기준각 표시

        .vel_a      (enc_vel_a),
        .vel_b      (enc_vel_b),
        .pos_a      (enc_pos_a),
        .pos_b      (enc_pos_b),

        .uart_tx_o  (uart_tx_wire)
    );

    //==========================================================================
    // open-drain I2C
    //==========================================================================
    assign i2c_sda = (sda_enable) ? 1'b0 : 1'bz;
    assign i2c_scl = (scl_enable) ? 1'b0 : 1'bz;

    //==========================================================================
    // UART 출력
    //==========================================================================
    assign uart_tx  = uart_tx_wire;
    assign STBY_out = sw_stby;



endmodule

