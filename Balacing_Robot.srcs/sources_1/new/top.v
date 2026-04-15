
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
    output wire        BIN2_out
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
    wire signed [15:0] angle;        // 0.01° 단위
    wire               angle_valid;

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
    // 방향 판단 threshold
    //--------------------------------------------------------------------------
    localparam signed [15:0] TILT_TH = 16'sd1000;   // 앞/뒤/좌/우 판정
    localparam signed [15:0] Z_TH    = 16'sd12000;  // 위/아래 판정 (약 0.73g)

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
    //
    // 실제 보드 방향에 따라 front/back, left/right 부호가 반대일 수 있음
    // 반대로 나오면 해당 두 줄만 서로 바꾸면 됨
    //--------------------------------------------------------------------------
    assign dir_front = (ax_corr >  TILT_TH);
    assign dir_back  = (ax_corr < -TILT_TH);

    assign dir_right = (ay_corr >  TILT_TH);
    assign dir_left  = (ay_corr < -TILT_TH);

    // 위/아래는 평평한 자세 상태 판정
    // up   : 윗면이 위로 향함 (정상 방향)
    // down : 뒤집힘
    assign dir_up    = (az_corr >  Z_TH);
    assign dir_down  = (az_corr < -Z_TH);

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
    // - data_valid가 올라올 때만 샘플 사용
    // - 처음 256개 평균으로 bias 계산
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

                    if (cal_cnt == 8'hFF) begin
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
        .gyro_x      (gy_corr),

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
                        if (rx_cnt > 4'd0) begin
                            if      (rx_cmd == 3'd0) kp_reg       <= rx_acc;
                            else if (rx_cmd == 3'd1) ki_reg       <= rx_acc;
                            else if (rx_cmd == 3'd2) kd_reg       <= rx_acc;
                            else                     setpoint_reg <= rx_sign
                                                         ? -$signed({1'b0, rx_acc[14:0]})
                                                         :  $signed({1'b0, rx_acc[14:0]});
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

        .angle_in  (angle),
        .setpoint  (setpoint_reg),  // UART 'S' 명령으로 런타임 조정

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

        .dirB_cmd (motor_dir_cmd),
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
    // - corrected 값 출력
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

        .angle      (angle),

        .kp         (kp_reg),
        .ki         (ki_reg),
        .kd         (kd_reg),

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
    assign uart_tx = uart_tx_wire;



endmodule

