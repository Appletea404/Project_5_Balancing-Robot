`timescale 1ns / 1ps

module top
(
    input  wire        clk,
    input  wire        reset,   // active-high

    inout  wire        i2c_sda,
    inout  wire        i2c_scl,

    input  wire        uart_rx,
    output wire        uart_tx,

    output wire        PWMA_out,
    output wire        AIN1_out,
    output wire        AIN2_out,
    output wire        PWMB_out,
    output wire        BIN1_out,
    output wire        BIN2_out,
    output wire        STBY_out,

    input  wire        sw_stby,

    input  wire        encA_a,
    input  wire        encA_b,
    input  wire        encB_a,
    input  wire        encB_b
);

    //==========================================================================
    // reset
    //==========================================================================
    wire rst_n = ~reset;

    //==========================================================================
    // tick
    //==========================================================================
    wire         tick;

    //==========================================================================
    // mpu6050_ctrl <-> i2c_master
    //==========================================================================
    wire         start_req;
    wire         rw;
    wire [7:0]   reg_addr;
    wire [7:0]   tx_data;
    wire [7:0]   burst_len;

    //==========================================================================
    // i2c_master status
    //==========================================================================
    wire         busy;
    wire         done;
    wire         ack_ok;
    wire [2:0]   err_code;
    wire [7:0]   rx_data;
    wire [127:0] rx_buf;
    wire [7:0]   rx_count;

    //==========================================================================
    // open-drain control
    //==========================================================================
    wire         sda_enable;
    wire         scl_enable;

    //==========================================================================
    // MPU raw data
    //==========================================================================
    wire signed [15:0] ax_raw;
    wire signed [15:0] ay_raw;
    wire signed [15:0] az_raw;
    wire signed [15:0] gx_raw;
    wire signed [15:0] gy_raw;
    wire signed [15:0] gz_raw;

    //==========================================================================
    // corrected data / bias
    //==========================================================================
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

    //==========================================================================
    // bias accumulation
    //==========================================================================
    reg [7:0]          cal_cnt;
    reg signed [24:0]  ax_sum;
    reg signed [24:0]  ay_sum;
    reg signed [24:0]  az_sum;
    reg signed [24:0]  gx_sum;
    reg signed [24:0]  gy_sum;
    reg signed [24:0]  gz_sum;

    //==========================================================================
    // upper state
    //==========================================================================
    wire               init_done;
    wire               data_valid;
    wire [2:0]         last_err;

    //==========================================================================
    // UART line
    //==========================================================================
    wire               uart_tx_wire;

    //==========================================================================
    // angle
    //==========================================================================
    wire signed [15:0] angle;
    wire               angle_valid;

    // S command captured balance offset
    reg signed [15:0]  angle_offset;

    // corrected angle (0 after S)
    wire signed [15:0] angle_adj = angle - angle_offset;

    //==========================================================================
    // PID / motor
    //==========================================================================
    wire [15:0]  pwm_duty;
    wire         pid_dir;
    wire         pid_en;

    wire [6:0]   motor_duty;
    wire [1:0]   motor_dir_cmd;
    reg          angle_valid_d;
    reg          angle_valid_dd;
    reg          angle_valid_ddd;

    //==========================================================================
    // encoder
    //==========================================================================
    wire signed [31:0] enc_pos_a, enc_pos_b;
    wire signed [15:0] enc_vel_a, enc_vel_b;

    //--------------------------------------------------------------------------
    // PID에 직접 넣을 velocity feedback
    //
    // 주의:
    // 이 값은 "로봇의 전진/후진 축 속도"가 되어야 한다.
    // 현재 기본값은 enc_vel_a - enc_vel_b 로 두었지만,
    // 실제 로그에서 앞으로 굴릴 때 VA/VB 부호 관계를 반드시 확인해야 한다.
    //
    // 1) 앞으로 갈 때 VA/VB가 서로 반대 부호면
    //    아래 diff 방식이 맞을 가능성이 크다.
    //
    // 2) 앞으로 갈 때 VA/VB가 같은 부호면
    //    아래 assign을 주석 처리하고 sum 방식으로 바꿔야 한다.
    //--------------------------------------------------------------------------
    wire signed [15:0] vel_fb;
    assign vel_fb = enc_vel_a - enc_vel_b;
    // assign vel_fb = (enc_vel_a + enc_vel_b) >>> 1;

    //==========================================================================
    // outer loop refs / runtime gains
    //==========================================================================
    reg signed [31:0] enc_ref_a, enc_ref_b;
    reg        [15:0] kv_reg;

    reg [15:0]        kp_reg;
    reg [15:0]        ki_reg;
    reg [15:0]        kd_reg;
    reg signed [15:0] setpoint_reg;

    //==========================================================================
    // UART RX parser
    //==========================================================================
    wire         rx_done;
    wire [7:0]   rx_byte;

    localparam [1:0] RX_IDLE  = 2'd0,
                     RX_DIGIT = 2'd1;

    reg [1:0]    rx_state;
    reg [2:0]    rx_cmd;
    reg [15:0]   rx_acc;
    reg [3:0]    rx_cnt;
    reg          rx_sign;
    reg          s_pending;

    //==========================================================================
    // outer loop : position + velocity
    //==========================================================================
    wire signed [31:0] enc_delta_a = enc_pos_a - enc_ref_a;
    wire signed [31:0] enc_delta_b = enc_pos_b - enc_ref_b;

    // 전후 이동 위치 오차
    wire signed [31:0] pos_err_w = enc_delta_a - enc_delta_b;

    // 전후 이동 속도 오차
    wire signed [15:0] vel_err_w = enc_vel_a - enc_vel_b;

    reg signed [15:0] pid_setpoint;
    wire signed [15:0] pos_err_sat_w;
    reg  signed [15:0] pos_err_sat_reg;
    reg  signed [15:0] vel_err_reg;
    reg  signed [31:0] pos_term_reg;
    reg  signed [31:0] vel_term_reg;
    wire signed [31:0] sp_corr_w;
    wire               outer_loop_en = (kv_reg != 16'd0);

    //--------------------------------------------------------------------------
    // 속도항 gain
    // 너무 세게 주면 오히려 발산이 커져서 보수적으로 유지
    //--------------------------------------------------------------------------
    localparam signed [15:0] KV_VEL = 16'sd8;

    //--------------------------------------------------------------------------
    // 외부 루프 최대 보정 각도 제한 (0.01도 단위)
    // 기존 ±220보다 약간 확장
    //--------------------------------------------------------------------------
    localparam signed [15:0] SP_CORR_MAX = 16'sd260;

    assign pos_err_sat_w = (pos_err_w >  32'sd32767) ? 16'sd32767 :
                           (pos_err_w < -32'sd32768) ? -16'sd32768 :
                           pos_err_w[15:0];

    assign sp_corr_w = (pos_term_reg >>> 8) + (vel_term_reg >>> 4);

    //--------------------------------------------------------------------------
    // outer loop pipeline
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos_err_sat_reg <= 16'sd0;
            vel_err_reg  <= 16'sd0;
            pos_term_reg <= 32'sd0;
            vel_term_reg <= 32'sd0;
            pid_setpoint <= 16'sd0;
        end
        else begin
            if (angle_valid) begin
                // Stage 0: capture encoder error first so the long carry chain
                // from the quadrature counters does not have to feed the DSP directly.
                pos_err_sat_reg <= pos_err_sat_w;
                vel_err_reg     <= vel_err_w;
            end

            if (angle_valid_d) begin
                // Register the expensive multiply terms first so the encoder
                // feedback does not have to reach pid_setpoint in one 100MHz cycle.
                if (outer_loop_en) begin
                    pos_term_reg <= $signed(pos_err_sat_reg) * $signed({1'b0, kv_reg});
                    vel_term_reg <= $signed(vel_err_reg) * $signed(KV_VEL);
                end
                else begin
                    pos_term_reg <= 32'sd0;
                    vel_term_reg <= 32'sd0;
                end
            end

            if (angle_valid_dd) begin
                // Apply the setpoint correction one cycle later. The control loop
                // runs at 10ms, so this extra FPGA clock is negligible.
                if (sp_corr_w >  $signed(SP_CORR_MAX))
                    pid_setpoint <= setpoint_reg - SP_CORR_MAX;
                else if (sp_corr_w < -$signed(SP_CORR_MAX))
                    pid_setpoint <= setpoint_reg + SP_CORR_MAX;
                else
                    pid_setpoint <= setpoint_reg - sp_corr_w[15:0];
            end
        end
    end

    //--------------------------------------------------------------------------
    // Delay the enable pulse so PID sees the updated angle/setpoint pipeline.
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            angle_valid_d <= 1'b0;
            angle_valid_dd <= 1'b0;
            angle_valid_ddd <= 1'b0;
        end
        else begin
            angle_valid_d <= angle_valid;
            angle_valid_dd <= angle_valid_d;
            angle_valid_ddd <= angle_valid_dd;
        end
    end

    //==========================================================================
    // direction thresholds
    //==========================================================================
    localparam signed [15:0] TILT_TH = 16'sd1000;
    localparam signed [15:0] Z_TH    = 16'sd12000;

    wire dir_front;
    wire dir_back;
    wire dir_left;
    wire dir_right;
    wire dir_up;
    wire dir_down;

    //==========================================================================
    // PID trigger
    //==========================================================================
    assign pid_en = angle_valid_ddd & init_done & bias_done;

    //==========================================================================
    // duty scaling
    //==========================================================================
    assign motor_duty = (pwm_duty >= 16'd1000) ? 7'd100 : pwm_duty[9:0] / 10;

    //==========================================================================
    // direction conversion
    //==========================================================================
    assign motor_dir_cmd = pid_dir ? 2'b01 : 2'b10;

    //==========================================================================
    // simple direction indicators
    //==========================================================================
    assign dir_front = (ax_corr >  TILT_TH);
    assign dir_back  = (ax_corr < -TILT_TH);
    assign dir_right = (ay_corr >  TILT_TH);
    assign dir_left  = (ay_corr < -TILT_TH);
    assign dir_up    = (az_corr >  Z_TH);
    assign dir_down  = (az_corr < -Z_TH);

    //==========================================================================
    // encoder
    //==========================================================================
    encoder u_encoder
    (
        .clk        (clk),
        .rst_n      (rst_n),
        .encA_a     (encA_a),
        .encA_b     (encA_b),
        .encB_a     (encB_a),
        .encB_b     (encB_b),
        .vel_period (24'd1_000_000),
        .pos_a      (enc_pos_a),
        .pos_b      (enc_pos_b),
        .vel_a      (enc_vel_a),
        .vel_b      (enc_vel_b)
    );

    //==========================================================================
    // tick generator
    //==========================================================================
    clk_divider u_clk_divider
    (
        .clk   (clk),
        .rst_n (rst_n),
        .tick  (tick)
    );

    //==========================================================================
    // MPU6050 top control
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
    // bias calculation + corrected values
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
            if (!bias_done) begin
                if (data_valid) begin
                    ax_sum <= ax_sum + ax_raw;
                    ay_sum <= ay_sum + ay_raw;
                    az_sum <= az_sum + az_raw;
                    gx_sum <= gx_sum + gx_raw;
                    gy_sum <= gy_sum + gy_raw;
                    gz_sum <= gz_sum + gz_raw;

                    if (cal_cnt == 8'd31) begin
                        ax_bias   <= (ax_sum + ax_raw) >>> 5;
                        ay_bias   <= (ay_sum + ay_raw) >>> 5;
                        az_bias   <= (az_sum + az_raw) >>> 5;
                        gx_bias   <= (gx_sum + gx_raw) >>> 5;
                        gy_bias   <= (gy_sum + gy_raw) >>> 5;
                        gz_bias   <= (gz_sum + gz_raw) >>> 5;
                        bias_done <= 1'b1;
                    end
                    else begin
                        cal_cnt <= cal_cnt + 8'd1;
                    end
                end

                ax_corr <= ax_raw;
                ay_corr <= ay_raw;
                az_corr <= az_raw;
                gx_corr <= gx_raw;
                gy_corr <= gy_raw;
                gz_corr <= gz_raw;
            end
            else begin
                ax_corr <= ax_raw - ax_bias;
                ay_corr <= ay_raw - ay_bias;
                az_corr <= az_raw - az_bias;

                gx_corr <= gx_raw - gx_bias;
                gy_corr <= gy_raw - gy_bias;
                gz_corr <= gz_raw - gz_bias;
            end
        end
    end

    //==========================================================================
    // angle calculation
    //==========================================================================
    angle_calc u_angle_calc
    (
        .clk         (clk),
        .rst_n       (rst_n),
        .data_valid  (data_valid),
        .accel_x     (ay_corr),
        .accel_z     (az_corr),
        .gyro_x      (gx_corr),
        .angle       (angle),
        .angle_valid (angle_valid)
    );

    //==========================================================================
    // UART RX
    //==========================================================================
    uart_rx u_uart_rx
    (
        .clk     (clk),
        .reset   (reset),
        .rx_pin  (uart_rx),
        .rx_data (rx_byte),
        .rx_done (rx_done)
    );

    //==========================================================================
    // runtime gain registers + UART parser
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
            s_pending    <= 1'b0;
        end
        else if (rx_done) begin
            case (rx_state)
                RX_IDLE: begin
                    rx_acc  <= 16'd0;
                    rx_cnt  <= 4'd0;
                    rx_sign <= 1'b0;

                    if      (rx_byte == "P") begin
                        rx_cmd    <= 3'd0;
                        rx_state  <= RX_DIGIT;
                        s_pending <= 1'b0;
                    end
                    else if (rx_byte == "I") begin
                        rx_cmd    <= 3'd1;
                        rx_state  <= RX_DIGIT;
                        s_pending <= 1'b0;
                    end
                    else if (rx_byte == "D") begin
                        rx_cmd    <= 3'd2;
                        rx_state  <= RX_DIGIT;
                        s_pending <= 1'b0;
                    end
                    else if (rx_byte == "S") begin
                        rx_cmd    <= 3'd3;
                        rx_state  <= RX_DIGIT;
                        s_pending <= 1'b1;
                    end
                    else if (rx_byte == "V") begin
                        rx_cmd    <= 3'd5;
                        rx_state  <= RX_DIGIT;
                        s_pending <= 1'b0;
                    end
                end

                RX_DIGIT: begin
                    if (rx_byte == "-" && rx_cnt == 4'd0) begin
                        rx_sign <= 1'b1;
                    end
                    else if (rx_byte >= "0" && rx_byte <= "9" && rx_cnt < 4'd5) begin
                        rx_acc <= rx_acc * 10 + (rx_byte - "0");
                        rx_cnt <= rx_cnt + 4'd1;
                    end
                    else if (rx_byte == 8'h0D || rx_byte == 8'h0A) begin
                        if (s_pending) begin
                            angle_offset <= angle;
                            setpoint_reg <= 16'sd0;
                            enc_ref_a    <= enc_pos_a;
                            enc_ref_b    <= enc_pos_b;
                            s_pending    <= 1'b0;
                        end
                        else if (rx_cnt > 4'd0) begin
                            // Gains are unsigned. If a negative value is typed,
                            // clamp it to zero instead of silently wrapping.
                            if (rx_sign) begin
                                if      (rx_cmd == 3'd0) kp_reg <= 16'd0;
                                else if (rx_cmd == 3'd1) ki_reg <= 16'd0;
                                else if (rx_cmd == 3'd2) kd_reg <= 16'd0;
                                else if (rx_cmd == 3'd5) kv_reg <= 16'd0;
                            end
                            else begin
                                if      (rx_cmd == 3'd0) kp_reg <= rx_acc;
                                else if (rx_cmd == 3'd1) ki_reg <= rx_acc;
                                else if (rx_cmd == 3'd2) kd_reg <= rx_acc;
                                else if (rx_cmd == 3'd5) kv_reg <= rx_acc;
                            end

                            s_pending <= 1'b0;
                        end
                        else begin
                            s_pending <= 1'b0;
                        end

                        rx_state <= RX_IDLE;
                        rx_cmd   <= 3'd0;
                    end
                    else begin
                        rx_state  <= RX_IDLE;
                        rx_cmd    <= 3'd0;
                        s_pending <= 1'b0;
                    end
                end

                default: begin
                    rx_state  <= RX_IDLE;
                    rx_cmd    <= 3'd0;
                    s_pending <= 1'b0;
                end
            endcase
        end
    end

    //==========================================================================
    // PID controller
    //==========================================================================
    pid u_pid
    (
        .clk       (clk),
        .rst_n     (rst_n),
        .en        (pid_en),

        .angle_in  (-angle_adj),
        .setpoint  (pid_setpoint),
        .vel_in    (vel_fb),
        .gyro_in   (-gx_corr),

        .kp        (kp_reg),
        .ki        (ki_reg),
        .kd        (kd_reg),

        .pwm_duty  (pwm_duty),
        .dir       (pid_dir)
    );

    //==========================================================================
    // motor driver
    //==========================================================================
    TB6612FNG u_tb6612fng
    (
        .clk      (clk),
        .reset    (reset),

        .dirA_cmd (motor_dir_cmd),
        .dutyA    (motor_duty),

        .dirB_cmd (~motor_dir_cmd),
        .dutyB    (motor_duty),

        .PWMA     (PWMA_out),
        .AIN1     (AIN1_out),
        .AIN2     (AIN2_out),

        .PWMB     (PWMB_out),
        .BIN1     (BIN1_out),
        .BIN2     (BIN2_out)
    );

    //==========================================================================
    // debug UART
    //==========================================================================
    mpu6050_debug_uart u_debug_uart
    (
        .clk          (clk),
        .rst_n        (rst_n),
        .init_done    (init_done),
        .data_valid   (data_valid),

        .accel_x      (ax_corr),
        .accel_y      (ay_corr),
        .accel_z      (az_corr),
        .gyro_x       (gx_corr),
        .gyro_y       (gy_corr),
        .gyro_z       (gz_corr),

        .angle        (angle_adj),
        .kp           (kp_reg),
        .ki           (ki_reg),
        .kd           (kd_reg),
        .setpoint     (pid_setpoint),
        .angle_offset (angle_offset),

        .vel_a        (enc_vel_a),
        .vel_b        (enc_vel_b),
        .pos_a        (enc_pos_a),
        .pos_b        (enc_pos_b),

        .uart_tx_o    (uart_tx_wire)
    );

    //==========================================================================
    // open-drain I2C
    //==========================================================================
    assign i2c_sda = (sda_enable) ? 1'b0 : 1'bz;
    assign i2c_scl = (scl_enable) ? 1'b0 : 1'bz;

    //==========================================================================
    // outputs
    //==========================================================================
    assign uart_tx  = uart_tx_wire;
    assign STBY_out = sw_stby;

endmodule


// `timescale 1ns / 1ps

// module top
// (
//     input  wire        clk,
//     input  wire        reset,   // active-high

//     inout  wire        i2c_sda,
//     inout  wire        i2c_scl,

//     input  wire        uart_rx,
//     output wire        uart_tx,

//     output wire        PWMA_out,
//     output wire        AIN1_out,
//     output wire        AIN2_out,
//     output wire        PWMB_out,
//     output wire        BIN1_out,
//     output wire        BIN2_out,
//     output wire        STBY_out,

//     input  wire        sw_stby,

//     input  wire        encA_a,
//     input  wire        encA_b,
//     input  wire        encB_a,
//     input  wire        encB_b
// );

//     //==========================================================================
//     // reset
//     //==========================================================================
//     wire rst_n = ~reset;

//     //==========================================================================
//     // tick
//     //==========================================================================
//     wire         tick;

//     //==========================================================================
//     // mpu6050_ctrl <-> i2c_master
//     //==========================================================================
//     wire         start_req;
//     wire         rw;
//     wire [7:0]   reg_addr;
//     wire [7:0]   tx_data;
//     wire [7:0]   burst_len;

//     //==========================================================================
//     // i2c_master status
//     //==========================================================================
//     wire         busy;
//     wire         done;
//     wire         ack_ok;
//     wire [2:0]   err_code;
//     wire [7:0]   rx_data;
//     wire [127:0] rx_buf;
//     wire [7:0]   rx_count;

//     //==========================================================================
//     // open-drain control
//     //==========================================================================
//     wire         sda_enable;
//     wire         scl_enable;

//     //==========================================================================
//     // MPU raw data
//     //==========================================================================
//     wire signed [15:0] ax_raw;
//     wire signed [15:0] ay_raw;
//     wire signed [15:0] az_raw;
//     wire signed [15:0] gx_raw;
//     wire signed [15:0] gy_raw;
//     wire signed [15:0] gz_raw;

//     //==========================================================================
//     // corrected data / bias
//     //==========================================================================
//     reg                bias_done;

//     reg signed [15:0]  ax_bias;
//     reg signed [15:0]  ay_bias;
//     reg signed [15:0]  az_bias;
//     reg signed [15:0]  gx_bias;
//     reg signed [15:0]  gy_bias;
//     reg signed [15:0]  gz_bias;

//     reg signed [15:0]  ax_corr;
//     reg signed [15:0]  ay_corr;
//     reg signed [15:0]  az_corr;
//     reg signed [15:0]  gx_corr;
//     reg signed [15:0]  gy_corr;
//     reg signed [15:0]  gz_corr;

//     //==========================================================================
//     // bias accumulation
//     //==========================================================================
//     reg [7:0]          cal_cnt;
//     reg signed [24:0]  ax_sum;
//     reg signed [24:0]  ay_sum;
//     reg signed [24:0]  az_sum;
//     reg signed [24:0]  gx_sum;
//     reg signed [24:0]  gy_sum;
//     reg signed [24:0]  gz_sum;

//     //==========================================================================
//     // upper state
//     //==========================================================================
//     wire               init_done;
//     wire               data_valid;
//     wire [2:0]         last_err;

//     //==========================================================================
//     // UART line
//     //==========================================================================
//     wire               uart_tx_wire;

//     //==========================================================================
//     // angle
//     //==========================================================================
//     wire signed [15:0] angle;
//     wire               angle_valid;

//     // S command captured balance offset
//     reg signed [15:0]  angle_offset;

//     // corrected angle (0 after S)
//     wire signed [15:0] angle_adj = angle - angle_offset;

//     //==========================================================================
//     // PID / motor
//     //==========================================================================
//     wire [15:0]  pwm_duty;
//     wire         pid_dir;
//     wire         pid_en;

//     wire [6:0]   motor_duty;
//     wire [1:0]   motor_dir_cmd;

//     //==========================================================================
//     // encoder
//     //==========================================================================
//     wire signed [31:0] enc_pos_a, enc_pos_b;
//     wire signed [15:0] enc_vel_a, enc_vel_b;

//     //==========================================================================
//     // outer loop refs / runtime gains
//     //==========================================================================
//     reg signed [31:0] enc_ref_a, enc_ref_b;
//     reg        [15:0] kv_reg;

//     reg [15:0]        kp_reg;
//     reg [15:0]        ki_reg;
//     reg [15:0]        kd_reg;
//     reg signed [15:0] setpoint_reg;

//     //==========================================================================
//     // UART RX parser
//     //==========================================================================
//     wire         rx_done;
//     wire [7:0]   rx_byte;

//     localparam [1:0] RX_IDLE  = 2'd0,
//                      RX_DIGIT = 2'd1;

//     reg [1:0]    rx_state;
//     reg [2:0]    rx_cmd;
//     reg [15:0]   rx_acc;
//     reg [3:0]    rx_cnt;
//     reg          rx_sign;
//     reg          s_pending;

//     //==========================================================================
//     // outer loop : position + velocity
//     //==========================================================================
//     wire signed [31:0] enc_delta_a = enc_pos_a - enc_ref_a;
//     wire signed [31:0] enc_delta_b = enc_pos_b - enc_ref_b;

//     // 전후 이동 위치 오차
//     wire signed [31:0] pos_err_w = enc_delta_a - enc_delta_b;

//     // 전후 이동 속도 오차
//     wire signed [15:0] vel_err_w = enc_vel_a - enc_vel_b;

//     reg signed [31:0] pos_err_reg;
//     reg signed [15:0] vel_err_reg;

//     reg signed [31:0] pos_term_reg;
//     reg signed [31:0] vel_term_reg;

//     reg signed [15:0] pid_setpoint;

//     //--------------------------------------------------------------------------
//     // 속도항 gain
//     // 너무 세게 주면 오히려 발산이 커져서 보수적으로 유지
//     //--------------------------------------------------------------------------
//     localparam signed [15:0] KV_VEL = 16'sd8;

//     //--------------------------------------------------------------------------
//     // 외부 루프 최대 보정 각도 제한 (0.01도 단위)
//     // 기존 ±220보다 약간 확장
//     //--------------------------------------------------------------------------
//     localparam signed [15:0] SP_CORR_MAX = 16'sd260;

//     //--------------------------------------------------------------------------
//     // outer loop pipeline
//     //--------------------------------------------------------------------------
//     always @(posedge clk or negedge rst_n) begin
//         if (!rst_n) begin
//             pos_err_reg  <= 32'sd0;
//             vel_err_reg  <= 16'sd0;
//             pos_term_reg <= 32'sd0;
//             vel_term_reg <= 32'sd0;
//             pid_setpoint <= 16'sd0;
//         end
//         else if (angle_valid) begin
//             //------------------------------------------------------------------
//             // Stage 0: sample position / velocity error
//             //------------------------------------------------------------------
//             pos_err_reg <= pos_err_w;
//             vel_err_reg <= vel_err_w;

//             //------------------------------------------------------------------
//             // Stage 1: position term
//             // kv_reg is position gain from UART "V"
//             //------------------------------------------------------------------
//             pos_term_reg <= $signed(
//                 (pos_err_reg >  32'sd32767) ? 16'sd32767 :
//                 (pos_err_reg < -32'sd32768) ? -16'sd32768 :
//                 pos_err_reg[15:0]
//             ) * $signed({1'b0, kv_reg});

//             //------------------------------------------------------------------
//             // Stage 2: velocity term
//             //------------------------------------------------------------------
//             vel_term_reg <= $signed(vel_err_reg) * $signed(KV_VEL);

//             //------------------------------------------------------------------
//             // Stage 3: final setpoint correction
//             //
//             // position term keeps previous scale
//             // velocity term is kept moderate to avoid overreaction
//             //------------------------------------------------------------------
//             begin : OUTER_LOOP_SETPOINT
//                 reg signed [31:0] sp_corr;

//                 sp_corr = (pos_term_reg >>> 8) + (vel_term_reg >>> 4);

//                 if (sp_corr >  $signed(SP_CORR_MAX))
//                     pid_setpoint <= setpoint_reg - SP_CORR_MAX;
//                 else if (sp_corr < -$signed(SP_CORR_MAX))
//                     pid_setpoint <= setpoint_reg + SP_CORR_MAX;
//                 else
//                     pid_setpoint <= setpoint_reg - sp_corr[15:0];
//             end
//         end
//     end

//     //==========================================================================
//     // direction thresholds
//     //==========================================================================
//     localparam signed [15:0] TILT_TH = 16'sd1000;
//     localparam signed [15:0] Z_TH    = 16'sd12000;

//     wire dir_front;
//     wire dir_back;
//     wire dir_left;
//     wire dir_right;
//     wire dir_up;
//     wire dir_down;

//     //==========================================================================
//     // PID trigger
//     //==========================================================================
//     assign pid_en = angle_valid & init_done & bias_done;

//     //==========================================================================
//     // duty scaling
//     //==========================================================================
//     assign motor_duty = (pwm_duty >= 16'd1000) ? 7'd100 : pwm_duty[9:0] / 10;

//     //==========================================================================
//     // direction conversion
//     //==========================================================================
//     assign motor_dir_cmd = pid_dir ? 2'b01 : 2'b10;

//     //==========================================================================
//     // simple direction indicators
//     //==========================================================================
//     assign dir_front = (ax_corr >  TILT_TH);
//     assign dir_back  = (ax_corr < -TILT_TH);
//     assign dir_right = (ay_corr >  TILT_TH);
//     assign dir_left  = (ay_corr < -TILT_TH);
//     assign dir_up    = (az_corr >  Z_TH);
//     assign dir_down  = (az_corr < -Z_TH);

//     //==========================================================================
//     // encoder
//     //==========================================================================
//     encoder u_encoder
//     (
//         .clk        (clk),
//         .rst_n      (rst_n),
//         .encA_a     (encA_a),
//         .encA_b     (encA_b),
//         .encB_a     (encB_a),
//         .encB_b     (encB_b),
//         .vel_period (24'd1_000_000),
//         .pos_a      (enc_pos_a),
//         .pos_b      (enc_pos_b),
//         .vel_a      (enc_vel_a),
//         .vel_b      (enc_vel_b)
//     );

//     //==========================================================================
//     // tick generator
//     //==========================================================================
//     clk_divider u_clk_divider
//     (
//         .clk   (clk),
//         .rst_n (rst_n),
//         .tick  (tick)
//     );

//     //==========================================================================
//     // MPU6050 top control
//     //==========================================================================
//     mpu6050_ctrl u_mpu6050_ctrl
//     (
//         .clk        (clk),
//         .rst_n      (rst_n),
//         .tick       (tick),

//         .busy       (busy),
//         .done       (done),
//         .ack_ok     (ack_ok),
//         .err_code   (err_code),
//         .rx_buf     (rx_buf),
//         .rx_count   (rx_count),

//         .start_req  (start_req),
//         .rw         (rw),
//         .reg_addr   (reg_addr),
//         .tx_data    (tx_data),
//         .burst_len  (burst_len),

//         .init_done  (init_done),
//         .data_valid (data_valid),
//         .last_err   (last_err),

//         .ax         (ax_raw),
//         .ay         (ay_raw),
//         .az         (az_raw),
//         .gx         (gx_raw),
//         .gy         (gy_raw),
//         .gz         (gz_raw)
//     );

//     //==========================================================================
//     // I2C master
//     //==========================================================================
//     i2c_master u_i2c_master
//     (
//         .clk        (clk),
//         .rst_n      (rst_n),
//         .tick       (tick),

//         .start_req  (start_req),
//         .rw         (rw),
//         .reg_addr   (reg_addr),
//         .tx_data    (tx_data),
//         .burst_len  (burst_len),
//         .sda_in     (i2c_sda),

//         .busy       (busy),
//         .done       (done),
//         .ack_ok     (ack_ok),
//         .err_code   (err_code),
//         .rx_data    (rx_data),
//         .rx_buf     (rx_buf),
//         .rx_count   (rx_count),

//         .sda_enable (sda_enable),
//         .scl_enable (scl_enable)
//     );

//     //==========================================================================
//     // bias calculation + corrected values
//     //==========================================================================
//     always @(posedge clk or negedge rst_n) begin
//         if (!rst_n) begin
//             bias_done <= 1'b0;
//             cal_cnt   <= 8'd0;

//             ax_sum    <= 25'sd0;
//             ay_sum    <= 25'sd0;
//             az_sum    <= 25'sd0;
//             gx_sum    <= 25'sd0;
//             gy_sum    <= 25'sd0;
//             gz_sum    <= 25'sd0;

//             ax_bias   <= 16'sd0;
//             ay_bias   <= 16'sd0;
//             az_bias   <= 16'sd0;
//             gx_bias   <= 16'sd0;
//             gy_bias   <= 16'sd0;
//             gz_bias   <= 16'sd0;

//             ax_corr   <= 16'sd0;
//             ay_corr   <= 16'sd0;
//             az_corr   <= 16'sd0;
//             gx_corr   <= 16'sd0;
//             gy_corr   <= 16'sd0;
//             gz_corr   <= 16'sd0;
//         end
//         else begin
//             if (!bias_done) begin
//                 if (data_valid) begin
//                     ax_sum <= ax_sum + ax_raw;
//                     ay_sum <= ay_sum + ay_raw;
//                     az_sum <= az_sum + az_raw;
//                     gx_sum <= gx_sum + gx_raw;
//                     gy_sum <= gy_sum + gy_raw;
//                     gz_sum <= gz_sum + gz_raw;

//                     if (cal_cnt == 8'd31) begin
//                         ax_bias   <= (ax_sum + ax_raw) >>> 5;
//                         ay_bias   <= (ay_sum + ay_raw) >>> 5;
//                         az_bias   <= (az_sum + az_raw) >>> 5;
//                         gx_bias   <= (gx_sum + gx_raw) >>> 5;
//                         gy_bias   <= (gy_sum + gy_raw) >>> 5;
//                         gz_bias   <= (gz_sum + gz_raw) >>> 5;
//                         bias_done <= 1'b1;
//                     end
//                     else begin
//                         cal_cnt <= cal_cnt + 8'd1;
//                     end
//                 end

//                 ax_corr <= ax_raw;
//                 ay_corr <= ay_raw;
//                 az_corr <= az_raw;
//                 gx_corr <= gx_raw;
//                 gy_corr <= gy_raw;
//                 gz_corr <= gz_raw;
//             end
//             else begin
//                 ax_corr <= ax_raw - ax_bias;
//                 ay_corr <= ay_raw - ay_bias;
//                 az_corr <= az_raw - az_bias;

//                 gx_corr <= gx_raw - gx_bias;
//                 gy_corr <= gy_raw - gy_bias;
//                 gz_corr <= gz_raw - gz_bias;
//             end
//         end
//     end

//     //==========================================================================
//     // angle calculation
//     //==========================================================================
//     angle_calc u_angle_calc
//     (
//         .clk         (clk),
//         .rst_n       (rst_n),
//         .data_valid  (data_valid),
//         .accel_x     (ay_corr),
//         .accel_z     (az_corr),
//         .gyro_x      (gx_corr),
//         .angle       (angle),
//         .angle_valid (angle_valid)
//     );

//     //==========================================================================
//     // UART RX
//     //==========================================================================
//     uart_rx u_uart_rx
//     (
//         .clk     (clk),
//         .reset   (reset),
//         .rx_pin  (uart_rx),
//         .rx_data (rx_byte),
//         .rx_done (rx_done)
//     );

//     //==========================================================================
//     // runtime gain registers + UART parser
//     //==========================================================================
//     always @(posedge clk or negedge rst_n) begin
//         if (!rst_n) begin
//             kp_reg       <= 16'd200;
//             ki_reg       <= 16'd0;
//             kd_reg       <= 16'd0;
//             setpoint_reg <= 16'sd0;
//             angle_offset <= 16'sd0;
//             kv_reg       <= 16'd0;

//             enc_ref_a    <= 32'sd0;
//             enc_ref_b    <= 32'sd0;

//             rx_state     <= RX_IDLE;
//             rx_cmd       <= 3'd0;
//             rx_acc       <= 16'd0;
//             rx_cnt       <= 4'd0;
//             rx_sign      <= 1'b0;
//             s_pending    <= 1'b0;
//         end
//         else if (rx_done) begin
//             case (rx_state)
//                 RX_IDLE: begin
//                     rx_acc  <= 16'd0;
//                     rx_cnt  <= 4'd0;
//                     rx_sign <= 1'b0;

//                     if      (rx_byte == "P") begin
//                         rx_cmd    <= 3'd0;
//                         rx_state  <= RX_DIGIT;
//                         s_pending <= 1'b0;
//                     end
//                     else if (rx_byte == "I") begin
//                         rx_cmd    <= 3'd1;
//                         rx_state  <= RX_DIGIT;
//                         s_pending <= 1'b0;
//                     end
//                     else if (rx_byte == "D") begin
//                         rx_cmd    <= 3'd2;
//                         rx_state  <= RX_DIGIT;
//                         s_pending <= 1'b0;
//                     end
//                     else if (rx_byte == "S") begin
//                         rx_cmd    <= 3'd3;
//                         rx_state  <= RX_DIGIT;
//                         s_pending <= 1'b1;
//                     end
//                     else if (rx_byte == "V") begin
//                         rx_cmd    <= 3'd5;
//                         rx_state  <= RX_DIGIT;
//                         s_pending <= 1'b0;
//                     end
//                 end

//                 RX_DIGIT: begin
//                     if (rx_byte == "-" && rx_cnt == 4'd0) begin
//                         rx_sign <= 1'b1;
//                     end
//                     else if (rx_byte >= "0" && rx_byte <= "9" && rx_cnt < 4'd5) begin
//                         rx_acc <= rx_acc * 10 + (rx_byte - "0");
//                         rx_cnt <= rx_cnt + 4'd1;
//                     end
//                     else if (rx_byte == 8'h0D || rx_byte == 8'h0A) begin
//                         if (s_pending) begin
//                             angle_offset <= angle;
//                             setpoint_reg <= 16'sd0;
//                             enc_ref_a    <= enc_pos_a;
//                             enc_ref_b    <= enc_pos_b;
//                             s_pending    <= 1'b0;
//                         end
//                         else if (rx_cnt > 4'd0) begin
//                             if      (rx_cmd == 3'd0) kp_reg <= rx_acc;
//                             else if (rx_cmd == 3'd1) ki_reg <= rx_acc;
//                             else if (rx_cmd == 3'd2) kd_reg <= rx_acc;
//                             else if (rx_cmd == 3'd5) kv_reg <= rx_acc;

//                             s_pending <= 1'b0;
//                         end
//                         else begin
//                             s_pending <= 1'b0;
//                         end

//                         rx_state <= RX_IDLE;
//                         rx_cmd   <= 3'd0;
//                     end
//                     else begin
//                         rx_state  <= RX_IDLE;
//                         rx_cmd    <= 3'd0;
//                         s_pending <= 1'b0;
//                     end
//                 end

//                 default: begin
//                     rx_state  <= RX_IDLE;
//                     rx_cmd    <= 3'd0;
//                     s_pending <= 1'b0;
//                 end
//             endcase
//         end
//     end

//     //==========================================================================
//     // PID controller
//     //==========================================================================
//     pid u_pid
//     (
//         .clk       (clk),
//         .rst_n     (rst_n),
//         .en        (pid_en),

//         .angle_in  (-angle_adj),
//         .setpoint  (pid_setpoint),

//         .kp        (kp_reg),
//         .ki        (ki_reg),
//         .kd        (kd_reg),

//         .pwm_duty  (pwm_duty),
//         .dir       (pid_dir)
//     );

//     //==========================================================================
//     // motor driver
//     //==========================================================================
//     TB6612FNG u_tb6612fng
//     (
//         .clk      (clk),
//         .reset    (reset),

//         .dirA_cmd (motor_dir_cmd),
//         .dutyA    (motor_duty),

//         .dirB_cmd (~motor_dir_cmd),
//         .dutyB    (motor_duty),

//         .PWMA     (PWMA_out),
//         .AIN1     (AIN1_out),
//         .AIN2     (AIN2_out),

//         .PWMB     (PWMB_out),
//         .BIN1     (BIN1_out),
//         .BIN2     (BIN2_out)
//     );

//     //==========================================================================
//     // debug UART
//     //==========================================================================
//     mpu6050_debug_uart u_debug_uart
//     (
//         .clk          (clk),
//         .rst_n        (rst_n),
//         .init_done    (init_done),
//         .data_valid   (data_valid),

//         .accel_x      (ax_corr),
//         .accel_y      (ay_corr),
//         .accel_z      (az_corr),
//         .gyro_x       (gx_corr),
//         .gyro_y       (gy_corr),
//         .gyro_z       (gz_corr),

//         .angle        (angle_adj),
//         .kp           (kp_reg),
//         .ki           (ki_reg),
//         .kd           (kd_reg),
//         .setpoint     (pid_setpoint),
//         .angle_offset (angle_offset),

//         .vel_a        (enc_vel_a),
//         .vel_b        (enc_vel_b),
//         .pos_a        (enc_pos_a),
//         .pos_b        (enc_pos_b),

//         .uart_tx_o    (uart_tx_wire)
//     );

//     //==========================================================================
//     // open-drain I2C
//     //==========================================================================
//     assign i2c_sda = (sda_enable) ? 1'b0 : 1'bz;
//     assign i2c_scl = (scl_enable) ? 1'b0 : 1'bz;

//     //==========================================================================
//     // outputs
//     //==========================================================================
//     assign uart_tx  = uart_tx_wire;
//     assign STBY_out = sw_stby;

// endmodule
