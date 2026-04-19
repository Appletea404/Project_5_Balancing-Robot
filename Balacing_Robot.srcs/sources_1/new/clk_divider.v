`timescale 1ns / 1ps
// ================================================================================
// clk_divider
// - 100MHz 시스템 클럭에서 주기적인 tick pulse 생성
//
// 현재 설정
//   100MHz -> 1클럭 = 10ns
//   cnt == 999 -> 1000클럭마다 tick 발생
//   1000 x 10ns = 10us
//
// 즉:
//   10us마다 1클럭 폭 tick 발생
//
// 주의
// - tick은 느린 클럭이 아니라 1-cycle pulse
// - i2c_master, mpu6050_ctrl가 tick 기준으로 한 단계씩 진행됨
// - uart_tx는 이 tick을 쓰지 않고 100MHz clk를 직접 사용
// ================================================================================

module clk_divider
(
    input  wire clk,      // 100MHz system clock
    input  wire rst_n,    // Active low reset
    output reg  tick      // 1-cycle tick pulse
);

    //--------------------------------------------------------------------------
    // 10us tick 생성용 카운터
    //--------------------------------------------------------------------------
    reg [15:0] cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt  <= 16'd0;
            tick <= 1'b0;
        end
        else begin
            if (cnt == 16'd999) begin
                cnt  <= 16'd0;
                tick <= 1'b1;
            end
            else begin
                cnt  <= cnt + 16'd1;
                tick <= 1'b0;
            end
        end
    end

endmodule