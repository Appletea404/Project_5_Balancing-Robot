`timescale 1ns / 1ps
// ================================================================================
// clk_divider -> 100MHz 시스템 클럭을 분주하여
//                I2C FSM 또는 제어 FSM에서 사용할 느린 tick을 생성하는 모듈
//
// 100MHz => 1초에 100,000,000번 => 주기 = 10ns
// cnt == 49,999 => 50,000클럭 => 50,000 x 10ns = 0.5ms
// => 0.5ms마다 tick 발생 => 2kHz
//
// tick은 2kHz 클럭이 아니라,
// 0.5ms마다 1클럭 동안만 1이 되는 펄스 신호
//
// FSM을 100MHz로 직접 돌리지 않고,
// tick 기준으로 한 단계씩 진행하기 위해 사용
// ================================================================================


module clk_divider
(

    input wire clk,         // 100MHz system clock
    input wire rst_n,       // Active low reset(0 = 리셋 동작/ 1 = 정상 동작)
    output reg tick         // 1-cycle tick output

);

// 분주 카운터
reg [15:0] cnt;


// ================================================================================
// 카운터를 증가시키다가 특정 값에 도달하면 tick을 1클럭 동안 출력
// ================================================================================

always @(posedge clk or negedge rst_n) begin
    // rst_n == 0 이면 초기화
    if (!rst_n) begin
        cnt  <= 16'd0;
        tick <= 1'b0;
    end

    // 목표 카운트 도달 검사
    else begin
        if (cnt == 16'd49999) begin
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


