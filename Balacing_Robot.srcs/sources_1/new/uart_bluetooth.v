`timescale 1ns / 1ps


// module uart_bluetooth(
//     input  clk,
//     input  reset,
//     input  rx_pin,
//     output tx_pin
// );

//     wire        b_tick;
//     wire [7:0]  rx_data;
//     wire        rx_done;

//     // echo 로직: rx_done 펄스 → tx_data/tx_start 생성
//     reg [7:0] echo_data;
//     reg       echo_start;

//     always @(posedge clk or posedge reset) begin
//         if (reset) begin
//             echo_data  <= 0;
//             echo_start <= 0;
//         end else begin
//             echo_start <= 0;          // 기본 0 (1클럭 펄스)
//             if (rx_done) begin
//                 echo_data  <= rx_data;
//                 echo_start <= 1;
//             end
//         end
//     end

//     baud_rate u_baud(
//         .clk(clk),
//         .reset(reset),
//         .b_tick(b_tick)
//     );

//     uart_tx u_tx(
//         .clk(clk),
//         .reset(reset),
//         .b_tick(b_tick),
//         .tx_start(echo_start),
//         .tx_data(echo_data),
//         .tx_pin(tx_pin),
//         .tx_done()              // echo 용도에선 미사용
//     );

//     uart_rx u_rx(
//         .clk(clk),
//         .reset(reset),
//         .rx_pin(rx_pin),
//         .rx_data(rx_data),
//         .rx_done(rx_done)
//     );

// endmodule


module baud_rate (
    input clk,
    input reset,
    output reg b_tick
);

reg [13:0] b_cnt;             //100M / 9600 = 10416 정도이므로 log2(10416) = 13.3이므로 14비트가 필요

always @(posedge clk) begin
    if (reset) begin
        b_cnt <= 0;
        b_tick <= 0;
    end else if (b_cnt == 14'd10415) begin
        b_cnt <= 0;
        b_tick <= 1;
    end else begin
        b_cnt <= b_cnt + 1;
        b_tick <= 0;
    end
end
endmodule

module uart_tx (
    input clk,
    input reset,
    input b_tick,           // baud rate tick
    input tx_start,
    input [7:0] tx_data,
    output reg tx_pin,          //tx 핀 출력
    output reg tx_done
);

    localparam IDLE  = 2'd0;
    localparam START = 2'd1;
    localparam DATA  = 2'd2;
    localparam STOP  = 2'd3;

    reg [1:0] state;
    reg [2:0] bit_cnt;      // 1바이트라 0~7까지 카운팅
    reg [7:0] tx_buf;       // 8bit tx_data (임시 저장)

    always @(posedge clk or posedge reset) begin
        if(reset) begin
            state <= IDLE;
            tx_pin <= 1'b1;
            tx_done <= 1'b0;
            bit_cnt <= 0;
            tx_buf <= 0;
        end else begin
            
            tx_done <= 1'b0;

            case (state)
                IDLE : begin
                    tx_pin <= 1'b1;
                    if (tx_start) begin
                        tx_buf <= tx_data;
                        state <= START;
                    end
                end

                START : begin
                    if (b_tick) begin
                        tx_pin <= 1'b0;
                        state <= DATA;
                    end
                end

                DATA : begin
                    if (b_tick) begin
                        tx_pin <= tx_buf[0];                // 0번비트 대입
                        if(bit_cnt != 7)begin
                            bit_cnt <= bit_cnt + 1;
                            tx_buf <= tx_buf >> 1;          // 이후 비트시프팅
                        
                        end else begin
                            state <= STOP;
                            bit_cnt <= 0;
                        end
                    end
                end
                STOP : begin
                    if (b_tick) begin
                        tx_pin <= 1'b1;
                        tx_done <= 1'b1;
                        state <= IDLE;
                    end
                end
            
                default: ;
            endcase

        end
    end

endmodule

module uart_rx (
    input clk,
    input reset,
    input rx_pin,           //rx pin 입력
    output reg [7:0] rx_data,          
    output reg rx_done
);
    
    reg [1:0] state;
    reg [3:0] tick_cnt;
    reg [2:0] bit_cnt;
    reg [7:0] rx_buf;
    localparam IDLE  = 2'd0;
    localparam START = 2'd1;
    localparam DATA  = 2'd2;
    localparam STOP  = 2'd3;

    reg [9:0] b_cnt_16;             

    wire tick16;
    assign tick16 = (b_cnt_16 == 10'd650);
 
    always @(posedge clk or posedge reset) begin
        if (reset) b_cnt_16 <=0;
        else if (tick16) b_cnt_16 <= 0;
        else b_cnt_16 <= b_cnt_16 + 1;        
    end


always @(posedge clk or posedge reset) begin
        if(reset) begin
            state <= IDLE;
            tick_cnt <= 0;
            bit_cnt <= 0;
            rx_done <= 1'b0;
            rx_data <= 0;
            rx_buf <= 0;
        end else begin
            
            rx_done <= 1'b0;

            case (state)
                IDLE : begin
                    if (rx_pin == 1'b0) begin
                        tick_cnt <= 0;
                        state <= START;
                    end
                end

                START : begin
                    if (tick16) begin
                        if (tick_cnt == 4'd7) begin
                            if (rx_pin == 1'b0) begin
                                tick_cnt <= 0;
                                bit_cnt <= 0;
                                state <= DATA;
                            end else
                                state <= IDLE;
                        end else
                            tick_cnt <= tick_cnt + 1;
                    end
                end

                DATA : begin
                    if (tick16) begin
                        if (tick_cnt == 4'd15) begin
                            rx_buf <= {rx_pin, rx_buf [7:1]};
                            tick_cnt <= 0;
                            if(bit_cnt == 3'd7) begin
                                state <= STOP;
                            end else
                                bit_cnt <= bit_cnt + 1;
                        end else
                            tick_cnt <= tick_cnt + 1;
                    end
                end

                STOP : begin
                    if (tick16) begin
                        if (tick_cnt == 4'd15) begin
                            rx_data <= rx_buf;
                            rx_done <= 1'b1;
                            state <= IDLE;
                        end else
                            tick_cnt <= tick_cnt + 1;
                    end
                end
            
                default: ;
            endcase

        end
    end


endmodule