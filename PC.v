module program_counter(input clk, input rst, input pcwrite , input [7:0] pcin, output reg[7:0] pcout);
always@(posedge clk or posedge rst) begin
    if(rst) begin
        pcout <= 8'b00000000;
    end
    else if(pcwrite) begin
        pcout <= pcin;
    end
    else  
       pcout <= pcout + 8'd4;
end
endmodule
    
