
module instruction_memory( input rst,input [7:0] address, input memread,output reg [7:0] instruction);
reg [7:0] memory [255:0];
always@(*) begin 
    if(rst) begin
        instruction <= 8'b00000000;
    end
    else if(memread) begin
        instruction <= memory[address];
    end
end
endmodule

module instruction_register(input clk, input rst, input irwrite, input [7:0] instruction_in, output reg [7:0] instruction_out);

always@(posedge clk or posedge rst) begin
    if(rst) begin
        instruction_out <= 8'b00000000;
    end
    else if(irwrite) begin
        instruction_out <= instruction_in;
    end
end
endmodule
