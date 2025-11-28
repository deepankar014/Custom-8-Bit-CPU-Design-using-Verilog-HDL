module ALU(input [7:0] A,input [7:0] B,input [2:0] opcode, output reg [7:0] result, output reg zero);

    always @(*) begin
        result = 8'b00000000; 

        case(opcode)
            3'b000: result = A + B;
            3'b001: result = A - B;
            default: result = 8'b00000000; 
        endcase

        if (result == 8'b00000000)
            zero = 1'b1;
        else
            zero = 1'b0;
    end

endmodule
