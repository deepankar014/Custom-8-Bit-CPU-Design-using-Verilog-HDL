module Register_File(input clk,input rst, input regwrite, input [2:0] readadd1,input [2:0] readadd2, input [2:0] writeadd, input [7:0] writedata,output reg [7:0] readdata1,output reg[7:0] readdata2);
    reg [7:0] register[7:0];
    always@(*) begin
        readdata1 = register[readadd1];
        readdata2 = register[readadd2];
    end

    always@(posedge clk or posedge rst)begin
        if(rst) begin
            register[0] <= 8'b00000000;
            register[1] <= 8'b00000000;
            register[2] <= 8'b00000000;
            register[3] <= 8'b00000000;
            register[4] <= 8'b00000000;
            register[5] <= 8'b00000000;
            register[6] <= 8'b00000000;
            register[7] <= 8'b00000000;
        end
        else if(regwrite) begin
            register[writeadd] <= writedata;
        end
    end
endmodule
