module memory(input clk,input rst, input memwrite,input memread,input [7:0] address, input [7:0] writedata, output reg [7:0] readdata);
reg [7:0] mem[255:0];
integer i;

always @(*) begin
    if (memread)
        readdata = mem[address];
    else
        readdata = readdata; // hold previous value (optional)
end
always@(posedge clk or posedge rst)
     begin
        if (rst) begin
            for (i = 0; i < 256; i = i + 1) 
                begin
                    mem[i] <= 8'b00000000;
                end
            end
     end
        else if(memwrite) 
            begin
                mem[address] <= writedata;
            end
endmodule       
