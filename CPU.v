module cpu_top(input clk , input rst);

//internal signals connecting datapath and control path
wire [2:0] opcode;
wire [2:0] mode_flag;
wire zero, carry, negative, overflow;
wire pcwrite, irwrite, IRmemread, memread, memwrite;
wire regwrite, alusrc, memtoreg, pc_select_control;
wire [1:0] aluop;


cpu_datapath datapath(.clk(clk),.rst(rst),.pcwrite(pcwrite),.irwrite(irwrite),.memread(memread),.IRmemread(IRmemread),.memwrite(memwrite),.pc_select_control(pc_select_control),.regwrite(regwrite),.alusrc(alusrc),.memtoreg(memtoreg),.aluop(aluop),.opcode(opcode),.mode_flag(mode_flag),.zero(zero),.carry(carry),.negative(negative),.overflow(overflow));

controller ctrl(.clk(clk),.rst(rst),.opcode(opcode),.mode_flag(mode_flag),.zero(zero),.carry(carry),.negative(negative),.overflow(overflow),.IRmemread(IRmemread),.pcwrite(pcwrite),.pc_select_control(pc_select_control),.irwrite(irwrite),.memread(memread),.memwrite(memwrite),.regwrite(regwrite),.memtoreg(memtoreg),.alusrc(alusrc),.aluop(aluop));

endmodule


module cpu_datapath(input clk ,input rst,input pcwrite,input irwrite,input memread,input IRmemread, input memwrite, input pc_select_control,input regwrite, input alusrc,input memtoreg, input[1:0] aluop ,output wire [2:0] opcode,output wire [2:0] mode_flag, output reg zero, output reg carry,output reg negative,output reg overflow);

// ALU 
wire [7:0] A,B;
wire [7:0] alu_result;
reg [7:0] alu_result_reg; 
wire  zero_w,carry_w,neg_w,ovf_w;
//memory
wire [7:0] mem_address,writedata,readdata;
reg [7:0] mem_address_reg;

//PC
wire [23:0] pcin,pcout;

//reg_file4
wire [2:0] readadd1,readadd2,writeadd;
wire[7:0] writedata_reg,readdata1,readdata2;
//IRmem_IR
wire [23:0] IRaddress, IRMEM_instruction;
//IR
wire [23:0] instruction_in,instruction_out;
//mux_pc
wire [23:0] adder1_result,adder2_result;
//sign_extender
wire [7:0] data_in;
wire [15:0] data_out_16;
//shift_left_2
wire [7:0] data_in_to_shift, data_shifted;
//mux ALU
wire [7:0] sel_alu_out;
//MUX Reg_file
wire [7:0] out_mux_reg;
//immediate
wire [8:0] immediate;    

//module instantiations
MUX_pc mod_mux_pc(adder1_result,adder2_result,pc_select_control,pcin);
PC pc_mod(clk ,rst ,pcwrite, pcin ,pcout);
pc_adder mod_pc_adder(pcout,adder1_result);
IRmem mod_IRMEM(clk,rst, IRaddress,IRmemread,IRMEM_instruction);
IR mod_IR(clk,rst,irwrite,instruction_in,instruction_out);
sign_extender mod_sign_extender(data_in,data_out_16);
shift_left_2 mod_shift_left(data_in_to_shift,data_shifted);
Adder2 mod_addr2(adder1_result, data_shifted, adder2_result);
Reg_File mod_reg(clk,rst,regwrite,readadd1,readadd2,writeadd,writedata_reg,readdata1,readdata2);
MUX mod_mux_alu(readdata2,imm_8bit,alusrc,sel_alu_out);
ALU alu_mod (A, B, aluop, alu_result, zero_w, carry_w, neg_w, ovf_w);
Memory mem_mod( clk, rst,  memwrite, memread, mem_address,  writedata, readdata );
MUX mod_mux_reg(alu_result_reg,readdata,memtoreg,out_mux_reg);

//assignments
assign instruction_in = IRMEM_instruction;
assign IRaddress = pcout;
assign A = readdata1;
assign B = sel_alu_out; 
assign data_in_to_shift = data_out_16[7:0];
assign writedata = readdata2;
assign data_in = immediate[7:0];
assign writedata_reg = out_mux_reg;
always @(posedge clk or posedge rst) begin
    if (rst)
        mem_address_reg <= 8'b0;
    else
        mem_address_reg <= alu_result;
end

assign mem_address = mem_address_reg;
//alu result reg cuz it doesn't get stored
always @(posedge clk or posedge rst) begin
    if (rst) begin
        alu_result_reg <= 8'b0;
    end else begin
        alu_result_reg <= alu_result;  // Latch ALU result every cycle
    end
end
always @(posedge clk or posedge rst) begin
    if (rst) begin
        zero <= 0;
        carry <= 0;
        negative <= 0;
        overflow <= 0;
    end else begin
        zero <= zero_w;
        carry <= carry_w;
        negative <= neg_w;
        overflow <= ovf_w;
    end
end
//instruction Division
assign opcode = instruction_out[23:21];
assign writeadd = instruction_out[20:18];   // ✓ Rd - destination
assign readadd1 = instruction_out[17:15];   // ✓ Rs1 - first source
assign readadd2 = instruction_out[14:12];   // ✓ Rs2 - second source
assign immediate = instruction_out[11:3];
assign mode_flag = instruction_out[2:0];
wire [7:0] imm_8bit;
assign imm_8bit = immediate[7:0];
endmodule
//Datapath
//1). mux_pc
module MUX_pc(input [23:0] x , input[23:0] y, input sel , output  [23:0] sel_out);
    assign sel_out = sel ? y : x;
endmodule
//2). Mux
module MUX(input [7:0] x , input[7:0] y, input sel , output  [7:0] sel_out);
    assign sel_out = sel ? y : x;
endmodule

//2). PC
module PC(input clk , input rst , input pcwrite, input [23:0] pcin, output reg [23:0] pcout);
always@(posedge clk or posedge rst) begin
    if(rst) begin
        pcout <=  24'd0;
    end else if(pcwrite)
        pcout <= pcin;
        else
        pcout <= pcout;
end
endmodule

//3). pc_adder
module pc_adder(input [23:0] pcout, output wire [23:0] adder1_result);
assign adder1_result = pcout + 24'd1;
endmodule

//4). IRmem
module IRmem( input clk, input rst,input [23:0] address, input memread,output reg [23:0] instruction);
reg [23:0] memory [255:0];
integer i;
always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < 256; i = i + 1)
                memory[i] <= 24'b0;
        end
    end
    
    // Combinational read
    always @(*) begin
        if (rst)
            instruction = 24'b0;
        else if (memread)
            instruction = memory[address];
        else
            instruction = 24'b0;
    end
endmodule

//5).IR
module IR(input clk, input rst, input irwrite, input [23:0] instruction_in, output reg [23:0] instruction_out);
always@(posedge clk or posedge rst) begin
    if(rst) begin
        instruction_out <= 24'b0;
    end
    else if(irwrite) begin
        instruction_out <= instruction_in;
    end
end
endmodule

//6).sign_extender
module sign_extender(input [7:0] data_in,output [15:0] data_out_16 );
assign data_out_16 = {{8{data_in[7]}}, data_in};
endmodule

//7).shift_left_2
module shift_left_2(input [7:0] data_in_to_shift,output [7:0] data_shifted);
assign data_shifted = data_in_to_shift <<2;
endmodule

//8).Adder2
module Adder2(input [23:0] adder1_result, input [7:0] data_shifted, output wire [23:0] adder2_result);
assign adder2_result = adder1_result + data_shifted;
endmodule

//9).Reg_File
module Reg_File(input clk,input rst, input regwrite, input [2:0] readadd1,input [2:0] readadd2, input [2:0] writeadd, input [7:0] writedata,output reg [7:0] readdata1,output reg[7:0] readdata2);
    reg [7:0] register[7:0];
    always@(*) begin
        if (rst) begin
        readdata1 = 8'b0;
        readdata2 = 8'b0;
    end else begin
        readdata1 = register[readadd1];
        readdata2 = register[readadd2];
    end
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
        else if(regwrite) 
            register[writeadd] <= writedata;
        
    end
    
endmodule

//10). ALU
module ALU(
    input [7:0] A, 
    input [7:0] B, 
    input [1:0] aluop,           // Changed from opcode to aluop
    output reg [7:0] result, 
    output  reg zero, 
    output  reg carry, 
    output  reg negative, 
    output  reg overflow
);
    
    reg [8:0] temp_result;  
    
    always @(*) begin
        // Default values to avoid latches
        carry = 0;
        overflow = 0;
        temp_result = 9'b0;
        
        case(aluop)
            2'b00: begin  // ADD
                temp_result = A + B;
                result = temp_result[7:0];
                carry = temp_result[8];
                // Overflow: both operands same sign, result different sign
                overflow = (A[7] == B[7]) && (result[7] != A[7]);
            end
            
             2'b01: begin  // SUB
                result = A - B;
                carry = (A < B);
                overflow = (A[7] != B[7]) && (result[7] == B[7]);
            end
            
            2'b10: begin //and
                result = A & B;
            end
            
            2'b11 : begin  // or
                 result = A | B;
            end
            default: begin
                result = 8'h00;
            end
        endcase
        
        // Update flags based on result
        zero = (result == 8'h00);
        negative = result[7];
    end
endmodule

//11). Memory
module Memory(input clk,input rst, input memwrite,input memread,input [7:0] address, input [7:0] writedata, output reg [7:0] readdata);
reg [7:0] mem[255:0];
integer i;


    always @(posedge clk or posedge rst) begin
        if (rst) begin
            readdata <= 8'b0;
            for (i = 0; i < 256; i = i + 1)
                mem[i] <= 8'b0;
        end else begin
            // Synchronous write
            if (memwrite)
                mem[address] <= writedata;
            
            // Synchronous read
            if (memread)
                readdata <= mem[address];
        end
    end
endmodule       


//control signal
module controller(input clk, input rst, input [2:0] opcode,input [2:0] mode_flag, input  zero, input  carry,input  negative,input overflow,output reg IRmemread, output reg pcwrite,output reg pc_select_control,output reg irwrite,output reg memread,output reg memwrite,output reg regwrite,output reg memtoreg,output reg alusrc,output reg [1:0]aluop );
parameter s0_fetch = 2'b00, s1_execute = 2'b01, s2_memory = 2'b10, s3_writeback = 2'b11;
reg[1:0] state;

//sequential state logic always block
always@(posedge clk or posedge rst) begin
    if(rst) begin
     state <= s0_fetch;
    end
    else 
    case(state)
    s0_fetch: 
    state <= s1_execute;
    s1_execute: 
        case(opcode)
                    3'b000 : state <= s3_writeback;
                    3'b001 : state <= s3_writeback;
                    3'b010 : state <= s3_writeback;
                    3'b011 : state <= s3_writeback;
                    3'b100 : state <= s2_memory;
                    3'b101 : state <= s2_memory;
                    3'b110 : state <= s0_fetch;
                    3'b111 : state <= s0_fetch;
                    default : state <= s0_fetch;
        endcase
    s2_memory: begin
        if(opcode == 3'b100)
            state <= s3_writeback;
        else
            state <= s0_fetch;
    end
    s3_writeback: state <= s0_fetch;
    default : state <= s0_fetch;
    endcase
end
//combinational output logic
always@(*) begin
    IRmemread = 0;
    pcwrite = 0;
    irwrite = 0;
    alusrc = 0;
    pc_select_control = 0;
    memtoreg = 0;
    memread = 0;
    memwrite = 0;
    regwrite = 0;
    aluop =2'b00;

    case(state)
        s0_fetch: begin
            pcwrite = 1;
            irwrite = 1;
            IRmemread = 1;
        end
        s1_execute: begin
            case(opcode) 
                3'b000 : begin
                    aluop = 2'b00;
                    alusrc = 0;
                end
                3'b001 : begin
                    aluop = 2'b01;
                    alusrc = 0;
                end
                3'b010: begin
                    aluop = 2'b10;
                    alusrc = 0;
                end
                3'b011 : begin
                    aluop = 2'b11;
                    alusrc = 0;
                end
                3'b100 : begin   // load 
                    aluop = 2'b00;
                    alusrc = 1;
                end
                3'b101 : begin   // store
                    aluop = 2'b00;
                    alusrc = 1;
                end
        3'b110: begin// branch
                
         case(mode_flag)
            3'b000 : begin   // branch if equal
                    if(zero == 1) begin
                        pc_select_control = 1;
                        pcwrite = 1;
                    end
            end
            3'b001 : begin // branch if not equal
                    if(zero != 1) begin
                        pc_select_control = 1;
                        pcwrite = 1;
                    end
            end
            3'b010 : begin
                    if(negative) begin
                        pc_select_control = 1;
                        pcwrite = 1;
                    end
            end
            3'b011 : begin
                    if(!negative) begin
                        pc_select_control = 1;
                        pcwrite = 1;
                    end
            end
             default: begin
                            pc_select_control = 0;
                            pcwrite = 0;
                        end
            endcase
            end
        3'b111 : begin // JUMP
            pc_select_control = 1;
            pcwrite = 1;
        end
            
            endcase
        end
        s2_memory : begin
                    alusrc = 1;
                    aluop = 2'b00;
            if(opcode == 3'b100) begin
                memread = 1;
            end
             else if (opcode == 3'b101) begin
                memwrite = 1;
                
             end
        end
        s3_writeback : begin
            regwrite = 1;

            if(opcode == 3'b100)
            memtoreg = 1;
            else
            memtoreg = 0;
        
            
        end
    endcase
end
endmodule

