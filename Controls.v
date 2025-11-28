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

