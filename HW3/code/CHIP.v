`define OPCODE_LOAD     7'b0000011
`define OPCODE_STORE    7'b0100011
`define OPCODE_BRANCH   7'b1100011
`define OPCODE_OP_IMM   7'b0010011
`define OPCODE_JAL      7'b1101111
`define OPCODE_JALR     7'b1100111
`define OPCODE_LUI      7'b0110111
`define OPCODE_AUIPC    7'b0010111

module CHIP
    (clk,
     rst_n,
     mem_wen_D,
     mem_addr_D,
     mem_wdata_D,
     mem_rdata_D,
     mem_addr_I,
     mem_rdata_I
    );

    input         clk, rst_n ;
    output        mem_wen_D  ;  // mem_wen_D is high, CHIP writes data to D-mem; else, CHIP reads data from D-mem
    output [31:0] mem_addr_D ;  // the specific address to fetch/store data
    output [31:0] mem_wdata_D;  // data writing to D-mem
    input  [31:0] mem_rdata_D;  // data reading from D-mem
    output [31:0] mem_addr_I ;  // the fetching address of next instruction
    input  [31:0] mem_rdata_I;  // instruction reading from I-mem

    function [31:0] BE32;
        input [31:0] word;
        BE32 = {word[7:0],
                word[15:8],
                word[23:16],
                word[31:24]};
    endfunction

    function [31:0] LE32;
        input [31:0] word;
        LE32 = BE32(word);
    endfunction

    wire RegWrite, ALUSrc, MemtoReg, Branch, zero, and2or, jal, or2Mux4;
    wire jalr, Or2toMux3, MemRead;
    wire [31:0] RegFile2Mux, ImmGen2Mux, ALUin2, RegFile2ALUin1;
    wire [31:0] Mux2toMux3in1, Mux4toMux5, Adder3toMux5, Adder2toMux4;
    wire [31:0] D_MEM_rdata, I_MEM_rdata, D_MEM_wdata;
    wire [31:0] Adder1toMux4, Mux5toPC, WriteData, tmp;
    wire [3:0] ALUop;

    assign D_MEM_rdata = BE32(mem_rdata_D);
    assign I_MEM_rdata = BE32(mem_rdata_I);
    assign mem_wdata_D = LE32(D_MEM_wdata);

    assign D_MEM_wdata = RegFile2Mux;
    assign Or2toMux3 = jal | jalr;
    assign and2or = Branch & zero;
    assign or2Mux4 = and2or | jal;

    PC PC(.clk(clk),
          .rst_n(rst_n),
          .pc_in(Mux5toPC),
          .pc_out(mem_addr_I)
         );

    Register_File regster_file(.read_reg_1(I_MEM_rdata[19:15]),
                               .read_reg_2(I_MEM_rdata[24:20]),
                               .write_reg(I_MEM_rdata[11:7]),
                               .write_data(WriteData),
                               .read_data_1(RegFile2ALUin1),
                               .read_data_2(RegFile2Mux),
                               .reg_wen(RegWrite),
                               .clk(clk),
                               .rst_n(rst_n)
                              );

    ImmGen ImmGen(.inst(I_MEM_rdata),
                  .extend_imm(ImmGen2Mux)
                 );

    control_unit control_unit(.clk(clk),
                              .rst_n(rst_n),
                              .inst(I_MEM_rdata),
                              .Branch(Branch),
                              .MemRead(MemRead),
                              .MemWrite(mem_wen_D),
                              .ALUOp(ALUop),
                              .ALUSrc(ALUSrc),
                              .RegWrite(RegWrite),
                              .MemtoReg(MemtoReg),
                              .jal(jal),
                              .jalr(jalr)
                             );

    MUX MUX1(.sel(ALUSrc),
             .in1(RegFile2Mux),
             .in2(ImmGen2Mux),
             .out(ALUin2)
            );

    ALU ALU(.ALUOp(ALUop),
            .in1(RegFile2ALUin1),
            .in2(ALUin2),
            .out(mem_addr_D),
            .zero(zero)
           );

    MUX MUX2(.sel(MemtoReg),
             .in1(mem_addr_D),
             .in2(D_MEM_rdata),
             .out(Mux2toMux3in1)
            );

    MUX MUX3(.sel(Or2toMux3),
             .in1(Mux2toMux3in1),
             .in2(Adder1toMux4),
             .out(WriteData)
            );

    MUX MUX4(.sel(or2Mux4),
             .in1(Adder1toMux4),
             .in2(Adder2toMux4),
             .out(Mux4toMux5)
            );

    MUX MUX5(.sel(jalr),
             .in1(Mux4toMux5),
             .in2(Adder3toMux5),
             .out(Mux5toPC)
            );

    Adder Adder1(.in1(mem_addr_I),
                 .in2(32'b100),
                 .out(Adder1toMux4)
                );

    Adder Adder2(.in1(mem_addr_I),
                 .in2(ImmGen2Mux),
                 .out(Adder2toMux4)
                );

    Adder Adder3(.in1(ImmGen2Mux),
                 .in2(RegFile2ALUin1),
                 .out(Adder3toMux5)
                );
endmodule


module PC #(
        parameter ADDR_W = 32,
        parameter INST_W = 32,
        parameter DATA_W = 32
    )(
        input                       clk,
        input                       rst_n,
        input       [ADDR_W-1:0]    pc_in,
        output  reg [ADDR_W-1:0]    pc_out
    );

    always @(posedge clk)
    begin
        if (~rst_n)
        begin
            pc_out <= 32'b0;
        end
        else
        begin
            pc_out <= pc_in;
        end
    end
endmodule

module Adder #(
        parameter ADDR_W = 32,
        parameter INST_W = 32,
        parameter DATA_W = 32
    )(
        input   [DATA_W-1:0]    in1,
        input   [DATA_W-1:0]    in2,
        output  [DATA_W-1:0]    out
    );
    assign out = in1 + in2;
endmodule

module MUX #(
        parameter ADDR_W = 32,
        parameter INST_W = 32,
        parameter DATA_W = 32
    )(
        input   [DATA_W-1:0]    in1,
        input   [DATA_W-1:0]    in2,
        input                   sel,
        output  [DATA_W-1:0]    out
    );
    assign out = (sel == 0) ? in1 : in2;
endmodule

module Register_File #(
        parameter ADDR_W = 32,
        parameter INST_W = 32,
        parameter DATA_W = 32
    )(
        input  [4:0]        read_reg_1,
        input  [4:0]        read_reg_2,
        input  [4:0]        write_reg,
        input  [DATA_W-1:0] write_data,
        input  [DATA_W-1:0] read_data_1,
        input  [DATA_W-1:0] read_data_2,
        input               reg_wen,
        input               clk,
        input               rst_n
    );

    reg [31:0] reg_file [0:31];
    reg [31:0] reg_file_tmp1;
    reg [31:0] reg_file_tmp2;
    integer i;

    always @(posedge clk)
    begin
        if (~rst_n)
        begin
            for (i = 0; i <= 31; i = i + 1)
            begin
                reg_file[i] <= 32'b0;
            end
        end
        else
        begin
            if (reg_wen)
            begin
                if (write_reg != 0)
                    reg_file[write_reg] <= write_data;
            end
        end
    end
    assign read_data_1 = reg_file[read_reg_1];
    assign read_data_2 = reg_file[read_reg_2];
endmodule


module ImmGen #(
        parameter ADDR_W = 32,
        parameter INST_W = 32,
        parameter DATA_W = 32
    )(
        input   [INST_W-1:0]    inst,
        output  [INST_W-1:0]    extend_imm
    );

    reg [31:0] tmp;

    always @(*)
    begin
        case(inst[6:0])
            `OPCODE_LOAD:
                tmp = {{20{inst[31]}}, inst[31:20]};
            `OPCODE_STORE:
                tmp = {{20{inst[31]}}, inst[31:25], inst[11:7]};
            `OPCODE_BRANCH:
                tmp = {{20{inst[31]}}, inst[7], inst[30:25], inst[11:8], 1'b0};
            `OPCODE_OP_IMM:
                tmp = {{20{inst[31]}}, inst[31:20]};
            `OPCODE_JAL:
                tmp = {{12{inst[31]}}, inst[19:12], inst[20], inst[30:25], inst[24:21], 1'b0};
            `OPCODE_JALR:
                tmp = {{21{inst[31]}}, inst[30:20]};
            `OPCODE_LUI:
                tmp = {inst[31:12], 12'b0};
            `OPCODE_AUIPC:
                tmp = {inst[31:12], 12'b0};
            default:
                tmp = 32'b0;
        endcase
    end

    assign extend_imm = tmp;
endmodule

module control_unit #(
        parameter ADDR_W = 32,
        parameter INST_W = 32,
        parameter DATA_W = 32
    )(
        input                   clk,
        input                   rst_n,
        input   [INST_W-1:0]    inst,
        output         [3:0]    ALUOp,
        output                  Branch,
        output                  MemRead,
        output                  MemWrite,
        output                  ALUSrc,
        output                  RegWrite,
        output                  MemtoReg,
        output                  jal,
        output                  jalr
    );

    reg [3:0] ALUOp_tmp;
    reg Branch_tmp, MemRead_tmp, MemWrite_tmp, ALUSrc_tmp, RegWrite_tmp, MemtoReg_tmp, jal_tmp, jalr_tmp;

    reg [6:0] opcode;
    reg [2:0] func3;
    reg [6:0] func7;

    always @(*)
    begin
        if (~rst_n)
        begin
            ALUOp_tmp = 0;
            Branch_tmp = 0;
            MemRead_tmp = 0;
            MemWrite_tmp = 0;
            ALUSrc_tmp = 0;
            RegWrite_tmp = 0;
            MemtoReg_tmp = 0;
            jal_tmp = 0;
            jalr_tmp = 0;
        end
        opcode = inst[6:0];
        func3 = inst[14:12];
        func7 = inst[31:25];

        if (opcode == 7'b0110011)
        begin
            if (func3 == 3'b000 && func7 == 7'b0000000) // add
            begin
                ALUOp_tmp = 4'b0010;
                Branch_tmp = 0;
                MemRead_tmp = 0;
                MemWrite_tmp = 0;
                ALUSrc_tmp = 0;
                RegWrite_tmp = 1;
                MemtoReg_tmp = 0;
                jal_tmp = 0;
                jalr_tmp = 0;
            end
            else if (func3 == 3'b000 && func7 == 7'b0100000) // sub
            begin
                ALUOp_tmp = 4'b0110;
                Branch_tmp = 0;
                MemRead_tmp = 0;
                MemWrite_tmp = 0;
                ALUSrc_tmp = 0;
                RegWrite_tmp = 1;
                MemtoReg_tmp = 0;
                jal_tmp = 0;
                jalr_tmp = 0;
            end
            else if (func3 == 3'b111 && func7 == 7'b0000000) // and
            begin
                ALUOp_tmp = 4'b0000;
                Branch_tmp = 0;
                MemRead_tmp = 0;
                MemWrite_tmp = 0;
                ALUSrc_tmp = 0;
                RegWrite_tmp = 1;
                MemtoReg_tmp = 0;
                jal_tmp = 0;
                jalr_tmp = 0;
            end
            else if (func3 == 3'b110 && func7 == 7'b0000000) // or
            begin
                ALUOp_tmp = 4'b0001;
                Branch_tmp = 0;
                MemRead_tmp = 0;
                MemWrite_tmp = 0;
                ALUSrc_tmp = 0;
                RegWrite_tmp = 1;
                MemtoReg_tmp = 0;
                jal_tmp = 0;
                jalr_tmp = 0;
            end
            else if (func3 == 3'b010 && func7 == 7'b0000000) // slt
            begin
                ALUOp_tmp = 4'b1000;
                Branch_tmp = 0;
                MemRead_tmp = 0;
                MemWrite_tmp = 0;
                ALUSrc_tmp = 0;
                RegWrite_tmp = 1;
                MemtoReg_tmp = 0;
                jal_tmp = 0;
                jalr_tmp = 0;
            end
        end
        else if (opcode == 7'b0000011)
        begin
            if (func3 == 3'b010) // lw
            begin
                ALUOp_tmp = 4'b0010;
                Branch_tmp = 0;
                MemRead_tmp = 1;
                MemWrite_tmp = 0;
                ALUSrc_tmp = 1;
                RegWrite_tmp = 1;
                MemtoReg_tmp = 1;
                jal_tmp = 0;
                jalr_tmp = 0;
            end
        end
        else if (opcode == 7'b0100011)
        begin
            if(func3 == 3'b010) // sw
            begin
                ALUOp_tmp = 4'b0010;
                Branch_tmp = 0;
                MemRead_tmp = 0;
                MemWrite_tmp = 1;
                ALUSrc_tmp = 1;
                RegWrite_tmp = 0;
                MemtoReg_tmp = 0;
                jal_tmp = 0;
                jalr_tmp = 0;
            end
        end
        else if (opcode == 7'b1100011)
        begin
            if (func3 == 3'b000) // beq
            begin
                ALUOp_tmp = 4'b0110;
                Branch_tmp = 1;
                MemRead_tmp = 0;
                MemWrite_tmp = 0;
                ALUSrc_tmp = 0;
                RegWrite_tmp = 0;
                MemtoReg_tmp = 0;
                jal_tmp = 0;
                jalr_tmp = 0;
            end
        end
        else if (opcode == 7'b1101111) // jal
        begin
            ALUOp_tmp = 4'bx;
            Branch_tmp = 0;
            MemRead_tmp = 0;
            MemWrite_tmp = 0;
            ALUSrc_tmp = 0;
            RegWrite_tmp = 1;
            MemtoReg_tmp = 0;
            jal_tmp = 1;
            jalr_tmp = 0;
        end
        else if (opcode == 7'b1100111)
        begin
            if (func3 == 3'b000) // jalr
            begin
                ALUOp_tmp = 4'bx;
                Branch_tmp = 0;
                MemRead_tmp = 0;
                MemWrite_tmp = 0;
                ALUSrc_tmp = 0;
                RegWrite_tmp = 1;
                MemtoReg_tmp = 0;
                jal_tmp = 0;
                jalr_tmp = 1;
            end
        end
    end
    assign ALUOp = ALUOp_tmp;
    assign Branch = Branch_tmp;
    assign MemRead = MemRead_tmp;
    assign MemWrite = MemWrite_tmp;
    assign ALUSrc = ALUSrc_tmp;
    assign RegWrite = RegWrite_tmp;
    assign MemtoReg = MemtoReg_tmp;
    assign jal = jal_tmp;
    assign jalr = jalr_tmp;
endmodule


module ALU #(
        parameter ADDR_W = 32,
        parameter INST_W = 32,
        parameter DATA_W = 32
    )(
        input   [3:0]           ALUOp,
        input   [DATA_W-1:0]    in1,
        input   [DATA_W-1:0]    in2,
        output  [DATA_W-1:0]    out,
        output                  zero
    );

    reg [31:0] tmp;

    assign out = (ALUOp == 4'b0000) ? in1 & in2
           : (ALUOp == 4'b0001) ? in1 | in2
           : (ALUOp == 4'b0010) ? in1 + in2
           : (ALUOp == 4'b0110) ? in1 - in2
           : (ALUOp == 4'b1000) ? in1 < in2
           : 32'b0;

    assign zero = out == 32'b0 ? 1'b1 : 1'b0;
endmodule

module MUX3 #(
        parameter ADDR_W = 32,
        parameter INST_W = 32,
        parameter DATA_W = 32
    )(
        input   [1:0]           sel,
        input   [DATA_W-1:0]    in1,
        input   [DATA_W-1:0]    in2,
        input   [DATA_W-1:0]    in3,
        output  [DATA_W-1:0]    out
    );

    assign out = (sel == 2'b00) ? in1
           : (sel == 2'b01) ? in2
           : (sel == 2'b10) ? in3
           : 32'b0;
endmodule
