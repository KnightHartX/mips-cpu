// 顶层设计
module top (
    input clk, reset,//输入时钟信号，复位信号
    input BU, BD, BL, BR,//这里是板子中间的四个按键，分别为UP,DOWN,LEFT,RIGHT四个方向的
    input [15  : 0] SW,//输入16个拨码开关信号
    output [15 : 0] LED//输出16个LED
    );
    wire [31:0] memData, writeMemData;
    wire MemWrite;
    wire [1:0] MemMode;
    wire [15:0] memAddr;
    mips _mips(clk, reset, memData, MemWrite, MemMode, writeMemData, memAddr);
    exmemory _ex_mem(clk, reset, MemWrite, MemMode, memAddr, writeMemData, SW, BU, BD, BL, BR, LED, memData);//分别定义了mips和exmemory的相关变量
endmodule
//////////////////////////////////////////////
/////////////////////////////////////////////
//储存器模块的定义
module exmemory #(parameter WIDTH = 32, ADDR_WIDTH = 16) (
    input clk, reset,
    // ========= Signal from Controller ========= //
    //来自控制器的信号
    input MemWrite,
    input [1            : 0] MemMode,//1位的MemMode信号。表示数据类型。
    input [ADDR_WIDTH-1 : 0] memAddr,//16位的memAddr信号。表示数据地址。
    input [WIDTH-1      : 0] memWriteData,//32位的memWriteData。表示要写的数据。
    // ========= I/O Devices ========= //
    //IO外设
    input [15           : 0] SW,//拨码开关的外设
    input [0            : 0] BU, BD, BL, BR,//输入：四个按键
    output reg [15      : 0] LED,//输出：led灯
    
    output reg [WIDTH-1 : 0] memReadData//输出存储器数据
    );
    wire [31 : 0] romData;
    wire [31 : 0] ramData;
    wire RamWrite;
    
    wire [1:0] IOState;//   I/O状态
    
    button _btn(clk, reset, BU, BD, BL, BR, IOState);//设置按键相关

    assign RamWrite = ~reset & MemWrite & (memAddr[15:12]==4'h1);//RamWrite为0时才能输出。reset需要为0，MemWrite为1，memAddr==0001
    //1. 4'd1 4'h1 4'b1中的d/h/b分别表示二进制、十六进制、十进制。
    //但是你将它换算到真正的数值上时，这三种表示形式放到硬件的总线上时都表示{0, 0, 0, 1}，所以是相等的。
    rom ROM(memAddr[11:2], romData);//ROM:输入地址，输出rom数据（也就是程序）
    ram RAM(clk, RamWrite, memAddr[11:2], memWriteData, ramData);//读/写RAM

    initial begin
        LED = 0;//初始设置LED=0
    end

    always @ ( * ) begin
        case (memAddr[15:12])//根据memAddr的12——15位分支
            4'h0: begin//当为0000，读rom
                case (MemMode)//根据memmode三种取值情况分支
                    2'b00: memReadData <= romData;//00为读字
                    2'b01: begin//01为读有符号数。最高位为符号位。
                        case (memAddr[1:0])//根据memaddr（存储器地址分支）
                            //00读低八位
                            2'b00: memReadData <= {{24{romData[7]}}, romData[7:0]};
                            //01读8——16位
                            2'b01: memReadData <= {{24{romData[15]}}, romData[15:8]};
                            //10读16——24位
                            2'b10: memReadData <= {{24{romData[23]}}, romData[23:16]};
                            //11读后八位
                            2'b11: memReadData <= {{24{romData[31]}}, romData[31:24]};
                        endcase
                    end
                    2'b10: begin//10为读无符号数
                        case (memAddr[1:0])
                            //无符号数最高为均为0。
                            2'b00: memReadData <= {24'b0, romData[7:0]};
                            2'b01: memReadData <= {24'b0, romData[15:8]};
                            2'b10: memReadData <= {24'b0, romData[23:16]};
                            2'b11: memReadData <= {24'b0, romData[31:24]};
                        endcase
                    end
                endcase
            end
            4'h1: begin//当为1111，读ram
                case (MemMode)
                    2'b00: memReadData <= ramData;
                    2'b01: begin
                        case (memAddr[1:0])
                            2'b00: memReadData <= {{24{ramData[7]}}, ramData[7:0]};
                            2'b01: memReadData <= {{24{ramData[15]}}, ramData[15:8]};
                            2'b10: memReadData <= {{24{ramData[23]}}, ramData[23:16]};
                            2'b11: memReadData <= {{24{ramData[31]}}, ramData[31:24]};
                        endcase
                    end
                    2'b10: begin
                        case (memAddr[1:0])
                            2'b00: memReadData <= {24'b0, ramData[7:0]};
                            2'b01: memReadData <= {24'b0, ramData[15:8]};
                            2'b10: memReadData <= {24'b0, ramData[23:16]};
                            2'b11: memReadData <= {24'b0, ramData[31:24]};
                        endcase
                    end
                endcase
            end
            4'hf: begin//当为ffff，读拨码开关和按键
                case (memAddr)
		              16'hfffe:memReadData <= {24'b0, SW[7:0]};//先读低八位，高八位补0
        		      16'hffff:memReadData <= {24'b0, SW[15:8]};//拨码开关分配地址。在读高八位
                      16'hfffb: memReadData <= {30'b0, IOState};//I/O 状态分配地址。读按钮
                endcase
            end
        endcase
    end

    always @ (posedge clk) begin
        if (reset) begin//重置时，LED为0
            LED <= 0;
           
        end else if (MemWrite && ~RamWrite) begin//如果MemWrite和RamWrite取反的值相与为1。那么说MemWrite的值必须为1，RamWrite的值必须为0。
            case (memAddr)
               16'hfffc: LED     <= {memWriteData[15:0]};//控制器将数据写入主存，然后输出LED灯情况
            endcase
        end
    end
endmodule // exmemory
//////////////////////////////////////////////
/////////////////////////////////////////////
//模块：按钮——也就是板子中央的四个
//输入clk，reset，以及按键，产生IO状态
module button (
    input clk, reset,
    input BU, BD, BL, BR,
    output reg [1:0] IOState
    );
    initial begin
        IOState = 0;//开始时设置I/O状态为0
    end
    always @ (posedge clk) begin
        if (reset | BD) IOState <= 2'b00;//center,down。中间的按钮为reset作用
        else if (BU) IOState    <= 2'b01;//up
        else if (BR) IOState    <= 2'b10;//right
        else if (BL) IOState    <= 2'b11;//left
    end//四个状态编码
endmodule // button
//////////////////////////////////////////////
/////////////////////////////////////////////
//ram模块相关代码
//RAM可写可读。
module ram #(parameter WIDTH = 32, ADDR_WIDTH = 10) (
    input clk,
    input write,
    input [ADDR_WIDTH-1 : 0] addr,
    input [WIDTH-1      : 0] inData,
    output [WIDTH-1     : 0] outData
    );
    reg [WIDTH-1 : 0] RAM[0:(1<<ADDR_WIDTH)-1];//左移一位。这里的RAM[]是一个32位的变量，存有地址位addr的值
    assign outData = RAM[addr];//读数据
    always @ (posedge clk) begin
        if (write)//如果写
            RAM[addr] <= inData;//则将写入的数据（indata）保存至相应地址
    end
endmodule // ram
//////////////////////////////////////////////
/////////////////////////////////////////////
//ROM模块
//RAM不可写
module rom #(parameter WIDTH = 32, ADDR_WIDTH = 10) (
    input [ADDR_WIDTH-1 : 0] addr,
    output [WIDTH-1     : 0] data
    );
    reg [WIDTH-1 : 0] ROM[0:(1<<ADDR_WIDTH)-1];
    initial begin
        $display("initialize rom...");
        $readmemh("hammingcode.txt",ROM);//初始化装载汉明码程序到ROM
    end
    assign data = ROM[addr];//读ROM的相关操作
endmodule // rom
//////////////////////////////////////////////
/////////////////////////////////////////////
//cpu核心部分
module mips (
    input clk, reset,
    // ========= Data from Memory =========
    //读存储器中的数据到cpu
    input [31  : 0] memData,
    // ========= Signal to Memory =========
    //向存储器发出写操作
    output MemWrite,//1为写，0为不写
    output [1  : 0] MemMode,//读的三种数据
    // ========= Data to Memory =========
    //向存储器写数据
    output [31 : 0] writeMemData,
    output [15 : 0] memAddr
    );
    wire PCWriteCond, PCWrite;//程序计数器是否启用
    wire [1:0] PCSource;
    wire IorD, MemToReg, IRWrite, RegWrite, RegDst;
    //IorD：取指令或者取数据
    wire [1:0] ALUSrcA, ALUSrcB;
    wire zero;
    wire [31:0] aluResult, aluParamData1, aluParamData2;
    wire [5:0] op, funct;
    wire [4:0] ALUOP;

    //实例化所有所需模块
    datapath _datapath(clk, reset, PCWriteCond, PCWrite, PCSource, IorD, MemToReg, IRWrite, RegWrite, RegDst, ALUSrcA, ALUSrcB, zero, aluResult, memData, op, funct, aluParamData1, aluParamData2, writeMemData, memAddr);

    controller _controller(clk, reset, op, funct, MemWrite, MemMode, PCWriteCond, PCWrite, PCSource, IorD, MemToReg, IRWrite, RegWrite, RegDst, ALUSrcA, ALUSrcB, ALUOP);

    alu _alu(aluParamData1, aluParamData2, ALUOP, zero, aluResult);

endmodule // mips
//////////////////////////////////////////////
/////////////////////////////////////////////
//控制器模块
module controller (
    input clk, reset,
    // ========= Data from Datapath =========
    //读取数据通路数据
    input [5      : 0] op,
    input [5      : 0] funct,
    // ========= Signal to Memory =========
    output reg [0 : 0] MemWrite,

    //向存储器写信号
    // Word or Byte：选择是写字还是字节
    // 2'b00: Word。00为字
    // 2'b01: Signed Byte。有符号数
    // 2'b10: Unsigned Byte。无符号数
    output reg [1 : 0] MemMode,
    // ========= Signal to Datapath =========
    //向数据通路写数据
    output reg [0 : 0] PCWriteCond, PCWrite,//程序计数器是否启用；是否写入PC
    output reg [1 : 0] PCSource,
    output reg [0 : 0] IorD,
    output reg [0 : 0] MemToReg,
    output reg [0 : 0] IRWrite,
    output reg [0 : 0] RegWrite, RegDst,
    output reg [1 : 0] ALUSrcA, ALUSrcB,
    // ========= Signal to Datapath =========
    //向数据通路写数据
    output reg [4 : 0] ALUOP
    );

    // Runtime States
    //运行时状态
    reg [5:0] state, nextState;//6位的现在状态与下一个状态

    // States
    //定义一些6位的状态码参数
    parameter FETCH                  = 6'b000000;//取指
    parameter DECODE                 = 6'b000001;//译指
    parameter MEM_ADDR_COMPUTE       = 6'b000010;//计算内存地址
    parameter MEM_READ_WORD          = 6'b000011;//读字
    parameter MEM_READ_SIGNED_BYTE   = 6'b000100;//读有符号数
    parameter MEM_READ_UNSIGNED_BYTE = 6'b000101;//读无符号数
    parameter MEM_WRITE_BACK_TO_REG  = 6'b000110;//内存数据写回寄存器
    parameter MEM_WRITE_WORD         = 6'b000111;//向内存写字
    parameter MEM_WRITE_BYTE         = 6'b001000;//向内存写字节
    parameter RTYPE_EXCUTION         = 6'b001001;//执行R指令
    parameter RTYPE_COMPLETION       = 6'b001010;//Rtyoe执行完毕
    parameter BRANCH_COMPLETION      = 6'b001011;//分支完型
    parameter JUMP_COMPLETION        = 6'b001100;//跳转完成
    parameter IMM_EXCUTION           = 6'b001101;//立即数执行
    parameter IMM_COMPLETION         = 6'b001110;//立即数执行完毕
    parameter RESET                  = 6'b111110;//重置
    parameter HALT                   = 6'b111111;//中断

    // OP
    //指令OP
    parameter LW    = 6'b100011;  // test passed
    parameter LB    = 6'b100000;  // test passed
    parameter LBU   = 6'b100100;  // test passed
    parameter J     = 6'b000010;  // test passed
    parameter RTYPE = 6'b000000;
    parameter SB    = 6'b101000;  // test passed?
    parameter SW    = 6'b101011;  // test passed
    parameter ORI   = 6'b001101;  // test passed
    parameter LUI   = 6'b001111;  // test passed
    parameter BEQ   = 6'b000100;  // test passed
    parameter ADDI  = 6'b001000;  // test passed
    parameter ADDIU = 6'b001001;  // test passed

    // funct
    parameter SLL   = 6'b000000;  // test passed
    parameter SRL   = 6'b000010;  // test passed
    parameter SRA   = 6'b000011;  // test passed
    parameter OR    = 6'b100101;  // test passed
    parameter XOR    = 6'b100110;  // test passed
    parameter ADDU  = 6'b100001;  // test passed
    parameter ADD   = 6'b100000;  // test passed
    parameter SUB   = 6'b100010;  // test passed
    parameter SUBU  = 6'b100011;  // test passed
    parameter AND   = 6'b100100;  // test passed

    //转移状态
    always @ (posedge clk) begin
        // $display("[controller] time: %h, state: %b, MemWrite: %b, MemMode: %b, PCWriteCond: %b, PCWrite: %b, PCSource: %b, IorD: %b, MemToReg: %b, IRWrite: %b, RegWrite: %b, RegDst: %b, ALUSrcA: %b, ALUSrcB: %b, ALUOP: %b", $time, state, MemWrite, MemMode, PCWriteCond, PCWrite, PCSource, IorD, MemToReg, IRWrite, RegWrite, RegDst, ALUSrcA, ALUSrcB, ALUOP);
        if (reset) state <= RESET;
        else state <= nextState;
    end

    //状态转移
    always @ ( * ) begin
        MemWrite    <= 1'b0;
        MemMode     <= 2'b00;
        PCWriteCond <= 1'b0;
        PCWrite     <= 1'b0;
        PCSource    <= 2'b00;
        IorD        <= 1'b0;
        MemToReg    <= 1'b0;
        IRWrite     <= 1'b0;
        RegWrite    <= 1'b0;
        RegDst      <= 0;
        ALUSrcA     <= 2'b0;
        ALUSrcB     <= 2'b0;
        ALUOP       <= 5'b0;
        case (state)
            RESET: begin
                $display("[controller] time: %h, current State: RESET", $time);
                nextState <= FETCH;
            end
            FETCH: begin
                $display("[controller] time: %h, current State: FETCH", $time);
                IRWrite   <= 1;
                ALUSrcB   <= 2'b01;
                PCWrite   <= 1;
                nextState <= DECODE;
            end
            DECODE: begin
                $display("[controller] time: %h, current State: DECODE", $time);
                ALUSrcB <= 2'b11;
                case (op)
                    LW: nextState      <= MEM_ADDR_COMPUTE;
                    LB: nextState      <= MEM_ADDR_COMPUTE;
                    LBU: nextState     <= MEM_ADDR_COMPUTE;
                    J: nextState       <= JUMP_COMPLETION;
                    RTYPE: nextState   <= RTYPE_EXCUTION;
                    SB: nextState      <= MEM_ADDR_COMPUTE;
                    SW: nextState      <= MEM_ADDR_COMPUTE;
                    ORI: nextState     <= IMM_EXCUTION;
                    LUI: nextState     <= IMM_EXCUTION;
                    BEQ: nextState     <= BRANCH_COMPLETION;
                    ADDI: nextState    <= IMM_EXCUTION;
                    ADDIU: nextState   <= IMM_EXCUTION;
                    AND: nextState     <= RTYPE_EXCUTION;
                    default: nextState <= HALT;
                endcase
            end
            BRANCH_COMPLETION: begin
                $display("[controller] time: %h, current State: BRANCH_COMPLETION", $time);
                ALUSrcA     <= 2'b01;
                ALUOP       <= 5'b01001;
                PCSource    <= 2'b01;
                PCWriteCond <= 1;
                nextState   <= FETCH;
            end
            IMM_EXCUTION: begin
                $display("[controller] time: %h, current State: IMM_EXCUTION", $time);
                ALUSrcA   <= 2'b01;
                ALUSrcB   <= 2'b10;
                case (op)
                    ORI: ALUOP <= 5'b01000;
                    LUI: ALUOP <= 5'b00111;
                endcase
                nextState <= IMM_COMPLETION;
            end
            IMM_COMPLETION: begin
                $display("[controller] time: %h, current State: IMM_COMPLETION", $time);
                RegWrite  <= 1;
                nextState <= FETCH;
            end
            //执行R型指令
            RTYPE_EXCUTION: begin
                $display("[controller] time: %h, current State: RTYPE_EXCUTION", $time);
                case (funct)
                    SLL: ALUSrcA     <= 2'b10;
                    SRL: ALUSrcA     <= 2'b10;
                    SRA: ALUSrcA     <= 2'b10;
                    default: ALUSrcA <= 2'b01;
                endcase
                ALUSrcB   <= 2'b00;
                case (funct)
                    SLL: ALUOP  <= 5'b00100;
                    SRL: ALUOP  <= 5'b00101;
                    SRA: ALUOP  <= 5'b00110;
                    OR : ALUOP  <= 5'b00001;
                    XOR: ALUOP  <= 5'b00010;
                    ADDU: ALUOP <= 5'b00000;
                    ADD: ALUOP  <= 5'b00000;
                    SUB: ALUOP  <= 5'b01001;
                    SUBU: ALUOP <= 5'b01001;
                    AND: ALUOP  <= 5'b00011;
                endcase
                nextState <= RTYPE_COMPLETION;
            end
            RTYPE_COMPLETION: begin
                $display("[controller] time: %h, current State: RTYPE_COMPLETION", $time);
                RegDst    <= 1;
                RegWrite  <= 1;
                MemToReg  <= 0;
                nextState <= FETCH;
            end
            JUMP_COMPLETION: begin
                $display("[controller] time: %h, current State: JUMP_COMPLETION", $time);
                PCWrite   <= 1;
                PCSource  <= 2'b10;
                nextState <= FETCH;
            end
            MEM_ADDR_COMPUTE: begin
                $display("[controller] time: %h, current State: MEM_ADDR_COMPUTE", $time);
                ALUSrcA <= 2'b01;
                ALUSrcB <= 2'b10;
                case (op)
                    LW: nextState      <= MEM_READ_WORD;
                    LB: nextState      <= MEM_READ_SIGNED_BYTE;
                    LBU: nextState     <= MEM_READ_UNSIGNED_BYTE;
                    SB: nextState      <= MEM_WRITE_BYTE;
                    SW: nextState      <= MEM_WRITE_WORD;
                    default: nextState <= FETCH;
                endcase
            end
            MEM_READ_WORD: begin
                $display("[controller] time: %h, current State: MEM_READ_WORD", $time);
                IorD      <= 1;
                nextState <= MEM_WRITE_BACK_TO_REG;
            end
            MEM_READ_SIGNED_BYTE: begin
                $display("[controller] time: %h, current State: MEM_READ_SIGNED_BYTE", $time);
                MemMode   <= 2'b01;
                IorD      <= 1;
                nextState <= MEM_WRITE_BACK_TO_REG;
            end
            MEM_READ_UNSIGNED_BYTE: begin
                $display("[controller] time: %h, current State: MEM_READ_UNSIGNED_BYTE", $time);
                MemMode   <= 2'b10;
                IorD      <= 1;
                nextState <= MEM_WRITE_BACK_TO_REG;
            end
            MEM_WRITE_BYTE: begin
                $display("[controller] time: %h, current State: MEM_WRITE_BYTE", $time);
                MemWrite  <= 1;
                IorD      <= 1;
                MemMode   <= 2'b01;
                nextState <= FETCH;
            end
            MEM_WRITE_WORD: begin
                $display("[controller] time: %h, current State: MEM_WRITE_WORD", $time);
                MemWrite  <= 1;
                IorD      <= 1;
                MemMode   <= 2'b00;
                nextState <= FETCH;
            end
            MEM_WRITE_BACK_TO_REG: begin
                $display("[controller] time: %h, current State: MEM_WRITE_BACK_TO_REG", $time);
                MemToReg  <= 1;
                RegWrite  <= 1;
                RegDst    <= 0;
                nextState <= FETCH;
            end
            //这里执行HALT后系统会反复执行这个循环，等价于停机
            HALT: begin
                $display("SYSTEM HALT!");
                nextState <= HALT;
            end
        endcase
    end

endmodule // controller
//////////////////////////////////////////////
/////////////////////////////////////////////
//数据通路：数字系统中，各个子系统通过数据总线连接形成的数据传送路径称为数据通路。 数据通路的设计直接影响到控制器的设计，同时也影响到数字系统的速度指标和成本。一般来说，处理速度快的数字系统，它的独立传送信息的通路较多。但是独立数据传送通路一旦增加，控制器的设计也就复杂了。因此，在满足速度指标的前提下，为使数字系统结构尽量简单，一般小型系统中多采用单一总线结构。
//也就是说，这是连接各部件，类似总线的结构
module datapath (
    input clk, reset,//套路的时钟信号和reset
    // ========= Signal from Controller =========
    //来自控制器的信号
    // All Signal Varibles are capitalized
    //所有变量都编译过了
    input PCWriteCond, PCWrite,
    input [1   : 0] PCSource,
    input IorD,//IorD	作用：读取I/O	低电平：不读I/O	高电平：读取I/O
    input MemToReg,//控制“内存到寄存器”传送
    input IRWrite,//指令寄存器写操作
    input RegWrite, RegDst,//RegWrite：控制寄存器写操作；RegDst：写入寄存器的地址来自哪 低电平表示来自[20:16]，高电平表示来自[15:11]
    input [1   : 0] ALUSrcA, ALUSrcB,//ALUScr：(2位)ALU的运算数所在位置
    // ========= Data from ALUController =========
    //来自ALUController的数据
    input zero,
    input [31  : 0] aluResult,
    // ========= Data from Memory =========
    //来自内存的数据
    input [31  : 0] memData,
    // ========= Data to Controller =========
    //由数据通路（总线）出发，到达控制器的数据
    output [5  : 0] op,
    // ========= Data to ALUController =========
    //由数据通路（总线）出发，到达ALU的数据
    output [5  : 0] funct,
    output [31 : 0] aluParamData1, aluParamData2,
    // ========= Data to Memory =========
    //由数据通路（总线）出发，到达内存的数据
    output [31 : 0] writeMemData,
    output [15 : 0] memAddr
    );


    // ========= Data directly from Registry stored in DFF =========
    //从寄存器读出数据，存储在DFF
    wire [31 : 0] regData1, regData2;
    // ==================
    
    assign writeMemData = regData2;

    // ========= processed PC Signal =========
    //处理程序计数器中的信号
    wire RealPCWrite;
    assign RealPCWrite = (PCWriteCond & zero) | PCWrite;
    // ==================

    // ========= Instructions Address Used by PC =========
    //PC指令地址
    wire [15 : 0] nextInsAddr;
    wire [15 : 0] currentInsAddr;
    // ==================

    // ========= Data produced by alu in dff =========
    //ALU数据
    wire [31 : 0] aluOut;
    // ==================

    // ========= Instruction fetched from Memory stored in instr_reg =========
    //从存储器中读取指令，存在指令寄存器中
    wire [31 : 0] instruction;
    // ==================

    // ========= Data fetched from Memory stored in mem_reg =========
    //从存储器中读取指令，存到mem_reg
    wire [31 : 0] memDataInReg;
    // ==================

    // ========= Registry Write Address =========
    //寄存器写地址
    wire [4  : 0] writeRegAddr;
    // ==================

    // ========= Registry Write Data =========
    //寄存器写数据
    wire [31 : 0] writeRegData;
    // ==================

    // ========= Data directly from Registry =========
    //寄存器数据
    wire [31 : 0] dataFromReg1, dataFromReg2;
    // ==================

    // ========= Instruction slices for output =========
    wire [4  : 0] regAddr1FromInstr, regAddr2FromInstr, regAddr3FromInstr; // regAddr3FromInstr is for R-type Instruction
    wire [15 : 0] immFromInstr;
    wire [25 : 0] jumpAddrFromInstr;
    //assign相当于一根导线，连接两端
    assign op                = instruction[31:26];
    assign funct             = instruction[5:0];
    assign regAddr1FromInstr = instruction[25:21];
    assign regAddr2FromInstr = instruction[20:16];
    assign regAddr3FromInstr = instruction[15:11];
    assign immFromInstr      = instruction[15:0];
    assign jumpAddrFromInstr = instruction[25:0];
    // ==================

    // ========= Sign-Extended Imm=========
    //扩展立即数
    wire [31 : 0] extendedImm;
    // ==================

    // ========= Left 2-bit shifted Values=========
    //左移两位的数据
    wire [31 : 0] extendedImmx4;
    wire [27 : 0] jumpAddrFromInstrx4;
    // ==================
    //实例化所有所需的模块
    PC mips_pc(clk, reset, nextInsAddr, RealPCWrite, currentInsAddr);//程序计数器
    mux2 #(16) mips_instr_addr_src(currentInsAddr, aluOut[15:0], IorD, memAddr);
    mux2 #(5) mips_reg_write_addr_src(regAddr2FromInstr, regAddr3FromInstr, RegDst, writeRegAddr);
    regfile mips_reg(clk, reset, RegWrite, regAddr1FromInstr, regAddr2FromInstr, writeRegAddr, writeRegData, dataFromReg1, dataFromReg2);
    dff mips_instr_reg(clk, reset, IRWrite, memData, instruction);
    dff mips_aluresult_reg(clk, reset, 1'b1, aluResult, aluOut);
    dff mips_mem_reg(clk, reset, 1'b1, memData, memDataInReg);
    mux2 mips_reg_write_data_src(aluOut, memDataInReg, MemToReg, writeRegData);
    dff mips_dff_a(clk, reset, 1'b1, dataFromReg1, regData1);
    dff mips_dff_b(clk, reset, 1'b1, dataFromReg2, regData2);
    mux4 mips_alu_src1({16'b0, currentInsAddr}, regData1, instruction, 32'b1, ALUSrcA, aluParamData1);
    signextend imm_extend(immFromInstr, extendedImm);//符号位扩展
    leftshift2 extended_imm_left_shift2(extendedImm, extendedImmx4);
    mux4 mips_alu_src2(regData2, 32'h4, extendedImm, extendedImmx4, ALUSrcB, aluParamData2);
    leftshift2 #(28) jump_addr_left_shift2({2'b0, jumpAddrFromInstr}, jumpAddrFromInstrx4);
    mux4 #(16) next_instr_src(aluResult[15:0], aluOut[15:0], jumpAddrFromInstrx4[15:0], 16'b0, PCSource, nextInsAddr);

    //打印一些信息
    always @ ( * ) begin
        $display("[datapath] time: %h, RealPCWrite: %b, PCSource: %h, nextInsAddr: %h, currentInsAddr: %h, aluResult %h, aluOut: %h, instruction: %h, memAddr: %h, memData: %h, memDataInReg: %h, writeRegAddr: %h, writeRegData: %h, dataFromReg1: %h, dataFromReg2: %h, aluParamData1: %h, aluParamData2: %h, ALUSrcA: %b, ALUSrcB: %b", $time, RealPCWrite, PCSource, nextInsAddr, currentInsAddr, aluResult, aluOut, instruction, memAddr, memData, memDataInReg, writeRegAddr, writeRegData, dataFromReg1, dataFromReg2, aluParamData1, aluParamData2, ALUSrcA, ALUSrcB);
    end

endmodule // datapath
//////////////////////////////////////////////
/////////////////////////////////////////////
//ALU:算术逻辑单元。输入两个32位的参数，以及控制数（ALUControl），输出zero和结果
//注意：param2是一个16位的数，需要扩展
module alu #(parameter WIDTH = 32) (
    input [WIDTH-1:0] param1, param2,
    input [4:0] ALUControl,
    output zero,
    output reg [WIDTH-1:0] aluResult
    );

    // ALUOP
    //在这里先定义好可能传入的alu控制参数，方便后续代码的编写
    parameter SUM  = 5'b00000;
    parameter OR   = 5'b00001;
    parameter XOR  = 5'b00010;
    parameter AND  = 5'b00011;
    parameter SL   = 5'b00100;
    parameter SRL  = 5'b00101;
    parameter SRA  = 5'b00110;
    parameter LUI  = 5'b00111;
    parameter ORI  = 5'b01000; // param2 is 16-bits, needs zero-extend.
    parameter SUB  = 5'b01001;


    assign zero = (aluResult == 32'b0) ? 1'b1 : 1'b0;//当结果0，zero=1

    always @ ( * ) begin
        case (ALUControl)
        //多种case情况，例如相SUM加，OR或，XOR异或，AND与，SL逻辑左移，SRL逻辑右移，SRA算术右移指令,LUI装入高位立即数，ORI：一个寄存器中的内容与一个立即数相或，SUB减
            SUM: aluResult <= param1 + param2;
            OR: aluResult  <= param1 | param2;
            XOR:aluResult  <= param1 ^ param2;
            AND: aluResult <= param1 & param2;
            SL: aluResult  <= (param2 << param1[10:6]);
            SRL: aluResult <= (param2 >> param1[10:6]);
            SRA: aluResult <= (param2[31] == 0) ? (param2 >> param1[10:6]) : ((~(1<<param1[10:6])<<(32-param1[10:6])) | (param2>>param1[10:6]));
            LUI: aluResult <= (param2 << 16);
            ORI: aluResult <= (32'h0000ffff & param2) | param1;//para2为16位，需要和32'h0000ffff与一下变成32位
            SUB: aluResult <= param1 - param2;
        endcase
    end

endmodule // alu

//寄存器文件
module regfile #(parameter WIDTH = 32, ADDR_WIDTH = 5) (
    input clk, reset,
    input regWrite,
    input [ADDR_WIDTH-1 : 0] readAddr1, readAddr2, writeAddr,//五位的读地址
    input [WIDTH-1      : 0] writeData,//一位写数据
    output [WIDTH-1     : 0] readData1, readData2//一位读数据
    );

    reg [WIDTH-1 : 0] MEM[0:(1<<ADDR_WIDTH)-1];

    assign readData1 = MEM[readAddr1];//读出来的数据=MEM[数据地址1]
    assign readData2 = MEM[readAddr2];//读出来的数据=MEM[数据地址2]

    initial begin
        MEM[0] = 0;
    end

    always @ (posedge clk) begin
        if (regWrite)//当有regwrite信号
            MEM[writeAddr] <= writeData;//把送入的数据写入对应地址
    end

    always @ ( posedge clk ) begin//输出一些信息到屏幕
        $display("[regfile] time: %h, regWrite: %b, writeAddr: %h, writeData: %h, $t0: %h, $t1: %h, $t2: %h, $t3: %h", $time, regWrite, writeAddr, writeData, MEM[8], MEM[9], MEM[10], MEM[11]);
    end
endmodule // regfile

// D触发器：检测信号，若为reset，则输出32位0、否则原值输出
module dff #(parameter WIDTH = 32) (
    input clk, reset, en,
    input      [WIDTH-1 : 0] d,
    output reg [WIDTH-1 : 0] q
    );

    always @ (posedge clk) begin
        if (reset)
            q <= {(WIDTH-1) {1'b0}};//q=32位0
        else if (en)
            q <= d;//否则等于原值
    end

endmodule // dff
//////////////////////////////////////////////
/////////////////////////////////////////////
//偏移两位：输入原值，输出左移两位的值
module leftshift2 #(parameter WIDTH = 32) (
    input [WIDTH-1  : 0] src,
    output [WIDTH-1 : 0] out
    );

    assign out = src << 2;

endmodule // leftshift2
//////////////////////////////////////////////
/////////////////////////////////////////////
//二路选择器：输入两个值，以及要输出的是哪个值，输出选中的那个
module mux2 #(parameter WIDTH = 32) (
    input  [WIDTH-1 : 0] data1,
    input  [WIDTH-1 : 0] data2,
    input                select,
    output [WIDTH-1 : 0] out
    );

    assign out = select ? data2 : data1;

endmodule // mux2
//////////////////////////////////////////////
/////////////////////////////////////////////
//四路选择器：输入四个值，以及要输出的是哪个值，输出选中的那个
module mux4 #(parameter WIDTH = 32) (
    input      [WIDTH-1 : 0] data1,
    input      [WIDTH-1 : 0] data2,
    input      [WIDTH-1 : 0] data3,
    input      [WIDTH-1 : 0] data4,
    input      [      1 : 0] select,
    output reg [WIDTH-1 : 0] out
    );

    always @ ( * ) begin
    //always@(*)里面的敏感变量为*，意思是说敏感变量由综合器根据always里面的输入变量自动添加，也就是所有变量都是敏感列表，不用自己考虑
        case (select)
            2'b00: out <= data1;//00输出data1
            2'b01: out <= data2;//01输出data2
            2'b10: out <= data3;//10输出data3
            2'b11: out <= data4;//11输出data4
        endcase
    end

endmodule // mux4
//////////////////////////////////////////////
/////////////////////////////////////////////
//PC:程序计数器，每周期给出指令地址，然后取指令送入IR
module PC (
    input           clk, reset,//输入时钟和复位信号
    input  [15 : 0] next_ins,//输入16位的指令
    input           PCWrite,
    output [15 : 0] ins//输入指令
    );

    dff #(16) _pc(clk, reset, PCWrite, next_ins, ins);

    // always @ ( * ) begin
    //     $display("[pc] time: %h, PCWrite: %b, next_ins: %h, ins: %h", $time, PCWrite, next_ins, ins);
    // end

endmodule // PC
//////////////////////////////////////////////
/////////////////////////////////////////////
//符号扩展器：在送偏移地址到ALU前需要进行signextend到32位
//详情参见：https://stackoverflow.com/questions/7461963/why-do-we-sign-extend-in-load-word-instruction
module signextend (
    input [15  : 0] srcImm,//进入16位
    output [31 : 0] res//出去32位
    );

    assign res = {{16{srcImm[15]}}, srcImm};

endmodule // signextend



