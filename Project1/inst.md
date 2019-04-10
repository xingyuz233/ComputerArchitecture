### Inst

|          | RegDst(EX) | RegWrite(WB) | ALUSrcA(EX) | ALUSrcB(EX) | ALUOp(EX) | MemWrite(MEM) | MemRead(MEM) | MemtoReg(WB) | Branch(MEM) | Jump(EX) |      |      |
| -------- | ---------- | ------------ | ----------- | ----------- | --------- | ------------- | ------------ | ------------ | ----------- | -------- | ---- | ---- |
| add      | 1(RD)      | 1            | 00(RS)      | 00(RT)      | ALU_ADD   | 0             | 0            | 0            | 0           | 0        |      |      |
| addu     | 1(RD)      | 1            | 00(RS)      | 00(RT)      | ALU_ADD   | 0             | 0            | 0            | 0           | 0        |      |      |
| subu     | 1(RD)      | 1            | 00(RS)      | 00(RT)      | ALU_SUB   | 0             | 0            | 0            | 0           | 0        |      |      |
| addiu    | 0(RT)      | 1            | 00(RS)      | 01(EX_IMM)  | ALU_ADD   | 0             | 0            | 0            | 0           | 0        |      |      |
| andi     | 1(RD)      | 1            | 00(RS)      | 01(EX_IMM)  | ALU_AND   | 0             | 0            | 0            | 0           | 0        |      |      |
| bne      | x          | 0            | 00(RS)      | 00(RT)      | ALU_SUB   | 0             | 0            | 0            | 1           | 0        |      |      |
| jump     | x          | 0            | x           | x           | ALU_NOP   | 0             | 0            | 0            | 0           | 1        |      |      |
| lui      | 0(RT)      | 1            | 11(0)       | 10(UP_IMM)  | ALU_ADD   | 0             | 0            | 0            | 0           | 0        |      |      |
| lw       | 0(RT)      | 1            | 10(UP_RS)   | 01(EX_IMM)  | ALU_ADD   | 0             | 1            | 1            | 0           | 0        |      |      |
| sll      | 1(RD)      | 1            | 01(SHAMT)   | 00(RT)      | ALU_SHTL  | 0             | 0            | 0            | 0           | 0        |      |      |
| sw       | x          | 0            | 10(UP_RS)   | 01(EX_IMM)  | ALU_ADD   | 1             | 0            | 0            | 0           | 0        |      |      |
| slti     | 0(RT)      | 1            | 00(RS)      | 01(EX_IMM)  | ALU_SLT   | 0             | 0            | 0            | 0           | 0        |      |      |
| sys call | x          | 0            | x           | x           | ALU_NOP   | 0             | 0            | 0            | 0           | 0        |      |      |
| nop      | x          | 0            | x           | x           | ALU_NOP   | 0             | 0            | 0            | 0           | 0        |      |      |



### handout

For detailed information, [see MIPS instruction](./MIPS_Instruction _Set.pdf)

