#include "machine.h"

/* define values related to operands, all possible combinations are included */
typedef struct{
  int in1;			/* input 1 register number */
  int in2;			/* input 2 register number */
  int in3;			/* input 3 register number */
  int out1;			/* output 1 register number */
  int out2;			/* output 2 register number */
  //......
}oprand_t;

// define RegDst
typedef enum {
  REGDST_RT = 0,
  REGDST_RD
} reg_dst_t;

// define ALUOp
typedef enum {
  ALU_NOP = 0,
  ALU_ADD,
  ALU_SUB,
  ALU_AND,
  ALU_OR,
  ALU_SLT,
  ALU_SHTL
} alu_op_t;

// define ALUSrcA
typedef enum {
  ALUSRCA_RS = 0,
  ALUSRCA_SHAMT,
  ALUSRCA_UP_RS,
  ALUSRCA_ZERO
} alu_src_A_t;

// define ALUSrcB
typedef enum {
  ALUSRCB_RT = 0,
  ALUSRCB_EX_IMM,
  ALUSRCB_UP_IMM,
} alu_src_B_t;

/*define buffer between fetch and decode stage*/
struct ifid_buf {
  md_inst_t inst;	    /* instruction that has been fetched */
  md_addr_t PC;	        /* pc value of current instruction */
  md_addr_t NPC;		/* the next instruction to fetch */
};


/*define buffer between decode and execute stage*/
struct idex_buf {
  md_inst_t inst;		/* instruction in ID stage */ 
  int opcode;			/* operation number */
  oprand_t oprand;		/* operand */
  // My code
  md_addr_t PC;
  int instFlags;
  int latched;

  int read_data_1;
  int read_data_2;
  int register_rs;
  int register_rt;
  int register_rd; 
  int ext_imm;
  int shamt;

  // control
  reg_dst_t RegDst;
  alu_op_t ALUOp;
  alu_src_A_t ALUSrcA;
  alu_src_B_t ALUSrcB;
  int Jump;

  int MemtoReg;
  int RegWrite;
  int MemWrite;
  
  int MemRead;
  int Branch;

  
};

/*define buffer between execute and memory stage*/
struct exmem_buf{
  md_inst_t inst;		/* instruction in EX stage */
  // My code
  md_addr_t PC;
  
  md_addr_t branch_address;
  int alu_zero;
  int alu_result;
  int write_data;
  int write_register;


  // control
  int MemtoReg;
  int RegWrite;
  int MemWrite;
  
  int MemRead;
  int Branch;
};

/*define buffer between memory and writeback stage*/
struct memwb_buf{
  md_inst_t inst;		/* instruction in MEM stage */
  // My code
  md_addr_t PC;

  int read_data;
  int alu_result;
  int write_register;
  // control
  int MemtoReg;
  int RegWrite;

};
  
/*define buffer for wb just for dump*/
struct wb_buf {
  md_inst_t inst;
  md_addr_t PC;
};

/*do fetch stage*/
void do_if();

/*do decode stage*/
void do_id();

/*do execute stage*/
void do_ex();

/*do memory stage*/
void do_mem();

/*do write_back to register*/
void do_wb();

/*dump the pipeline*/
void pipeline_dump();


#define MD_FETCH_INSTI(INST, MEM, PC)					\
  { INST.a = MEM_READ_WORD(mem, (PC));					\
    INST.b = MEM_READ_WORD(mem, (PC) + sizeof(word_t)); }

#define SET_OPCODE(OP, INST) ((OP) = ((INST).a & 0xff)) 

#define RSI(INST)		(INST.b >> 24& 0xff)		/* reg source #1 */
#define RTI(INST)		((INST.b >> 16) & 0xff)		/* reg source #2 */
#define RDI(INST)		((INST.b >> 8) & 0xff)		/* reg dest */

#define IMMI(INST)	((int)((/* signed */short)(INST.b & 0xffff)))	/*get immediate value*/
#define TARGI(INST)	(INST.b & 0x3ffffff)		/*jump target*/
