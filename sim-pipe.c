#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* An implementation of 5-stage classic pipeline simulation */

/* don't count instructions flag, enabled by default, disable for inst count */
#undef NO_INSN_COUNT

#include "host.h"
#include "misc.h"
#include "machine.h"
#include "regs.h"
#include "memory.h"
#include "loader.h"
#include "syscall.h"
#include "dlite.h"
#include "sim.h"
#include "sim-pipe.h"

/* simulated registers */
static struct regs_t regs;

/* simulated memory */
static struct mem_t *mem = NULL;

/* register simulator-specific options */
void
sim_reg_options(struct opt_odb_t *odb)
{
  opt_reg_header(odb, 
"sim-pipe: This simulator implements based on sim-fast.\n"
		 );
}

/* check simulator-specific option values */
void
sim_check_options(struct opt_odb_t *odb, int argc, char **argv)
{
  if (dlite_active)
    fatal("sim-pipe does not support DLite debugging");
}

/* register simulator-specific statistics */
void
sim_reg_stats(struct stat_sdb_t *sdb)
{
#ifndef NO_INSN_COUNT
  stat_reg_counter(sdb, "sim_num_insn",
		   "total number of instructions executed",
		   &sim_num_insn, sim_num_insn, NULL);
#endif /* !NO_INSN_COUNT */
  stat_reg_int(sdb, "sim_elapsed_time",
	       "total simulation time in seconds",
	       &sim_elapsed_time, 0, NULL);
#ifndef NO_INSN_COUNT
  stat_reg_formula(sdb, "sim_inst_rate",
		   "simulation speed (in insts/sec)",
		   "sim_num_insn / sim_elapsed_time", NULL);
#endif /* !NO_INSN_COUNT */
  ld_reg_stats(sdb);
  mem_reg_stats(mem, sdb);
}


struct ifid_buf fd;
struct idex_buf de;
struct exmem_buf em;
struct memwb_buf mw;
struct wb_buf wb;

#define DNA			(-1)

/* general register dependence decoders */
#define DGPR(N)			(N)
#define DGPR_D(N)		((N) &~1)

/* floating point register dependence decoders */
#define DFPR_L(N)		(((N)+32)&~1)
#define DFPR_F(N)		(((N)+32)&~1)
#define DFPR_D(N)		(((N)+32)&~1)

/* miscellaneous register dependence decoders */
#define DHI			(0+32+32)
#define DLO			(1+32+32)
#define DFCC		(2+32+32)
#define DTMP		(3+32+32)

/* initialize the simulator */
void
sim_init(void)
{
  /* allocate and initialize register file */
  regs_init(&regs);

  /* allocate and initialize memory space */
  mem = mem_create("mem");
  mem_init(mem);

  /* initialize stage latches*/
 
  /* IF/ID */
  init_fd();

  /* ID/EX */
  init_de();

  /* EX/MEM */
  init_em();

  /* MEM/WB */
  init_mw();

}

/* load program into simulated state */
void
sim_load_prog(char *fname,		/* program to load */
	      int argc, char **argv,	/* program arguments */
	      char **envp)		/* program environment */
{
  /* load program text and data, set up environment, memory, and regs */
  ld_load_prog(fname, argc, argv, envp, &regs, mem, TRUE);
}

/* print simulator-specific configuration information */
void
sim_aux_config(FILE *stream)
{  
	/* nothing currently */
}

/* dump simulator-specific auxiliary simulator statistics */
void
sim_aux_stats(FILE *stream)
{  /* nada */}

/* un-initialize simulator-specific state */
void 
sim_uninit(void)
{ /* nada */ }


/*
 * configure the execution engine
 */

/* next program counter */
#define SET_NPC(EXPR)		(regs.regs_NPC = (EXPR))

/* current program counter */
#define CPC			(regs.regs_PC)

/* general purpose registers */
#define GPR(N)			(regs.regs_R[N])
#define SET_GPR(N,EXPR)		(regs.regs_R[N] = (EXPR))
#define DECLARE_FAULT(EXP) 	{;}
#if defined(TARGET_PISA)

/* floating point registers, L->word, F->single-prec, D->double-prec */
#define FPR_L(N)		(regs.regs_F.l[(N)])
#define SET_FPR_L(N,EXPR)	(regs.regs_F.l[(N)] = (EXPR))
#define FPR_F(N)		(regs.regs_F.f[(N)])
#define SET_FPR_F(N,EXPR)	(regs.regs_F.f[(N)] = (EXPR))
#define FPR_D(N)		(regs.regs_F.d[(N) >> 1])
#define SET_FPR_D(N,EXPR)	(regs.regs_F.d[(N) >> 1] = (EXPR))

/* miscellaneous register accessors */
#define SET_HI(EXPR)		(regs.regs_C.hi = (EXPR))
#define HI			(regs.regs_C.hi)
#define SET_LO(EXPR)		(regs.regs_C.lo = (EXPR))
#define LO			(regs.regs_C.lo)
#define FCC			(regs.regs_C.fcc)
#define SET_FCC(EXPR)		(regs.regs_C.fcc = (EXPR))

#endif

/* precise architected memory state accessor macros */
#define READ_BYTE(SRC, FAULT)						\
  ((FAULT) = md_fault_none, MEM_READ_BYTE(mem, (SRC)))
#define READ_HALF(SRC, FAULT)						\
  ((FAULT) = md_fault_none, MEM_READ_HALF(mem, (SRC)))
#define READ_WORD(SRC, FAULT)						\
  ((FAULT) = md_fault_none, MEM_READ_WORD(mem, (SRC)))
#ifdef HOST_HAS_QWORD
#define READ_QWORD(SRC, FAULT)						\
  ((FAULT) = md_fault_none, MEM_READ_QWORD(mem, (SRC)))
#endif /* HOST_HAS_QWORD */

#define WRITE_BYTE(SRC, DST, FAULT)					\
  ((FAULT) = md_fault_none, MEM_WRITE_BYTE(mem, (DST), (SRC)))
#define WRITE_HALF(SRC, DST, FAULT)					\
  ((FAULT) = md_fault_none, MEM_WRITE_HALF(mem, (DST), (SRC)))
#define WRITE_WORD(SRC, DST, FAULT)					\
  ((FAULT) = md_fault_none, MEM_WRITE_WORD(mem, (DST), (SRC)))
#ifdef HOST_HAS_QWORD
#define WRITE_QWORD(SRC, DST, FAULT)					\
  ((FAULT) = md_fault_none, MEM_WRITE_QWORD(mem, (DST), (SRC)))
#endif /* HOST_HAS_QWORD */

/* system call handler macro */
#define SYSCALL(INST)	sys_syscall(&regs, mem_access, mem, INST, TRUE)

#ifndef NO_INSN_COUNT
#define INC_INSN_CTR()	sim_num_insn++
#else /* !NO_INSN_COUNT */
#define INC_INSN_CTR()	/* nada */
#endif /* NO_INSN_COUNT */

/* start simulation, program loaded, processor precise state initialized */
void
sim_main(void)
{
  fprintf(stderr, "sim: ** starting *pipe* functional simulation **\n");

  /* must have natural byte/word ordering */
  if (sim_swap_bytes || sim_swap_words)
    fatal("sim: *pipe* functional simulation cannot swap bytes or words");

  /* set up initial default next PC */
  regs.regs_NPC = regs.regs_PC + sizeof(md_inst_t);
  /* maintain $r0 semantics */
  regs.regs_R[MD_REG_ZERO] = 0;
 
  fd.PC = regs.regs_PC - sizeof(md_inst_t);

  while (TRUE)
  {
	  /*start your pipeline simulation here*/
    sim_num_insn++;
    stall_check_without_forwarding();
    do_wb();
    do_mem();
    do_ex();
    do_id();
    do_if();
    pipeline_dump();
    if (sim_num_insn >= 320) {
      break;
    }
  }
}

void do_if()
{

  md_inst_t instruction;
  md_addr_t jump_addr;
  int PCSrc;
  PCSrc = !em.alu_zero & em.Branch;
  if (de.Jump) {
    printf("de pc: %x\nj addr: %x\n", de.PC, jump_addr);
    fd.NPC = de.jump_target;
  } else if (PCSrc){
  	fd.NPC = em.branch_address;
  } else{
  	fd.NPC = fd.PC + sizeof(md_inst_t);
  }
  fd.PC = fd.NPC;
  MD_FETCH_INSTI(instruction, mem, fd.PC);
  fd.inst = instruction;

}

void do_id()
{
    de.inst = fd.inst;

    // flush(control harzard)
    if (em.alu_zero & em.Branch) {
      init_de();
    }



    if (de.inst.a == NOP) {
      return;
    }
    MD_SET_OPCODE(de.opcode, de.inst);
    de.PC = fd.PC;
    md_inst_t inst = de.inst;
#define DEFINST(OP,MSK,NAME,OPFORM,RES,FLAGS,O1,O2,I1,I2,I3)\
  if (OP==de.opcode){\
    de.instFlags = FLAGS;\
    de.oprand.out1 = O1;\
    de.oprand.out2 = O2;\
    de.oprand.in1 = I1;\
    de.oprand.in2 = I2;\
    de.oprand.in3 = I3;\
    goto READ_OPRAND_VALUE;\
  }
#define DEFLINK(OP,MSK,NAME,MASK,SHIFT)
#define CONNECT(OP)
#include "machine.def"

READ_OPRAND_VALUE:
  switch(de.opcode){
    case NOP:
      de.Jump = 0;

      de.MemWrite = 0;
      de.MemRead = 0;
      de.Branch = 0;

      de.MemtoReg = 0;
      de.RegWrite = 1;
      break;
    case ADD:
    case ADDU:
      de.RegDst = REGDST_RD;
      de.ALUSrcA = ALUSRCA_RS;
      de.ALUSrcB = ALUSRCB_RT;
      de.ALUOp = ALU_ADD;
      de.Jump = 0;

      de.MemWrite = 0;
      de.MemRead = 0;
      de.Branch = 0;

      de.MemtoReg = 0;
      de.RegWrite = 1;

      break;
    case SUBU:
      de.RegDst = REGDST_RD;
      de.ALUSrcA = ALUSRCA_RS;
      de.ALUSrcB = ALUSRCB_RT;
      de.ALUOp = ALU_SUB;
      de.Jump = 0;

      de.MemWrite = 0;
      de.MemRead = 0;
      de.Branch = 0;

      de.MemtoReg = 0;
      de.RegWrite = 1;
      
      break;
    case ADDIU:
      de.RegDst = REGDST_RT;
      de.ALUSrcA = ALUSRCA_RS;
      de.ALUSrcB = ALUSRCB_EX_IMM;
      de.ALUOp = ALU_ADD;
      de.Jump = 0;

      de.MemWrite = 0;
      de.MemRead = 0;
      de.Branch = 0;

      de.MemtoReg = 0;
      de.RegWrite = 1;

      break;
    case ANDI:
      de.RegDst = REGDST_RD;
      de.ALUSrcA = ALUSRCA_RS;
      de.ALUSrcB = ALUSRCB_EX_IMM;
      de.ALUOp = ALU_AND;
      de.Jump = 0;

      de.MemWrite = 0;
      de.MemRead = 0;
      de.Branch = 0;

      de.MemtoReg = 0;
      de.RegWrite = 1;

      break;
    case BNE:
      de.ALUSrcA = ALUSRCA_RS;
      de.ALUSrcB = ALUSRCB_RT;
      de.ALUOp = ALU_SUB;
      de.Jump = 0;

      de.MemWrite = 0;
      de.MemRead = 0;
      de.Branch = 1;

      de.MemtoReg = 0;
      de.RegWrite = 0;

      break;
    case JUMP:
      de.ALUOp = ALU_NOP;
      de.Jump = 1;

      de.MemWrite = 0;
      de.MemRead = 0;
      de.Branch = 0;

      de.MemtoReg = 0;
      de.RegWrite = 0;

      break;
    case LUI:
      de.RegDst = REGDST_RT;
      de.ALUSrcA = ALUSRCA_ZERO;
      de.ALUSrcB = ALUSRCB_UP_IMM;
      de.ALUOp = ALU_ADD;
      de.Jump = 0;

      de.MemWrite = 0;
      de.MemRead = 0;
      de.Branch = 0;

      de.MemtoReg = 0;
      de.RegWrite = 1;
      break;
    case LW:
      de.RegDst = REGDST_RT;
      de.ALUSrcA = ALUSRCA_UP_RS;
      de.ALUSrcB = ALUSRCB_EX_IMM;
      de.ALUOp = ALU_ADD;
      de.Jump = 0;

      de.MemWrite = 0;
      de.MemRead = 1;
      de.Branch = 0;

      de.MemtoReg = 1;
      de.RegWrite = 1;
      break;
    
    case SLL:
      de.RegDst = REGDST_RD;
      de.ALUSrcA = ALUSRCA_SHAMT;
      de.ALUSrcB = ALUSRCB_RT;
      de.ALUOp = ALU_SHTL;
      de.Jump = 0;

      de.MemWrite = 0;
      de.MemRead = 0;
      de.Branch = 0;

      de.MemtoReg = 0;
      de.RegWrite = 1;
      break;
    
    case SW:
      de.ALUSrcA = ALUSRCA_UP_RS;
      de.ALUSrcB = ALUSRCB_EX_IMM;
      de.ALUOp = ALU_ADD;
      de.Jump = 0;

      de.MemWrite = 1;
      de.MemRead = 0;
      de.Branch = 0;

      de.MemtoReg = 0;
      de.RegWrite = 0;
      break;
    case SLTI:
      // de.oprand.cons.imm = IMM;break;
      de.RegDst = REGDST_RT;
      de.ALUSrcA = ALUSRCA_RS;
      de.ALUSrcB = ALUSRCB_EX_IMM;
      de.ALUOp = ALU_SLT;
      de.Jump = 0;

      de.MemWrite = 0;
      de.MemRead = 0;
      de.Branch = 0;

      de.MemtoReg = 0;
      de.RegWrite = 1;
      break;

  }

  de.register_rs = RS;
  de.register_rt = RT;
  de.register_rd = RD;
  de.read_data_1 = GPR(de.register_rs);
  de.read_data_2 = GPR(de.register_rt);
  de.ext_imm = IMM;
  de.shamt = SHAMT;

  de.jump_target = (de.PC & 0xf0000000) + (TARG << 2);

  if (mw.RegWrite) {
    if (mw.MemtoReg) {
      SET_GPR(mw.write_register, mw.alu_result);
    } else {
      SET_GPR(mw.write_register, mw.read_data);
    }
  }
  
}

void do_ex()
{
  em.inst = de.inst;
  // my code
  em.oprand = de.oprand;
  em.PC = de.PC;

  em.Branch = de.Branch;
  em.MemRead = de.MemRead;
  em.MemWrite = de.MemWrite;

  em.MemtoReg = de.MemtoReg;
  em.RegWrite = de.RegWrite;

  em.branch_address = de.PC + (de.ext_imm << 2);

  int srcA;
  int srcB;

  switch (de.ALUSrcA) {
    case ALUSRCA_RS:
      srcA = de.read_data_1;
      break;
    case ALUSRCA_SHAMT:
      srcA = de.shamt;
      break;
    case ALUSRCA_UP_RS:
      srcA = de.read_data_1 << 16;
      break;
    case ALUSRCA_ZERO:
      srcA = 0;
      break;
    default:
      break;
  }

  switch (de.ALUSrcB) {
    case ALUSRCB_RT:
      srcB = de.read_data_2;
      break;
    case ALUSRCB_EX_IMM:
      srcB = de.ext_imm;
      break;
    case ALUSRCB_UP_IMM:
      srcB = de.ext_imm << 16;
      break;
    default:
      break;
  }

  switch(de.ALUOp) {
    case ALU_ADD:
      em.alu_result = srcA + srcB; 
      break;
    case ALU_SUB:
      em.alu_result = srcA - srcB;
      break;
    case ALU_AND:
      em.alu_result = srcA & srcB;
      break;
    case ALU_OR:
      em.alu_result = srcA | srcB;
      break;
    case ALU_SLT:
      em.alu_result = srcA < srcB;
      break;
    case ALU_SHTL:
      em.alu_result = srcB << srcA;
      break;
  }

  em.alu_zero = em.alu_result == 0;

  em.write_data = de.read_data_2;

  switch (de.RegDst) {
    case REGDST_RT:
      em.write_register = de.register_rt;
      break;
    case REGDST_RD:
      em.write_register = de.register_rd;
      break;
  }


}

void do_mem()
{
  enum md_fault_type _fault;
  mw.inst = em.inst;
  // my code
  mw.oprand = em.oprand;
  mw.PC = em.PC;

  mw.RegWrite = em.RegWrite;
  mw.MemtoReg = em.MemtoReg;

  if (em.MemRead) {
    mw.read_data = READ_WORD(em.alu_result, _fault);
  } else if (em.MemWrite) {
    WRITE_WORD(em.write_data, em.alu_result, _fault);
  }
  
}                                                                                        

void do_wb()
{
	// my code
  wb.inst = mw.inst;
  wb.PC = mw.PC;

  
  if (wb.inst.a == SYSCALL) {
    printf("Loop terminated. Result=%d\n",GPR(6));
    SET_GPR(2,SS_SYS_exit);
    SYSCALL(wb.inst);
  } 
}

void pipeline_dump() {
  enum md_fault_type _fault;
	printf("[Cycle %5d]---------------------------------\n",(int)sim_num_insn);
	printf("[IF]  ");md_print_insn(fd.inst, fd.PC, stdout);printf("\n");
	printf("[ID]  ");md_print_insn(de.inst, de.PC, stdout);printf("\n");
	printf("[EX]  ");md_print_insn(em.inst, em.PC, stdout);printf("\n");
	printf("[MEM] ");md_print_insn(mw.inst, mw.PC, stdout);printf("\n");
	printf("[WB]  ");md_print_insn(wb.inst, wb.PC, stdout);printf("\n");
	printf("[REGS]r0=%d r1=%d r2=%d r3=%d r4=%d r5=%d r6=%d mem = %d\n", 
			GPR(0),GPR(1),GPR(2),GPR(3),GPR(4),GPR(5),GPR(6),(int)READ_WORD(GPR(30)+16, _fault));
	printf("----------------------------------------------\n");
}


void init_fd() {
  fd.inst.a = NOP;
}


void init_de() {
  
  de.latched = 0;
  de.inst.a = NOP;

  de.RegDst = REGDST_RT;
  de.ALUSrcA = ALUSRCA_RS;
  de.ALUSrcB = ALUSRCB_RT;
  de.ALUOp = ALU_NOP;

  de.Jump = 0;

  de.MemWrite = 0;
  de.MemRead = 0;
  de.Branch = 0;

  de.MemtoReg = 0;
  de.RegWrite = 0;
}

void init_em() {
  em.inst.a = NOP;

  em.MemWrite = 0;
  em.MemRead = 0;
  em.Branch = 0;

  em.MemtoReg = 0;
  em.RegWrite = 0;
}

void init_mw() {
  mw.inst.a = NOP;
  mw.PC = 0;

  mw.MemtoReg = 0;
  mw.RegWrite = 0;
}

void stall_check_without_forwarding() {
  if((em.RegWrite == 1 && (em.inst.a != NOP && em.oprand.out1 != DNA 
		&& (em.oprand.out1 == de.oprand.in1 || em.oprand.out1 == de.oprand.in2)))
		||(mw.RegWrite == 1 && (mw.inst.a != NOP && mw.oprand.out1 != DNA
		&& (mw.oprand.out1 == de.oprand.in1 || mw.oprand.out1 == de.oprand.in2))))
	{
		fd.inst = de.inst;
		fd.PC = de.PC;
    init_de();
	}
}


