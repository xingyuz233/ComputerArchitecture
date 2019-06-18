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
cache_t cache;
counter_t sim_num_clk;

void stall_check_with_forwarding();


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

  /* CACHE */
  init_cache();

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

#define INC_CYCLE(NUM) (sim_num_clk += NUM)

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
    stall_check_with_forwarding();
    do_wb();
    do_mem();
    do_ex();
    do_id();
    do_if();    
  }
}

void do_if()
{

  md_inst_t instruction;
  md_addr_t jump_addr;
  int PCSrc;
    if (em.opcode == BNE) {
    PCSrc = !em.alu_zero & em.Branch;
  } else if (em.opcode == BEQ) {
    PCSrc = em.alu_zero & em.Branch;
  } else {
    PCSrc = 0;
  }
  if (PCSrc) {
  	fd.NPC = em.branch_address;
  } else if (de.Jump) {
    fd.NPC = de.jump_target;
  } else{
  	fd.NPC = fd.PC + sizeof(md_inst_t);
  }
  fd.PC = fd.NPC;
  int cycles;
  if (cache.enable) {
    cycles = cache_read(&cache, fd.PC, &(instruction.a)) + cache_read(&cache, fd.PC + 4, &(instruction.b));
  } else {
    MD_FETCH_INSTI(instruction, mem, fd.PC);
    cycles = 20;
  } 
  INC_CYCLE(cycles);
  // MD_FETCH_INSTI(instruction, mem, fd.PC);
  fd.inst = instruction;

}

void do_id()
{
    de.inst = fd.inst;

    // flush when branch detection false (control harzard)
    if ((em.opcode == BNE && !em.alu_zero && em.Branch)
      ||(em.opcode == BEQ && em.alu_zero && em.Branch)) {
      init_de();
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
      de.RegWrite = 0;
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
    case ADDI:
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
    case BEQ:
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
      de.ALUSrcA = ALUSRCA_RS;
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
      de.ALUSrcA = ALUSRCA_RS;
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

    case MULTU:
      de.ALUOp = ALU_NOP;
      de.Jump = 0;

      de.MemWrite = 0;
      de.MemRead = 0;
      de.Branch = 0;

      de.MemtoReg = 0;
      de.RegWrite = 0;
      break;

    case MFLO:
      de.RegDst = REGDST_RD;
      de.ALUSrcA = ALUSRCA_ZERO;
      de.ALUSrcB = ALUSRCB_LO;
      de.ALUOp = ALU_ADD;
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
  

  
}

void do_ex()
{
  em.inst = de.inst;
  em.opcode = de.opcode;
  // my code
  em.oprand = de.oprand;
  em.PC = de.PC;

  em.Branch = de.Branch;
  em.MemRead = de.MemRead;
  em.MemWrite = de.MemWrite;

  em.MemtoReg = de.MemtoReg;
  em.RegWrite = de.RegWrite;

  em.branch_address = de.PC + sizeof(md_inst_t) + (de.ext_imm << 2);

 

  int srcA;
  int srcB;

  // forward unit 
  if(wb.inst.a != NOP && 
			(de.oprand.in1 == wb.oprand.out1 || de.oprand.in2 == wb.oprand.out1)){
		if(wb.oprand.out1 == de.register_rs){
			de.read_data_1 = wb.MemtoReg == 1?wb.read_data:wb.alu_result;
		}else if(wb.oprand.out1 == de.register_rt){
		  de.read_data_2 = wb.MemtoReg == 1?wb.read_data:wb.alu_result;
		}
	}
	if(mw.inst.a != NOP && 
			(de.oprand.in1 == mw.oprand.out1 || de.oprand.in2 == mw.oprand.out1)){
		if(mw.oprand.out1 == de.register_rs){
			de.read_data_1 = mw.alu_result;
		}else if(mw.oprand.out1 == de.register_rt){
			de.read_data_2 = mw.alu_result;
		}
	}

  if (de.opcode == MULTU) {
    int i;
    SET_HI(0);
    SET_LO(0);
    if (de.read_data_2 & 020000000000){
      SET_LO(de.read_data_1);
    }
    for (i = 0; i < 31; i++) {		
      SET_HI(HI << 1);
      SET_HI(HI + extractl(LO, 31, 1));
      SET_LO(LO << 1);
      if ((extractl(de.read_data_2, 30 - i, 1)) == 1){
        if (((unsigned)037777777777 - (unsigned)LO) < (unsigned)de.read_data_1){
          SET_HI(HI + 1);
        }
        SET_LO(LO + de.read_data_1);
      }
    }
  }

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
    case ALUSRCB_LO:
      srcB = LO;
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
  mw.opcode = em.opcode;
  // my code
  mw.oprand = em.oprand;
  mw.PC = em.PC;

  mw.RegWrite = em.RegWrite;
  mw.MemtoReg = em.MemtoReg;

  int cycles = 0;
  if (em.MemRead) {
    if (cache.enable) {
      cycles = cache_read(&cache, em.alu_result, (word_t *)&mw.read_data);
    } else {
      mw.read_data = READ_WORD(em.alu_result, _fault);
      if (_fault != md_fault_none) {
        DECLARE_FAULT(_fault);
      }
      cycles = 10;
    } 
  } else if (em.MemWrite) {
    if (cache.enable) {
      cycles = cache_write(&cache, em.alu_result, (word_t *)&em.write_data);
    } else {
      WRITE_WORD(em.write_data, em.alu_result, _fault);
      if (_fault != md_fault_none) {
        DECLARE_FAULT(_fault);
      }
      cycles = 10;
    }
  }
  INC_CYCLE(cycles);

  // if (em.MemRead) {
  //   mw.read_data = READ_WORD(em.alu_result, _fault);
  // } else if (em.MemWrite) {
  //   WRITE_WORD(em.write_data, em.alu_result, _fault);
  // }
  mw.write_register = em.write_register;
  mw.alu_result = em.alu_result;
  
}                                                                                        

void do_wb()
{
	// my code
  wb.inst = mw.inst;
  wb.opcode = mw.opcode;
  wb.PC = mw.PC;
  
  wb.oprand = mw.oprand;
  wb.alu_result = mw.alu_result;
  wb.read_data = mw.read_data;
  wb.RegWrite = mw.RegWrite;
  wb.MemtoReg = mw.MemtoReg;
  
  if (wb.inst.a == SYSCALL) {
    cache_flush(&cache);
    cache_dump();
    SET_GPR(2,SS_SYS_exit);
    SYSCALL(wb.inst);
  } else if (mw.RegWrite) {
    if (mw.MemtoReg) {
      SET_GPR(mw.write_register, mw.read_data);
    } else {
      SET_GPR(mw.write_register, mw.alu_result);
    }
  }
}

void pipeline_dump() {

 enum md_fault_type _fault;
    fprintf(stdout, "[Cycle %4lld]---------------------------------------", sim_num_insn);
    fprintf(stdout, "\n[IF]\t");
    md_print_insn(fd.inst, fd.PC, stdout);
    fprintf(stdout, "\n[ID]\t");
    md_print_insn(de.inst, de.PC, stdout);
    fprintf(stdout, "\n[EX]\t");
    md_print_insn(em.inst, em.PC, stdout);
    fprintf(stdout, "\n[MEM]\t");
    md_print_insn(mw.inst, mw.PC, stdout);
    fprintf(stdout, "\n[WB]\t");
    md_print_insn(wb.inst, wb.PC, stdout);
    fprintf(stdout, "\n[REGS]\n");
    fprintf(stdout, "r[0]=%x r[1]=%x r[2]=%x r[3]=%x r[4]=%x r[5]=%x r[6]=%x r[7]=%x\n",
      GPR(0), GPR(1), GPR(2), GPR(3), GPR(4), GPR(5), GPR(6), GPR(7), _fault);
    fprintf(stdout, "r[8]=%x r[9]=%x r[10]=%x r[11]=%x r[12]=%x r[13]=%x r[14]=%x r[15]=%x\n",
      GPR(8), GPR(9), GPR(10), GPR(11), GPR(12), GPR(13), GPR(14), GPR(15), _fault);
    fprintf(stdout, "r[16]=%x r[17]=%x r[18]=%x r[19]=%x r[20]=%x r[21]=%x r[22]=%x r[23]=%x\n",
      GPR(16), GPR(17), GPR(18), GPR(19), GPR(20), GPR(21), GPR(22), GPR(23), _fault);
    fprintf(stdout, "r[24]=%x r[25]=%x r[26]=%x r[27]=%x r[28]=%x r[29]=%x r[30]=%x r[31]=%x\n",
      GPR(24), GPR(25), GPR(26), GPR(27), GPR(28), GPR(29), GPR(30), GPR(31), _fault);
    fprintf(stdout, "---------------------------------------------------\n");
    if (_fault != md_fault_none) {
      DECLARE_FAULT(_fault);
    }
}

void cache_dump() {
  enum md_fault_type _fault;
  printf("Clock Cycles: %d\n", sim_num_clk);
  printf("Memory Accesses: %d\n", cache.access);
  printf("Memory Hits: %d\n", cache.hit);
  printf("Memory Misses: %d\n", cache.miss);
  printf("Line Replacements: %d\n", cache.replace);
  printf("Line Write-backs: %d\n", cache.wb);
  if(_fault != md_fault_none){
    DECLARE_FAULT(_fault);
  }
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

void stall_check_with_forwarding() {

  // forward will be implemented in do_ex function
  // control harzard will be solved in do_id function 

  // data harzard only for Load instruction(i.e. MemRead = 1)
  if((em.MemRead == 1 && em.inst.a != NOP && 
  (em.oprand.out1 == de.oprand.in1 || em.oprand.out1 == de.oprand.in2)))
	{
		fd.inst = de.inst;
		fd.PC = de.PC;
    init_de();
	}
}

init_cache() {
  cache.enable = 1;
  cache.access = 0;
  cache.hit = 0;
  cache.miss = 0;
  cache.replace = 0;
  cache.wb = 0;
}

void en_cache_set(cache_set_t *set, cache_line_t *line) {
  if (set->tail != NULL) {
    set->tail->next = line;
  }
  set->tail = line;
  if (set->head == NULL) {
    set->head = line;
  }
  set->n += 1;
}
void de_cache_set(cache_set_t *set) {
  if (set->n == 0) {
    return;
  }
  cache_line_t *head = set->head;
  set->head = head->next;
  set->n -= 1;
  if (set->n == 0) {
      set->tail = NULL;
    }
    free(head);
}
int cache_access(cache_t *cache, md_addr_t addr, word_t *word, cache_word_func func) {
  unsigned int tag = ADDR_TAG(addr);
  unsigned int index = ADDR_INDEX(addr);
  unsigned int offset = ADDR_OFFSET(addr);

  cache_set_t *set = &cache->sets[index];
  cache_line_t *line = NULL;
  md_addr_t align_addr = addr & (~0xF);
  int cycles = 1;
  int miss = 1;
  cache->access++;
  for(line = set->head; line != NULL; line = line->next) {
    if(line->valid && tag == line->tag) {
      miss = 0;
      line->ref_count++;
      cache->hit++;
      func(line, word, offset);
    }
  }
  // miss here
  if(miss) {
    cycles = 10;
    cache->miss++;
    line = malloc_cache_line(align_addr);
    add_into_cache_set(set, line, index);
    func(line, word, offset);
  }
  return cycles;
}

int cache_read(cache_t *cache, md_addr_t addr, word_t *word) {
  return cache_access(cache, addr, word, cache_word_read);
}

int cache_write(cache_t *cache, md_addr_t addr, word_t *word) {
  return cache_access(cache, addr, word, cache_word_write);
}
void cache_word_read(cache_line_t *line, word_t *dst, int offset) {
  memcpy(dst, (void *)(&line->data)+offset, sizeof(word_t));   
}

void cache_word_write(cache_line_t *line, word_t *src, int offset) {
  memcpy((void *)(&line->data)+offset, src, sizeof(word_t));
  line->dirty = 1;
}

void cache_flush(cache_t* cache) {
  cache_set_t *set;
  cache_line_t *line;
  int i;
  for(i = 0; i < 16; i++) {
    set = &cache->sets[i];
    for(line = set->head; line != NULL; line = line->next) {
      if(line->dirty) {
        cache_write_back(line, i);
      }
    }
  }
}

cache_line_t *malloc_cache_line(md_addr_t addr) {
  cache_line_t *line = malloc(sizeof(cache_line_t));
  int i;
  enum md_fault_type _fault;    
  for(i = 0; i < 4; i++) {
    line->data[i] = READ_WORD(addr+(i*4), _fault);
  }
  if(_fault != md_fault_none) {
    panic("Memory Write Error!");
  }
  line->ref_count = 0;
  line->tag = ADDR_TAG(addr);
  line->dirty = 0;
  line->valid = 1;
  line->next = NULL;
  return line;
}

void cache_write_back(cache_line_t *line, int index) {
  md_addr_t addr = (line->tag<<8) | (index<<4);
  int i;
  enum md_fault_type _fault;
  for(i = 0; i < 4; i++) {
    WRITE_WORD(line->data[i], addr+(i*4), _fault);
  }
  line->dirty = 0;    
  if(_fault != md_fault_none) {
    panic("MEMORY ERROR!");
  }
}

void add_into_cache_set(cache_set_t *set, cache_line_t *line, int index) {
  if(set->n >= SET_NUM) {
    if(set->head->dirty) {
      cache.wb++;
      cache_write_back(set->head, index);
    }
    cache.replace++;
    de_cache_set(set);
  }
  en_cache_set(set, line);
}