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
  REGDST_RD,
  REGDST_HI_LO
} reg_dst_t;

// define ALUOp
typedef enum {
  ALU_NOP = 0,
  ALU_ADD,
  ALU_SUB,
  ALU_AND,
  ALU_OR,
  ALU_SLT,
  ALU_SHTL,
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
  ALUSRCB_LO
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
  md_addr_t jump_target;
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
  int opcode;
  // My code
  oprand_t oprand;
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
  int opcode;
  // My code
  oprand_t oprand;
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
  int opcode;
  md_addr_t PC;
  oprand_t oprand;

  int MemtoReg;
  int RegWrite;

  int alu_result;
  int read_data;
};

/* cache part */

typedef struct cache_line {
    unsigned int data[4];       // 16 bytes
    unsigned int tag:24;        // the tag of each cache line
    unsigned int dirty:1;       // whether the cache is dirty
    unsigned int valid:1;       // whether the cache is valid
    unsigned int ref_count;     // how many times the cache line is ref
    struct cache_line *next;    // the pointer to next cache_line
} cache_line_t;

typedef struct cache_set {
    cache_line_t *head;         // the head of the queue
    cache_line_t *tail;         // the tail of the queue
    int n;                      // the number of nodes in the queue
} cache_set_t;

typedef struct cache {
    cache_set_t sets[16];       // array of sets
    unsigned int enable;        // the flag whether cache is enabled
    unsigned int access;        // cache access times
    unsigned int hit;           // cache hit times
    unsigned int miss;          // cache miss times
    unsigned int replace;       // cache line replace times
    unsigned int wb;            // cache line write back times
} cache_t;

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


void init_fd();
void init_de();
void init_em();
void init_mw();


typedef void(*cache_word_func)(cache_line_t *, word_t *, int);
void en_cache_set(cache_set_t *, cache_line_t *);
void de_cache_set(cache_set_t *);
int cache_access(cache_t *, md_addr_t, word_t *, cache_word_func);
int cache_read(cache_t *, md_addr_t, word_t *);
int cache_write(cache_t *, md_addr_t, word_t *);
void cache_word_read(cache_line_t *, word_t *, int);
void cache_word_write(cache_line_t *, word_t *, int);
cache_line_t *malloc_cache_line(md_addr_t);
void cache_write_back(cache_line_t *, int);
void cache_flush(cache_t *);
void add_into_cache_set(cache_set_t *, cache_line_t *, int);


#define MD_FETCH_INSTI(INST, MEM, PC)					\
  { INST.a = MEM_READ_WORD(mem, (PC));					\
    INST.b = MEM_READ_WORD(mem, (PC) + sizeof(word_t)); }

#define SET_OPCODE(OP, INST) ((OP) = ((INST).a & 0xff)) 

#define RSI(INST)		(INST.b >> 24& 0xff)		/* reg source #1 */
#define RTI(INST)		((INST.b >> 16) & 0xff)		/* reg source #2 */
#define RDI(INST)		((INST.b >> 8) & 0xff)		/* reg dest */

#define IMMI(INST)	((int)((/* signed */short)(INST.b & 0xffff)))	/*get immediate value*/
#define TARGI(INST)	(INST.b & 0x3ffffff)		/*jump target*/


#define SET_NUM 4
#define ADDR_TAG(ADDR) (((unsigned int)ADDR)>>8)
#define ADDR_INDEX(ADDR) ((((unsigned int)ADDR)&0xF0)>>4)
#define ADDR_OFFSET(ADDR) (((unsigned int)ADDR)&0xF)