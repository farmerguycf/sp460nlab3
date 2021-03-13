/*
    Name 1: guy farmer
    UTEID 1: gcf375
*/

/***************************************************************/
/*                                                             */
/*   LC-3b Simulator                                           */
/*                                                             */
/*   EE 460N                                                   */
/*   The University of Texas at Austin                         */
/*                                                             */
/***************************************************************/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/***************************************************************/
/*                                                             */
/* Files:  ucode        Microprogram file                      */
/*         isaprogram   LC-3b machine language program file    */
/*                                                             */
/***************************************************************/

/***************************************************************/
/* These are the functions you'll have to write.               */
/***************************************************************/

void eval_micro_sequencer();
void cycle_memory();
void eval_bus_drivers();
void drive_bus();
void latch_datapath_values();
int sign_extend(int num, int bit);

/***************************************************************/
/* A couple of useful definitions.                             */
/***************************************************************/
#define FALSE 0
#define TRUE 1

/***************************************************************/
/* Use this to avoid overflowing 16 bits on the bus.           */
/***************************************************************/
#define Low16bits(x) ((x)&0xFFFF)

/***************************************************************/
/* Definition of the control store layout.                     */
/***************************************************************/
#define CONTROL_STORE_ROWS 64
#define INITIAL_STATE_NUMBER 18

/***************************************************************/
/* Definition of bit order in control store word.              */
/***************************************************************/
enum CS_BITS
{
    IRD,
    COND1,
    COND0,
    J5,
    J4,
    J3,
    J2,
    J1,
    J0,
    LD_MAR,
    LD_MDR,
    LD_IR,
    LD_BEN,
    LD_REG,
    LD_CC,
    LD_PC,
    GATE_PC,
    GATE_MDR,
    GATE_ALU,
    GATE_MARMUX,
    GATE_SHF,
    PCMUX1,
    PCMUX0,
    DRMUX,
    SR1MUX,
    ADDR1MUX,
    ADDR2MUX1,
    ADDR2MUX0,
    MARMUX,
    ALUK1,
    ALUK0,
    MIO_EN,
    R_W,
    DATA_SIZE,
    LSHF1,
    CONTROL_STORE_BITS
} CS_BITS;

/***************************************************************/
/* Functions to get at the control bits.                       */
/***************************************************************/
int GetIRD(int *x) { return (x[IRD]); }
int GetCOND(int *x) { return ((x[COND1] << 1) + x[COND0]); }
int GetJ(int *x) { return ((x[J5] << 5) + (x[J4] << 4) +
                           (x[J3] << 3) + (x[J2] << 2) +
                           (x[J1] << 1) + x[J0]); }
int GetLD_MAR(int *x) { return (x[LD_MAR]); }
int GetLD_MDR(int *x) { return (x[LD_MDR]); }
int GetLD_IR(int *x) { return (x[LD_IR]); }
int GetLD_BEN(int *x) { return (x[LD_BEN]); }
int GetLD_REG(int *x) { return (x[LD_REG]); }
int GetLD_CC(int *x) { return (x[LD_CC]); }
int GetLD_PC(int *x) { return (x[LD_PC]); }
int GetGATE_PC(int *x) { return (x[GATE_PC]); }
int GetGATE_MDR(int *x) { return (x[GATE_MDR]); }
int GetGATE_ALU(int *x) { return (x[GATE_ALU]); }
int GetGATE_MARMUX(int *x) { return (x[GATE_MARMUX]); }
int GetGATE_SHF(int *x) { return (x[GATE_SHF]); }
int GetPCMUX(int *x) { return ((x[PCMUX1] << 1) + x[PCMUX0]); }
int GetDRMUX(int *x) { return (x[DRMUX]); }
int GetSR1MUX(int *x) { return (x[SR1MUX]); }
int GetADDR1MUX(int *x) { return (x[ADDR1MUX]); }
int GetADDR2MUX(int *x) { return ((x[ADDR2MUX1] << 1) + x[ADDR2MUX0]); }
int GetMARMUX(int *x) { return (x[MARMUX]); }
int GetALUK(int *x) { return ((x[ALUK1] << 1) + x[ALUK0]); }
int GetMIO_EN(int *x) { return (x[MIO_EN]); }
int GetR_W(int *x) { return (x[R_W]); }
int GetDATA_SIZE(int *x) { return (x[DATA_SIZE]); }
int GetLSHF1(int *x) { return (x[LSHF1]); }

/***************************************************************/
/* The control store rom.                                      */
/***************************************************************/
int CONTROL_STORE[CONTROL_STORE_ROWS][CONTROL_STORE_BITS];

/***************************************************************/
/* Main memory.                                                */
/***************************************************************/
/* MEMORY[A][0] stores the least significant byte of word at word address A
   MEMORY[A][1] stores the most significant byte of word at word address A 
   There are two write enable signals, one for each byte. WE0 is used for 
   the least significant byte of a word. WE1 is used for the most significant 
   byte of a word. */

#define WORDS_IN_MEM 0x08000
#define MEM_CYCLES 5
int MEMORY[WORDS_IN_MEM][2];

/***************************************************************/

/***************************************************************/

/***************************************************************/
/* LC-3b State info.                                           */
/***************************************************************/
#define LC_3b_REGS 8

int RUN_BIT; /* run bit */
int BUS;     /* value of the bus */

typedef struct System_Latches_Struct
{

    int PC,  /* program counter */
        MDR, /* memory data register */
        MAR, /* memory address register */
        IR,  /* instruction register */
        N,   /* n condition bit */
        Z,   /* z condition bit */
        P,   /* p condition bit */
        BEN; /* ben register */

    int READY; /* ready bit */
               /* The ready bit is also latched as you dont want the memory system to assert it 
     at a bad point in the cycle*/

    int REGS[LC_3b_REGS]; /* register file. */

    int MICROINSTRUCTION[CONTROL_STORE_BITS]; /* The microintruction */

    int STATE_NUMBER; /* Current State Number - Provided for debugging */
} System_Latches;

/* Data Structure for Latch */

System_Latches CURRENT_LATCHES, NEXT_LATCHES;

/***************************************************************/
/* A cycle counter.                                            */
/***************************************************************/
int CYCLE_COUNT;

/***************************************************************/
/*                                                             */
/* Procedure : help                                            */
/*                                                             */
/* Purpose   : Print out a list of commands.                   */
/*                                                             */
/***************************************************************/
void help()
{
    printf("----------------LC-3bSIM Help-------------------------\n");
    printf("go               -  run program to completion       \n");
    printf("run n            -  execute program for n cycles    \n");
    printf("mdump low high   -  dump memory from low to high    \n");
    printf("rdump            -  dump the register & bus values  \n");
    printf("?                -  display this help menu          \n");
    printf("quit             -  exit the program                \n\n");
}

/***************************************************************/
/*                                                             */
/* Procedure : cycle                                           */
/*                                                             */
/* Purpose   : Execute a cycle                                 */
/*                                                             */
/***************************************************************/
void cycle()
{

    eval_micro_sequencer();
    cycle_memory();
    eval_bus_drivers();
    drive_bus();
    latch_datapath_values();

    CURRENT_LATCHES = NEXT_LATCHES;

    CYCLE_COUNT++;
}

/***************************************************************/
/*                                                             */
/* Procedure : run n                                           */
/*                                                             */
/* Purpose   : Simulate the LC-3b for n cycles.                 */
/*                                                             */
/***************************************************************/
void run(int num_cycles)
{
    int i;

    if (RUN_BIT == FALSE)
    {
        printf("Can't simulate, Simulator is halted\n\n");
        return;
    }

    printf("Simulating for %d cycles...\n\n", num_cycles);
    for (i = 0; i < num_cycles; i++)
    {
        if (CURRENT_LATCHES.PC == 0x0000)
        {
            RUN_BIT = FALSE;
            printf("Simulator halted\n\n");
            break;
        }
        cycle();
    }
}

/***************************************************************/
/*                                                             */
/* Procedure : go                                              */
/*                                                             */
/* Purpose   : Simulate the LC-3b until HALTed.                 */
/*                                                             */
/***************************************************************/
void go()
{
    if (RUN_BIT == FALSE)
    {
        printf("Can't simulate, Simulator is halted\n\n");
        return;
    }

    printf("Simulating...\n\n");
    while (CURRENT_LATCHES.PC != 0x0000)
        cycle();
    RUN_BIT = FALSE;
    printf("Simulator halted\n\n");
}

/***************************************************************/
/*                                                             */
/* Procedure : mdump                                           */
/*                                                             */
/* Purpose   : Dump a word-aligned region of memory to the     */
/*             output file.                                    */
/*                                                             */
/***************************************************************/
void mdump(FILE *dumpsim_file, int start, int stop)
{
    int address; /* this is a byte address */

    printf("\nMemory content [0x%.4x..0x%.4x] :\n", start, stop);
    printf("-------------------------------------\n");
    for (address = (start >> 1); address <= (stop >> 1); address++)
        printf("  0x%.4x (%d) : 0x%.2x%.2x\n", address << 1, address << 1, MEMORY[address][1], MEMORY[address][0]);
    printf("\n");

    /* dump the memory contents into the dumpsim file */
    fprintf(dumpsim_file, "\nMemory content [0x%.4x..0x%.4x] :\n", start, stop);
    fprintf(dumpsim_file, "-------------------------------------\n");
    for (address = (start >> 1); address <= (stop >> 1); address++)
        fprintf(dumpsim_file, " 0x%.4x (%d) : 0x%.2x%.2x\n", address << 1, address << 1, MEMORY[address][1], MEMORY[address][0]);
    fprintf(dumpsim_file, "\n");
    fflush(dumpsim_file);
}

/***************************************************************/
/*                                                             */
/* Procedure : rdump                                           */
/*                                                             */
/* Purpose   : Dump current register and bus values to the     */
/*             output file.                                    */
/*                                                             */
/***************************************************************/
void rdump(FILE *dumpsim_file)
{
    int k;

    printf("\nCurrent register/bus values :\n");
    printf("-------------------------------------\n");
    printf("Cycle Count  : %d\n", CYCLE_COUNT);
    printf("PC           : 0x%.4x\n", CURRENT_LATCHES.PC);
    printf("IR           : 0x%.4x\n", CURRENT_LATCHES.IR);
    printf("STATE_NUMBER : 0x%.4x\n\n", CURRENT_LATCHES.STATE_NUMBER);
    printf("BUS          : 0x%.4x\n", BUS);
    printf("MDR          : 0x%.4x\n", CURRENT_LATCHES.MDR);
    printf("MAR          : 0x%.4x\n", CURRENT_LATCHES.MAR);
    printf("CCs: N = %d  Z = %d  P = %d\n", CURRENT_LATCHES.N, CURRENT_LATCHES.Z, CURRENT_LATCHES.P);
    printf("Registers:\n");
    for (k = 0; k < LC_3b_REGS; k++)
        printf("%d: 0x%.4x\n", k, CURRENT_LATCHES.REGS[k]);
    printf("\n");

    /* dump the state information into the dumpsim file */
    fprintf(dumpsim_file, "\nCurrent register/bus values :\n");
    fprintf(dumpsim_file, "-------------------------------------\n");
    fprintf(dumpsim_file, "Cycle Count  : %d\n", CYCLE_COUNT);
    fprintf(dumpsim_file, "PC           : 0x%.4x\n", CURRENT_LATCHES.PC);
    fprintf(dumpsim_file, "IR           : 0x%.4x\n", CURRENT_LATCHES.IR);
    fprintf(dumpsim_file, "STATE_NUMBER : 0x%.4x\n\n", CURRENT_LATCHES.STATE_NUMBER);
    fprintf(dumpsim_file, "BUS          : 0x%.4x\n", BUS);
    fprintf(dumpsim_file, "MDR          : 0x%.4x\n", CURRENT_LATCHES.MDR);
    fprintf(dumpsim_file, "MAR          : 0x%.4x\n", CURRENT_LATCHES.MAR);
    fprintf(dumpsim_file, "CCs: N = %d  Z = %d  P = %d\n", CURRENT_LATCHES.N, CURRENT_LATCHES.Z, CURRENT_LATCHES.P);
    fprintf(dumpsim_file, "Registers:\n");
    for (k = 0; k < LC_3b_REGS; k++)
        fprintf(dumpsim_file, "%d: 0x%.4x\n", k, CURRENT_LATCHES.REGS[k]);
    fprintf(dumpsim_file, "\n");
    fflush(dumpsim_file);
}

/***************************************************************/
/*                                                             */
/* Procedure : get_command                                     */
/*                                                             */
/* Purpose   : Read a command from standard input.             */
/*                                                             */
/***************************************************************/
void get_command(FILE *dumpsim_file)
{
    char buffer[20];
    int start, stop, cycles;

    printf("LC-3b-SIM> ");

    scanf("%s", buffer);
    printf("\n");

    switch (buffer[0])
    {
    case 'G':
    case 'g':
        go();
        break;

    case 'M':
    case 'm':
        scanf("%i %i", &start, &stop);
        mdump(dumpsim_file, start, stop);
        break;

    case '?':
        help();
        break;
    case 'Q':
    case 'q':
        printf("Bye.\n");
        exit(0);

    case 'R':
    case 'r':
        if (buffer[1] == 'd' || buffer[1] == 'D')
            rdump(dumpsim_file);
        else
        {
            scanf("%d", &cycles);
            run(cycles);
        }
        break;

    default:
        printf("Invalid Command\n");
        break;
    }
}

/***************************************************************/
/*                                                             */
/* Procedure : init_control_store                              */
/*                                                             */
/* Purpose   : Load microprogram into control store ROM        */
/*                                                             */
/***************************************************************/
void init_control_store(char *ucode_filename)
{
    FILE *ucode;
    int i, j, index;
    char line[200];

    printf("Loading Control Store from file: %s\n", ucode_filename);

    /* Open the micro-code file. */
    if ((ucode = fopen(ucode_filename, "r")) == NULL)
    {
        printf("Error: Can't open micro-code file %s\n", ucode_filename);
        exit(-1);
    }

    /* Read a line for each row in the control store. */
    for (i = 0; i < CONTROL_STORE_ROWS; i++)
    {
        if (fscanf(ucode, "%[^\n]\n", line) == EOF)
        {
            printf("Error: Too few lines (%d) in micro-code file: %s\n",
                   i, ucode_filename);
            exit(-1);
        }

        /* Put in bits one at a time. */
        index = 0;

        for (j = 0; j < CONTROL_STORE_BITS; j++)
        {
            /* Needs to find enough bits in line. */
            if (line[index] == '\0')
            {
                printf("Error: Too few control bits in micro-code file: %s\nLine: %d\n",
                       ucode_filename, i);
                exit(-1);
            }
            if (line[index] != '0' && line[index] != '1')
            {
                printf("Error: Unknown value in micro-code file: %s\nLine: %d, Bit: %d\n",
                       ucode_filename, i, j);
                exit(-1);
            }

            /* Set the bit in the Control Store. */
            CONTROL_STORE[i][j] = (line[index] == '0') ? 0 : 1;
            index++;
        }

        /* Warn about extra bits in line. */
        if (line[index] != '\0')
            printf("Warning: Extra bit(s) in control store file %s. Line: %d\n",
                   ucode_filename, i);
    }
    printf("\n");
}

/************************************************************/
/*                                                          */
/* Procedure : init_memory                                  */
/*                                                          */
/* Purpose   : Zero out the memory array                    */
/*                                                          */
/************************************************************/
void init_memory()
{
    int i;

    for (i = 0; i < WORDS_IN_MEM; i++)
    {
        MEMORY[i][0] = 0;
        MEMORY[i][1] = 0;
    }
}

/**************************************************************/
/*                                                            */
/* Procedure : load_program                                   */
/*                                                            */
/* Purpose   : Load program and service routines into mem.    */
/*                                                            */
/**************************************************************/
void load_program(char *program_filename)
{
    FILE *prog;
    int ii, word, program_base;

    /* Open program file. */
    prog = fopen(program_filename, "r");
    if (prog == NULL)
    {
        printf("Error: Can't open program file %s\n", program_filename);
        exit(-1);
    }

    /* Read in the program. */
    if (fscanf(prog, "%x\n", &word) != EOF)
        program_base = word >> 1;
    else
    {
        printf("Error: Program file is empty\n");
        exit(-1);
    }

    ii = 0;
    while (fscanf(prog, "%x\n", &word) != EOF)
    {
        /* Make sure it fits. */
        if (program_base + ii >= WORDS_IN_MEM)
        {
            printf("Error: Program file %s is too long to fit in memory. %x\n",
                   program_filename, ii);
            exit(-1);
        }

        /* Write the word to memory array. */
        MEMORY[program_base + ii][0] = word & 0x00FF;
        MEMORY[program_base + ii][1] = (word >> 8) & 0x00FF;
        ii++;
    }

    if (CURRENT_LATCHES.PC == 0)
        CURRENT_LATCHES.PC = (program_base << 1);

    printf("Read %d words from program into memory.\n\n", ii);
}

/***************************************************************/
/*                                                             */
/* Procedure : initialize                                      */
/*                                                             */
/* Purpose   : Load microprogram and machine language program  */
/*             and set up initial state of the machine.        */
/*                                                             */
/***************************************************************/
void initialize(char *ucode_filename, char *program_filename, int num_prog_files)
{
    int i;
    init_control_store(ucode_filename);

    init_memory();
    for (i = 0; i < num_prog_files; i++)
    {
        load_program(program_filename);
        while (*program_filename++ != '\0')
            ;
    }
    CURRENT_LATCHES.Z = 1;
    CURRENT_LATCHES.STATE_NUMBER = INITIAL_STATE_NUMBER;
    memcpy(CURRENT_LATCHES.MICROINSTRUCTION, CONTROL_STORE[INITIAL_STATE_NUMBER], sizeof(int) * CONTROL_STORE_BITS);

    NEXT_LATCHES = CURRENT_LATCHES;

    RUN_BIT = TRUE;
}

/***************************************************************/
/*                                                             */
/* Procedure : main                                            */
/*                                                             */
/***************************************************************/
int main(int argc, char *argv[])
{
    FILE *dumpsim_file;

    /* Error Checking */
    if (argc < 3)
    {
        printf("Error: usage: %s <micro_code_file> <program_file_1> <program_file_2> ...\n",
               argv[0]);
        exit(1);
    }

    printf("LC-3b Simulator\n\n");

    initialize(argv[1], argv[2], argc - 2);

    if ((dumpsim_file = fopen("dumpsim", "w")) == NULL)
    {
        printf("Error: Can't open dumpsim file\n");
        exit(-1);
    }

    while (1)
        get_command(dumpsim_file);
}

/***************************************************************/
/* Do not modify the above code.
   You are allowed to use the following global variables in your
   code. These are defined above.

   CONTROL_STORE
   MEMORY
   BUS

   CURRENT_LATCHES
   NEXT_LATCHES

   You may define your own local/global variables and functions.
   You may use the functions to get at the control bits defined
   above.

   Begin your code here 	  			       */
/***************************************************************/

int current_state;
int *instruction;
void eval_micro_sequencer()
{

    /* 
   * Evaluate the address of the next state according to the 
   * micro sequencer logic. Latch the next microinstruction.
   */
    int j0, j1, j2, j3, j4, j5, jreg, cond0, cond1, ir11, ready, ben, ird;

    instruction = CURRENT_LATCHES.MICROINSTRUCTION;
    current_state = CURRENT_LATCHES.STATE_NUMBER;
    cond0 = instruction[COND0];
    cond1 = instruction[COND1];
    ir11 = (CURRENT_LATCHES.IR & 0x00000800) >> 11;
    ready = CURRENT_LATCHES.READY;
    ben = CURRENT_LATCHES.BEN;
    ird = instruction[IRD];
    j0 = instruction[J0] || (ir11 && cond0 && cond1);
    j1 = (instruction[J1] || ((~cond1) && cond0 && ready)) << 1;
    j2 = (instruction[J2] || ((~cond0) && ben && cond1)) << 2;
    j3 = instruction[J3] << 3;
    j4 = instruction[J4] << 4;
    j5 = instruction[J5] << 5;
    jreg = j0 | j1 | j2 | j3 | j4 | j5;
    if (ird)
    {
        NEXT_LATCHES.STATE_NUMBER = (CURRENT_LATCHES.IR & 0x0000f000) >> 12;
        for (int i = 0; i < CONTROL_STORE_BITS; i++)
        {
            NEXT_LATCHES.MICROINSTRUCTION[i] = CONTROL_STORE[NEXT_LATCHES.STATE_NUMBER][i];
        }
    }
    else
    {
        NEXT_LATCHES.STATE_NUMBER = jreg;
        for (int i = 0; i < CONTROL_STORE_BITS; i++)
        {
            NEXT_LATCHES.MICROINSTRUCTION[i] = CONTROL_STORE[NEXT_LATCHES.STATE_NUMBER][i];
        }
    }
}

int cycle_count;
void cycle_memory()
{

    /* 
   * This function emulates memory and the WE logic. 
   * Keep track of which cycle of MEMEN we are dealing with.  
   * If fourth, we need to latch Ready bit at the end of 
   * cycle to prepare microsequencer for the fifth cycle.  
   */
    int mioen, rw, we0, we1, data_size, mar0;
    mioen = instruction[MIO_EN];
    rw = instruction[R_W];
    data_size = instruction[DATA_SIZE];
    mar0 = CURRENT_LATCHES.MAR & 0x00000001;
    // we only occurs if the machine is doing a store
    we0 = ~mar0 && rw;
    we1 = rw && (mar0 ^ data_size);
    if (mioen)
    {
        if (cycle_count < 5)
        {
            cycle_count++;
            if (cycle_count == 4)
            {
                NEXT_LATCHES.READY = 1;
            }
        }
        if (CURRENT_LATCHES.READY)
        {
            //perform memory operation
            if (we0)
            {
                //writing to memory mMAR[7:0] <- MDR[7:0] 
                MEMORY[CURRENT_LATCHES.MAR>>1][0] = Low16bits(CURRENT_LATCHES.MDR & 0x000000ff);
            }
            if(we1)
            {
                // writing to memory mMAR[15:8] <- MDR[15:8]
                MEMORY[CURRENT_LATCHES.MAR>>1][1] = Low16bits((CURRENT_LATCHES.MDR>>8) & 0x000000ff);
            }
            else{
                // reading from memory
                NEXT_LATCHES.MDR = Low16bits(MEMORY[CURRENT_LATCHES.MAR>>1][0] | (MEMORY[CURRENT_LATCHES.MAR>>1][1]<<8)); 
                
            }
            cycle_count = 0;
            NEXT_LATCHES.READY = 0;
        }
    }
}

int mar_mux_res, pc_res, alu_res, shf_res, mdr_res, adder_res;
void eval_bus_drivers()
{
    int gate_mar_mux, gate_pc, gate_alu, gate_shf, gate_mdr, mar_mux, addr2_mux, addr1_mux, addr2_mux_reg,
    addr1_mux_reg, sr1_out, sr1_mux, sr2_out, lshf, ld_pc, aluk, data_size;
    /* 
   * Datapath routine emulating operations before driving the bus.
   * Evaluate the input of tristate drivers 
   *         Gate_MARMUX,
   *		 Gate_PC,
   *		 Gate_ALU,
   *		 Gate_SHF,
   *		 Gate_MDR.
   */
  gate_mar_mux = instruction[GATE_MARMUX];
  gate_pc = instruction[GATE_PC];
  gate_alu = instruction[GATE_ALU];
  gate_shf = instruction[GATE_SHF];
  gate_mdr = instruction[GATE_MDR];
  mar_mux = instruction[MARMUX];
  addr2_mux = instruction[ADDR2MUX0] | (instruction[ADDR2MUX1]<<1);
  addr1_mux = instruction[ADDR1MUX];
  sr1_mux = instruction[SR1MUX];
  lshf = instruction[LSHF1];
  ld_pc = instruction[LD_PC];
  aluk = instruction[ALUK0] | (instruction[ALUK1]<<1);
  data_size = instruction[DATA_SIZE];

    // retrieving sr1 value 
  if(sr1_mux){
      sr1_out = CURRENT_LATCHES.REGS[(CURRENT_LATCHES.IR & 0x000001c0)>>6];
  }
  else{
      sr1_out = CURRENT_LATCHES.REGS[(CURRENT_LATCHES.IR & 0x00000e00)>>9];
  }

  // retrieving sr2 value
  if(CURRENT_LATCHES.IR & 0x00000020){
      sr2_out = sign_extend(CURRENT_LATCHES.IR & 0x0000001f, 4);
  }
  else{
      sr2_out = CURRENT_LATCHES.REGS[CURRENT_LATCHES.IR & 0x00000007];
  }
  
  // retrieving the addr1mux value
  if(addr1_mux){
      addr1_mux_reg = CURRENT_LATCHES.REGS[(CURRENT_LATCHES.IR & 0x000001c0)>>6];
  }
  else{
      addr1_mux_reg = CURRENT_LATCHES.PC;
  }

  // retrieving addr2mux
  if(addr2_mux == 1){
      // sext ir[5:0]
      addr2_mux_reg = sign_extend(CURRENT_LATCHES.IR & 0x0000003f,5);
  }
  else if(addr2_mux == 2){
      // sext ir[8:0]
      addr2_mux_reg = sign_extend(CURRENT_LATCHES.IR & 0x000001ff , 8);
  }
  else if(addr2_mux == 3){
      // sext ir[10:0]
      addr2_mux_reg = sign_extend(CURRENT_LATCHES.IR & 0x000007ff , 10);
  }
  else{
      addr2_mux_reg = 0;
  }

  // handling lshf1
  if(lshf){
      addr2_mux_reg = addr2_mux_reg << 1;
  }

   // placing the adder result on wire in case ld.pc is high
  adder_res = Low16bits(addr1_mux_reg + addr2_mux_reg);


  // putting the marmux result on bus
  if(gate_mar_mux){
      if(mar_mux){
          // addr result
          mar_mux_res = adder_res;
      }else{
          //lshf(zext[IR[7:0]],1)
          mar_mux_res = Low16bits(sign_extend(CURRENT_LATCHES.IR & 0x000000ff,7)<<1);
      }
  }
  // if the program counter needs to be on bus
  else if(gate_pc){
      pc_res = Low16bits(CURRENT_LATCHES.PC);
  }
  // if we need to use the arithmetic logic unit
  else if(gate_alu){
      if(aluk == 0){
          // add
          alu_res = Low16bits(sr1_out + sr2_out);
      }
      else if(aluk == 1){
          // and
          alu_res = Low16bits(sr1_out & sr2_out); 
      }
      else if(aluk == 2){
          // xor
          alu_res = Low16bits(sr1_out ^ sr2_out);
      }
      else if (aluk == 3){
          // passa
          alu_res = Low16bits(sr1_out);
      }
  }
  // if the shifter is needed
  else if(gate_shf){
      // left shift
      if(((CURRENT_LATCHES.IR & 0x00000030) >>4) == 0){
          shf_res = Low16bits(sr1_out << (CURRENT_LATCHES.IR & 0x0000000f));
      }
      // right shift logical
      else if(((CURRENT_LATCHES.IR & 0x00000030) >>4) == 1){
          shf_res = Low16bits(sr1_out >> (CURRENT_LATCHES.IR & 0x0000000f));
      }
      // right shift arithmetic
      else if(((CURRENT_LATCHES.IR & 0x00000030) >>4) == 3){
          shf_res = Low16bits(sign_extend(sr1_out >>(CURRENT_LATCHES.IR & 0x0000000f), 15-(CURRENT_LATCHES.IR & 0x0000000f)));
      }
  }
  // get what is stored on the mdr
  else if(gate_mdr){
      if(data_size){
          mdr_res = Low16bits(CURRENT_LATCHES.MDR);
      }
      else {
          if(CURRENT_LATCHES.MAR & 0x00000001){
              mdr_res = Low16bits(sign_extend((CURRENT_LATCHES.MDR & 0x0000ff00)>>8,7));
          }
          else{
              mdr_res = Low16bits(sign_extend(CURRENT_LATCHES.MDR & 0x000000ff,7));
          }
      }
  }


}

int BUS;
void drive_bus()
{

    /* 
   * Datapath routine for driving the bus from one of the 5 possible 
   * tristate drivers. 
   */
  if(instruction[GATE_MARMUX]){
      BUS = Low16bits(mar_mux_res);
  }
  else if(instruction[GATE_PC]){
      BUS = Low16bits(pc_res);
  }
  else if(instruction[GATE_SHF]){
      BUS = Low16bits(shf_res);
  }
  else if(instruction[GATE_ALU]){
      BUS = Low16bits(alu_res);
  }
  else if(instruction[GATE_MDR]){
      BUS = Low16bits(mdr_res);
  }
  else {
      BUS = 0;
  }
  
}

void latch_datapath_values()
{

    /* 
   * Datapath routine for computing all functions that need to latch
   * values in the data path at the end of this cycle.  Some values
   * require sourcing the bus; therefore, this routine has to come 
   * after drive_bus.
   */
  // make sure to check the ldpc
    int pc_mux, dr_mux;
    pc_mux = instruction[PCMUX0] | (instruction[PCMUX1]<<1);
    dr_mux = instruction[DRMUX];
// update pc
    if(instruction[LD_PC]){
        if(pc_mux == 0){
            // pc + 2
             NEXT_LATCHES.PC = Low16bits(CURRENT_LATCHES.PC +2);
        }
        else if(pc_mux == 1){
          // bus
            NEXT_LATCHES.PC = Low16bits(BUS);
        }
        else if(pc_mux == 2){
              // adder
            NEXT_LATCHES.PC = Low16bits(adder_res);
        }
    }
    // ld ir
    if(instruction[LD_IR]){
        NEXT_LATCHES.IR = Low16bits(BUS);
    }
    // ld reg
    if(instruction[LD_REG]){
        if(dr_mux){
            NEXT_LATCHES.REGS[7] = Low16bits(BUS);
        }
        else{
            NEXT_LATCHES.REGS[(CURRENT_LATCHES.IR & 0x00000e00)>>9] = Low16bits(BUS);
        }
        if(instruction[LD_CC]){
            if(Low16bits(BUS)==0){
                // zero
                NEXT_LATCHES.Z = 1;
                NEXT_LATCHES.P = 0;
                NEXT_LATCHES.N = 0;
            }
            else if(Low16bits(BUS)&0x00008000){
                //negative
                NEXT_LATCHES.N = 1;
                NEXT_LATCHES.Z = 0;
                NEXT_LATCHES.P = 0;
            }
            else if(!(Low16bits(BUS) & 0x00008000)){
                //positive
                NEXT_LATCHES.P =1;
                NEXT_LATCHES.N = 0;
                NEXT_LATCHES.Z = 0;
            }
        }else{
            NEXT_LATCHES.N = CURRENT_LATCHES.N;
            NEXT_LATCHES.Z = CURRENT_LATCHES.Z;
            NEXT_LATCHES.P = CURRENT_LATCHES.P;
            }
    }
    // ld mar
    if(instruction[LD_MAR]){
        NEXT_LATCHES.MAR = Low16bits(BUS);
    }
    // ld mdr
    if(instruction[LD_MDR]){
        if(!instruction[MIO_EN]){
            if(instruction[DATA_SIZE]){
                // word onto mdr
                NEXT_LATCHES.MDR = Low16bits(BUS);
            }
            else{
                    NEXT_LATCHES.MDR = Low16bits((BUS & 0x000000ff) | ((BUS & 0x000000ff)<<8));
                
            }
        }
    }

    int n = (CURRENT_LATCHES.IR & 0x00000800)>>11;
    int z = (CURRENT_LATCHES.IR & 0x00000400)>>10;
    int p = (CURRENT_LATCHES.IR & 0x00000200)>>9;
    if(instruction[LD_BEN]){
        NEXT_LATCHES.BEN = (CURRENT_LATCHES.N && n) || (CURRENT_LATCHES.Z && z) || (CURRENT_LATCHES.P && p);
    }



}

int sign_extend(int num, int bit){
    int return_val;
    if((num >> bit)){
        return_val = num | (0xffffffff << bit);
    }else{return_val = num;}
    return Low16bits(return_val);
}
