/* ************************ 
   Pipeline Cache Simulator
   ************************ */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <strings.h>
#include <stdbool.h>

#define MAX_CACHE_SIZE 10240
#define CACHE_MISS_DELAY 10 // 10 cycle cache miss penalty
#define MAX_STAGES 5



//****** Functions *****//
// Initialize the simulator
void iplc_sim_init(int index, int blocksize, int assoc);

// Cache Simulator Functions
void iplc_sim_LRU_replace_on_miss(int index, int tag);
void iplc_sim_LRU_update_on_hit(int index, int assoc);
int iplc_sim_trap_address(unsigned int address);

// Pipeline Functions
unsigned int iplc_sim_parse_reg(char *reg_str);
void iplc_sim_parse_instruction(char *buffer);
void iplc_sim_push_pipeline_stage();
void iplc_sim_process_pipeline_rtype(char *instruction, int dest_reg, int reg1, int reg2_or_constant);
void iplc_sim_process_pipeline_lw(int dest_reg, int base_reg, unsigned int data_address);
void iplc_sim_process_pipeline_sw(int src_reg, int base_reg, unsigned int data_address);
void iplc_sim_process_pipeline_branch(int reg1, int reg2);
void iplc_sim_process_pipeline_jump();
void iplc_sim_process_pipeline_syscall();
void iplc_sim_process_pipeline_nop();

// Outout performance results
void iplc_sim_finalize();



//*****Variables and Data Structures*****//
typedef struct cache_line {
    /* Your data structures for implementing your cache should include:
       a valid bit
       a tag
       a method for handling varying levels of associativity
       a method for selecting which item in the cache is going to be replaced */
    char* valid_bit;
    int* tag;
    int* age; // Counter for time since last access
} cache_line_t;

typedef struct pa_run {
    /* Structure to hold the performance analysis sims */
    int index;
    int blocksize;
    int associativity;
    int branch_pred;
    double cpi;
    double cmr; // cache miss rate
} pa_run_t;

// Stats for the various instructions
typedef struct instruction_stats {
    int rtype;
    int lw;
    int sw;
    int branch;
    int jump;
    int syscall;
    int nop;
} inst_stats_t;
inst_stats_t inst_stats = {0,0,0,0,0,0,0};

// Cache Variables
cache_line_t* cache = NULL;
int cache_index = 0;
int cache_blocksize = 0;
int cache_blockoffsetbits = 0;
int cache_assoc = 0;

// Cache Statistics
long cache_miss = 0;
long cache_access = 0;
long cache_hit = 0;

char instruction[16];
char reg1[16];
char reg2[16];
char offsetwithreg[16];
unsigned int data_address = 0;
unsigned int instruction_address = 0;
unsigned int pipeline_cycles = 0;   // how many cycles did you pipeline consume
unsigned int instruction_count = 0; // home many real instructions ran thru the pipeline
unsigned int branch_predict_taken = 0;
unsigned int branch_count = 0;
unsigned int correct_branch_predictions = 0;

unsigned int debug = 0;
unsigned int dump_pipeline = 1;

enum instruction_type {NOP, RTYPE, LW, SW, BRANCH, JUMP, JAL, SYSCALL};

typedef struct rtype {
    char instruction[16];
    int reg1;
    int reg2_or_constant;
    int dest_reg;
} rtype_t;

typedef struct load_word {
    unsigned int data_address;
    int dest_reg;
    int base_reg;
} lw_t;

typedef struct store_word {
    unsigned int data_address;
    int src_reg;
    int base_reg;
} sw_t;

typedef struct branch {
    int reg1;
    int reg2;
} branch_t;


typedef struct jump {
    char instruction[16];
} jump_t;

typedef struct pipeline {
    enum instruction_type itype;
    unsigned int instruction_address;
    union {
        rtype_t   rtype;
        lw_t      lw;
        sw_t      sw;
        branch_t  branch;
        jump_t    jump;
    }
    stage;
} pipeline_t;

enum pipeline_stages {FETCH, DECODE, ALU, MEM, WRITEBACK};

pipeline_t pipeline[MAX_STAGES];



//*****Cache Function Implementations*****//
// Correctly configure the cache
void iplc_sim_init(int index, int blocksize, int assoc) {
    int i = 0, j = 0;
    unsigned long cache_size = 0;
    cache_index = index;
    cache_blocksize = blocksize;
    cache_assoc = assoc;
    
    cache_blockoffsetbits = (int) ceil((blocksize * 4) / 2);
    /* Note: rint function rounds the result up prior to casting */
    
    cache_size = assoc * (1 << index) * ((32 * blocksize) + 33 - index - cache_blockoffsetbits);
    
    printf("Cache Configuration \n");
    printf("   Index: %d bits or %d lines \n", cache_index, (1 << cache_index));
    printf("   BlockSize: %d \n", cache_blocksize);
    printf("   Associativity: %d \n", cache_assoc);
    printf("   BlockOffSetBits: %d \n", cache_blockoffsetbits);
    printf("   CacheSize: %lu \n", cache_size);
    
    if (cache_size > MAX_CACHE_SIZE) {
        printf("Cache too big. Great than MAX SIZE of %d .... \n", MAX_CACHE_SIZE);
        exit(-1);
    }
    
    cache = (cache_line_t*) malloc((sizeof(cache_line_t) * 1 << index));
    
    // Dynamically create our cache based on the information the user entered
    for (i = 0; i < (1 << index); i++) {
        // Dynamically allocate the members of each cache set
        cache[i].valid_bit = (char*) calloc(assoc, sizeof(char)); // We use calloc to initialize the valid bits to zero
        cache[i].tag = (int*) malloc(sizeof(int) * assoc);
        cache[i].age = (int*) calloc(assoc, sizeof(int));
    }
    
    // Init the pipeline -- set all data to zero and instructions to NOP
    for (i = 0; i < MAX_STAGES; i++) {
        // itype is set to O which is NOP type instruction
        bzero(&(pipeline[i]), sizeof(pipeline_t));
    }
}

/*  iplc_sim_trap_address() determined this is not in our cache. Put it there
    and make sure that is now our Most Recently Used (MRU) entry. */
void iplc_sim_LRU_replace_on_miss(int index, int tag) {
    int i;
    int oldest_age = 0;
    int target_line = 0;
    
    // Find the target block to insert our new block
    for (i = 0; i < cache_assoc; i++) {
        // If there is an empty space, just insert it
        if (cache[index].valid_bit[i] == 0) {
            target_line = i;
            break;
        }
        
        // Find the oldest block and mark it for replacement
        if (cache[index].age[i] > oldest_age) {
            oldest_age = cache[index].age[i];
            target_line = i;
        }
    }
    
    // Replace the tage of the target block and change the valid bit
    cache[index].tag[target_line] = tag;
    cache[index].valid_bit[target_line] = 1;
    
    // We now update the data for all valid blocks
    iplc_sim_LRU_update_on_hit(index, target_line);
}

/*  iplc_sim_trap_address() determined the entry is in our cache. Update its
    information in the cache. */
void iplc_sim_LRU_update_on_hit(int index, int assoc_entry) {
    int i;
   
    // Update all age counters for each valid line
    cache[index].age[assoc_entry] = 0;
    for (i = 0; i < cache_assoc; i++) {
        if (cache[index].valid_bit[i] == 1) {
            cache[index].age[i] += 1;
        }
    }
}

/*  Check if the address is in our cache. Update our counter statistics 
    for cache_access, cache_hit, etc. If our configuration supports
    associativity we may need to check through multiple entries for our
    desired index.  In that case we will also need to call the LRU functions. */
int iplc_sim_trap_address(unsigned int address) {

    int i, hit = 0, set_element = 0;
    int index = (1 << cache_index - 1)  & (address >> cache_blockoffsetbits); // Isolates the index

    int tag = address >> (cache_index + cache_blockoffsetbits); // Isolates the tag
    
    // Search for the appropriate tag in the appropriate set
    for (i = 0; i < cache_assoc; i++) {
        // Handle the case of a cahe hit
        if (cache[index].valid_bit[i] == 1 && cache[index].tag[i] == tag) {
            hit = 1;
            cache_hit += 1;
            iplc_sim_LRU_update_on_hit(index, i);
            break;
        }
    }
    
    // Handle the case of a cache miss
    if (!hit) {
        cache_miss += 1;
        iplc_sim_LRU_replace_on_miss(index, tag);
    }
    
    // Increment access counter
    cache_access += 1;
    
    // Expects you to return 1 for hit, 0 for miss
    return hit;
}

// Just output our summary statistics.
void iplc_sim_finalize() {
    // Finish processing all instructions in the Pipeline
    while (pipeline[FETCH].itype != NOP || pipeline[DECODE].itype != NOP || pipeline[ALU].itype != NOP ||
           pipeline[MEM].itype != NOP   || pipeline[WRITEBACK].itype != NOP) {
        iplc_sim_push_pipeline_stage();
    }
    
    printf(" Cache Performance \n");
    printf("\t Number of Cache Accesses is %ld \n", cache_access);
    printf("\t Number of Cache Misses is %ld \n", cache_miss);
    printf("\t Number of Cache Hits is %ld \n", cache_hit);
    printf("\t Cache Miss Rate is %f \n\n", (double)cache_miss / (double) cache_access);
    printf("Pipeline Performance \n");
    printf("\t Total Cycles is %u \n", pipeline_cycles);
    printf("\t Total Instructions is %u \n", instruction_count);
    printf("\t Total Branch Instructions is %u \n", branch_count);
    printf("\t Total Correct Branch Predictions is %u \n", correct_branch_predictions);
    printf("\t CPI is %f \n\n", (double)pipeline_cycles / (double) instruction_count);
}



//*****Pipeline Functions*****//
// Dump the current contents of our pipeline
void iplc_sim_dump_pipeline() {
    int i;
    
    for (i = 0; i < MAX_STAGES; i++) {
        switch(i) {
            case FETCH:
                printf("(cyc: %u) FETCH:\t %d: 0x%x \t", pipeline_cycles, pipeline[i].itype,
                       pipeline[i].instruction_address);
                break;
            case DECODE:
                printf("DECODE:\t %d: 0x%x \t", pipeline[i].itype, pipeline[i].instruction_address);
                break;
            case ALU:
                printf("ALU:\t %d: 0x%x \t", pipeline[i].itype, pipeline[i].instruction_address);
                break;
            case MEM:
                printf("MEM:\t %d: 0x%x \t", pipeline[i].itype, pipeline[i].instruction_address);
                break;
            case WRITEBACK:
                printf("WB:\t %d: 0x%x \n", pipeline[i].itype, pipeline[i].instruction_address);
                break;
            default:
                printf("DUMP: Bad stage!\n" );
                exit(-1);
        }
    }
}

/*
 * Check if various stages of our pipeline require stalls, forwarding, etc.
 * Then push the contents of our various pipeline stages through the pipeline.
 */
void iplc_sim_push_pipeline_stage()
{
    int i;
    int data_hit=1;

    int stall = 0;
    
    /* 1. Count WRITEBACK stage is "retired" -- This I'm giving you */
    if (pipeline[WRITEBACK].instruction_address) {
        instruction_count++;
        if (debug)
            printf("DEBUG: Retired Instruction at 0x%x, Type %d, at Time %u \n",
                   pipeline[WRITEBACK].instruction_address, pipeline[WRITEBACK].itype, pipeline_cycles);
    }
    
    /* 2. Check for BRANCH and correct/incorrect Branch Prediction */
    if (pipeline[DECODE].itype == BRANCH) {
        int branch_taken = 0;
        branch_count++;
        if(pipeline[FETCH].instruction_address != (pipeline[DECODE].instruction_address + 4) ){
            branch_taken++;
        }
        if(branch_predict_taken){ // if choose predict take branches and next instruction is not at address+4,
                          // prediction correct.  Else add nop for stall
            if(branch_taken){
                correct_branch_predictions++;
            }
            else{
                //memcpy(&pipeline[WRITEBACK], &pipeline[MEM], sizeof(pipeline_t));
                //memcpy(&pipeline[MEM], &pipeline[ALU], sizeof(pipeline_t));
                //memcpy(&pipeline[ALU], &pipeline[DECODE], sizeof(pipeline_t));
                //memcpy(&pipeline[DECODE], &pipeline[FETCH], sizeof(pipeline_t));
                //bzero(&(pipeline[FETCH]), sizeof(pipeline_t));
                stall = 1; // if incorrect remove this variable and just increment cycles
            }
        }
        else{
            if(!branch_taken){
                correct_branch_predictions++;
            }
            else{
                //memcpy(&pipeline[WRITEBACK], &pipeline[MEM], sizeof(pipeline_t));
                //memcpy(&pipeline[MEM], &pipeline[ALU], sizeof(pipeline_t));
                //memcpy(&pipeline[ALU], &pipeline[DECODE], sizeof(pipeline_t));
                //memcpy(&pipeline[DECODE], &pipeline[FETCH], sizeof(pipeline_t));
                //bzero(&(pipeline[FETCH]), sizeof(pipeline_t));
                stall = 1;
            }
        }
    }
    
    /* 3. Check for LW delays due to use in ALU stage and if data hit/miss
     *    add delay cycles if needed.
     */
    if (pipeline[MEM].itype == LW) {
        int inserted_nop = 0;
        if(pipeline[ALU].itype == RTYPE){
            if(pipeline[ALU].stage.rtype.reg1 == pipeline[MEM].stage.lw.dest_reg
                || pipeline[ALU].stage.rtype.reg2_or_constant == pipeline[MEM].stage.lw.dest_reg){
                stall++;
            }
        }
    }
    
    /* 4. Check for SW mem acess and data miss .. add delay cycles if needed */
    if (pipeline[MEM].itype == SW) {
        if(pipeline[ALU].itype == RTYPE){
            if(pipeline[ALU].stage.rtype.dest_reg == pipeline[MEM].stage.sw.base_reg){
                stall++;
            }
        }
    }
    
    /* 5. Increment pipe_cycles 1 cycle for normal processing */
    pipeline_cycles++;
    /* 6. push stages thru MEM->WB, ALU->MEM, DECODE->ALU, FETCH->ALU */ // FETCH->DECODE
    memcpy(&pipeline[WRITEBACK], &pipeline[MEM], sizeof(pipeline_t));
    memcpy(&pipeline[MEM], &pipeline[ALU], sizeof(pipeline_t));
    memcpy(&pipeline[ALU], &pipeline[DECODE], sizeof(pipeline_t));
    memcpy(&pipeline[DECODE], &pipeline[FETCH], sizeof(pipeline_t));


    
    // 7. This is a give'me -- Reset the FETCH stage to NOP via bezero */
    bzero(&(pipeline[FETCH]), sizeof(pipeline_t));

    if(stall){
        iplc_sim_push_pipeline_stage();
    }
}

/*
 * This function is fully implemented.  You should use this as a reference
 * for implementing the remaining instruction types.
 */
void iplc_sim_process_pipeline_rtype(char *instruction, int dest_reg, int reg1, int reg2_or_constant)
{
    /* This is an example of what you need to do for the rest */
    iplc_sim_push_pipeline_stage();
    
    pipeline[FETCH].itype = RTYPE;
    pipeline[FETCH].instruction_address = instruction_address;
    
    strcpy(pipeline[FETCH].stage.rtype.instruction, instruction);
    pipeline[FETCH].stage.rtype.reg1 = reg1;
    pipeline[FETCH].stage.rtype.reg2_or_constant = reg2_or_constant;
    pipeline[FETCH].stage.rtype.dest_reg = dest_reg;
}

void iplc_sim_process_pipeline_lw(int dest_reg, int base_reg, unsigned int data_address)
{
    /* You must implement this function */
    iplc_sim_push_pipeline_stage();

    pipeline[FETCH].itype = LW;
    pipeline[FETCH].instruction_address = instruction_address;

    pipeline[FETCH].stage.lw.data_address = data_address;
    pipeline[FETCH].stage.lw.dest_reg = dest_reg;
    pipeline[FETCH].stage.lw.base_reg = base_reg;
}

void iplc_sim_process_pipeline_sw(int src_reg, int base_reg, unsigned int data_address)
{
    /* You must implement this function */
    iplc_sim_push_pipeline_stage();

    pipeline[FETCH].itype = SW;
    pipeline[FETCH].instruction_address = instruction_address;

    pipeline[FETCH].stage.sw.data_address = data_address;
    pipeline[FETCH].stage.sw.src_reg = src_reg;
    pipeline[FETCH].stage.sw.base_reg = base_reg;
}

void iplc_sim_process_pipeline_branch(int reg1, int reg2)
{
    /* You must implement this function */
    iplc_sim_push_pipeline_stage();

    pipeline[FETCH].itype = BRANCH;
    pipeline[FETCH].instruction_address = instruction_address;

    pipeline[FETCH].stage.branch.reg1 = reg1;
    pipeline[FETCH].stage.branch.reg2 = reg2;
}

void iplc_sim_process_pipeline_jump(char *instruction)
{
    /* You must implement this function */
    iplc_sim_push_pipeline_stage();

    pipeline[FETCH].itype = JUMP;
    pipeline[FETCH].instruction_address = instruction_address;

    strcpy(pipeline[FETCH].stage.jump.instruction, instruction);
}

void iplc_sim_process_pipeline_syscall()
{
    /* You must implement this function */
    iplc_sim_push_pipeline_stage();

    pipeline[FETCH].itype = SYSCALL;
    pipeline[FETCH].instruction_address = instruction_address;
}

void iplc_sim_process_pipeline_nop()
{
    /* You must implement this function */
    iplc_sim_push_pipeline_stage();

    pipeline[FETCH].itype = NOP;
    pipeline[FETCH].instruction_address = instruction_address;
}



//*****Parsing Function*****//
// Don't touch this function.  It is for parsing the instruction stream.
unsigned int iplc_sim_parse_reg(char *reg_str) {
    int i;
    
    // Turn comma into \n
    if (reg_str[strlen(reg_str)-1] == ',')
        reg_str[strlen(reg_str)-1] = '\n';
    
    if (reg_str[0] != '$')
        return atoi(reg_str);
    else {
        // Copy down over $ character than return atoi
        for (i = 0; i < strlen(reg_str); i++)
            reg_str[i] = reg_str[i+1];
        
        return atoi(reg_str);
    }
}

// Don't touch this function.  It is for parsing the instruction stream.
void iplc_sim_parse_instruction(char *buffer) {
    int instruction_hit = 0;
    int i = 0, j = 0;
    int src_reg = 0;
    int src_reg2 = 0;
    int dest_reg = 0;
    char str_src_reg[16];
    char str_src_reg2[16];
    char str_dest_reg[16];
    char str_constant[16];
    
    if (sscanf(buffer, "%x %s", &instruction_address, instruction ) != 2) {
        printf("Malformed instruction \n");
        exit(-1);
    }
    
    instruction_hit = iplc_sim_trap_address( instruction_address );
    
    // if a MISS, then push current instruction thru pipeline
    if (!instruction_hit) {
        // need to subtract 1, since the stage is pushed once more for actual instruction processing
        // also need to allow for a branch miss prediction during the fetch cache miss time -- by
        // counting cycles this allows for these cycles to overlap and not doubly count.
        
        printf("INST MISS:\t Address 0x%x \n", instruction_address);
        
        for (i = pipeline_cycles, j = pipeline_cycles; i < j + CACHE_MISS_DELAY - 1; i++)
            iplc_sim_push_pipeline_stage();
    }
    else
        printf("INST HIT:\t Address 0x%x \n", instruction_address);
    
    // Parse the Instruction
    if (strncmp(instruction, "add", 3 ) == 0 ||
        strncmp(instruction, "sll", 3 ) == 0 ||
        strncmp(instruction, "ori", 3 ) == 0) {
        if (sscanf(buffer, "%x %s %s %s %s",
                   &instruction_address,
                   instruction,
                   str_dest_reg,
                   str_src_reg,
                   str_src_reg2 ) != 5) {
            printf("Malformed RTYPE instruction (%s) at address 0x%x \n",
                   instruction, instruction_address);
            exit(-1);
        }
        
        dest_reg = iplc_sim_parse_reg(str_dest_reg);
        src_reg = iplc_sim_parse_reg(str_src_reg);
        src_reg2 = iplc_sim_parse_reg(str_src_reg2);
        
        iplc_sim_process_pipeline_rtype(instruction, dest_reg, src_reg, src_reg2);
    }
    
    else if (strncmp( instruction, "lui", 3 ) == 0) {
        if (sscanf(buffer, "%x %s %s %s",
                   &instruction_address,
                   instruction,
                   str_dest_reg,
                   str_constant ) != 4 ) {
            printf("Malformed RTYPE instruction (%s) at address 0x%x \n",
                   instruction, instruction_address );
            exit(-1);
        }
        
        dest_reg = iplc_sim_parse_reg(str_dest_reg);
        src_reg = -1;
        src_reg2 = -1;
        iplc_sim_process_pipeline_rtype(instruction, dest_reg, src_reg, src_reg2);
    }
    
    else if (strncmp( instruction, "lw", 2 ) == 0 ||
             strncmp( instruction, "sw", 2 ) == 0  ) {
        if ( sscanf( buffer, "%x %s %s %s %x",
                    &instruction_address,
                    instruction,
                    reg1,
                    offsetwithreg,
                    &data_address ) != 5) {
            printf("Bad instruction: %s at address %x \n", instruction, instruction_address);
            exit(-1);
        }
        
        if (strncmp(instruction, "lw", 2 ) == 0) {
            
            dest_reg = iplc_sim_parse_reg(reg1);
            
            // Don't need to worry about base regs -- just insert -1 values
            iplc_sim_process_pipeline_lw(dest_reg, -1, data_address);
        }
        if (strncmp( instruction, "sw", 2 ) == 0) {
            src_reg = iplc_sim_parse_reg(reg1);
            
            // don't need to worry about base regs -- just insert -1 values
            iplc_sim_process_pipeline_sw( src_reg, -1, data_address);
        }
    }
    else if (strncmp( instruction, "beq", 3 ) == 0) {
        // don't need to worry about getting regs -- just insert -1 values
        iplc_sim_process_pipeline_branch(-1, -1);
    }
    else if (strncmp( instruction, "jal", 3 ) == 0 ||
             strncmp( instruction, "jr", 2 ) == 0 ||
             strncmp( instruction, "j", 1 ) == 0 ) {
        iplc_sim_process_pipeline_jump( instruction );
    }
    else if (strncmp( instruction, "jal", 3 ) == 0 ||
             strncmp( instruction, "jr", 2 ) == 0 ||
             strncmp( instruction, "j", 1 ) == 0 ) {
        /*
         * Note: no need to worry about forwarding on the jump register
         * we'll let that one go.
         */
        iplc_sim_process_pipeline_jump(instruction);
    }
    else if ( strncmp( instruction, "syscall", 7 ) == 0) {
        iplc_sim_process_pipeline_syscall( );
    }
    else if ( strncmp( instruction, "nop", 3 ) == 0) {
        iplc_sim_process_pipeline_nop( );
    }
    else {
        printf("Do not know how to process instruction: %s at address %x \n",
               instruction, instruction_address );
        exit(-1);
    }
}

/*
This function pretty prints the menu portion of the performance analysis table.
*/
void pretty_print_table_menu(char* title, char menu_sep, 
                        char* col1, char* col2, char* col3, char* col4, char* col5, char* col6,
                        int w1, int w2, int w3, int w4, int w5, int w6) {

    const char* padding = "--------------------------------------------------------------------------------";

    printf("\n");
    printf("%s:\n", 
        title);
    printf("%2c%s\n", ' ', 
        col1);
    printf("%2c%-*c%s\n", ' ', 
        w1+1, menu_sep, col2);
    printf("%2c%-*c%-*c%s\n", ' ', 
        w1+1, menu_sep, 
        w2+1, menu_sep, col3);
    printf("%2c%-*c%-*c%-*c%s\n", ' ', 
        w1+1, menu_sep, 
        w2+1, menu_sep, 
        w3+1, menu_sep, col4);
    printf("%2c%-*c%-*c%-*c%-*c%s\n", ' ', 
        w1+1, menu_sep, 
        w2+1, menu_sep, 
        w3+1, menu_sep, 
        w4+1, menu_sep, col5);
    printf("%2c%-*c%-*c%-*c%-*c%-*c%s\n", ' ', 
        w1+1, menu_sep, 
        w2+1, menu_sep, 
        w3+1, menu_sep, 
        w4+1, menu_sep, 
        w5+1, menu_sep, col6);
    printf("%2c%-*c%-*c%-*c%-*c%-*c%-*c\n", ' ', 
        w1+1, menu_sep, 
        w2+1, menu_sep, 
        w3+1, menu_sep, 
        w4+1, menu_sep, 
        w5+1, menu_sep, 
        w6+1, menu_sep);

}

/*
This function pretty prints the body portion of the performance analysis table.
Includes pointing out which run had the lowest CPI and cache miss rate.
*/
void pretty_print_table_body(pa_run_t* results, int m, int w1, int w2, int w3, int w4, int w5, int w6) {

    char* str;
    const char* padding = "--------------------------------------------------------------------------------";

    printf("%c%.*s%c%.*s%c%.*s%c%.*s%c%.*s%c%.*s%c\n", 
        '+',
        w1, padding, '+', 
        w2, padding, '+', 
        w3, padding, '+', 
        w4, padding, '+', 
        w5, padding, '+', 
        w6, padding, '+');

    for (int i = 0; i < 18; i++) {

        str = (m == i) ? " <-- best" : "";

        printf("%-2c%-*d%-2c%-*d%-2c%-*d%-2c%-*d%-2c%-*.*f%-2c%-*.*f%c%s\n", 
            '|', w1-1, results[i].index,
            ' ', w2-1, results[i].blocksize,
            ' ', w3-1, results[i].associativity,
            ' ', w4-1, results[i].branch_pred,
            '|', w5-1, w5-4, results[i].cpi,
            ' ', w6-1, w6-4, results[i].cmr,
            '|', str);

        /*
        +---+---+---+----+--------+--------+
        | 0   0   0   0  | 0.0000   0.0000 |
        | 0   0   0   0  | 0.0000   0.0000 |
        +---+---+---+----+--------+--------+
        */

    }

    printf("%c%.*s%c%.*s%c%.*s%c%.*s%c%.*s%c%.*s%c\n", 
        '+',
        w1, padding, '+', 
        w2, padding, '+', 
        w3, padding, '+', 
        w4, padding, '+', 
        w5, padding, '+', 
        w6, padding, '+');

}

/* pretty prints the performance analysis in a pretty table */
void pretty_print_table(char* title, char menu_sep, pa_run_t* results, int m,
                        char* col1, char* col2, char* col3, char* col4, char* col5, char* col6,
                        int w1, int w2, int w3, int w4, int w5, int w6) {

    pretty_print_table_menu(title, menu_sep,
                            col1, col2, col3, col4, col5, col6,
                            w1,w2,w3,w4,w5,w6);

    pretty_print_table_body(results, m, w1,w2,w3,w4,w5,w6);

}

/* runs the performance analysis testing and prints the results */
void run_pa(FILE* trace_file, pa_run_t* pa_sims, int p1, int p2) {
    // p1 and p2 are the precisions of the cpi and cache miss raterespectively

    char buffer[80];

    int index_inputs    [18] = {7,6,6,6,5,5,5,4,4,  7,6,6,6,5,5,5,4,4};
    int blocksize_inputs[18] = {1,1,2,4,1,2,4,2,4,  1,1,2,4,1,2,4,2,4};
    int assoclvl_inputs [18] = {1,2,1,1,4,2,2,4,4,  1,2,1,1,4,2,2,4,4};
    int brnchpred_inputs[18] = {0,0,0,0,0,0,0,0,0,  1,1,1,1,1,1,1,1,1};

    double cpi_outputs[18];
    double cmr_outputs[18];

    int m = 0;

    for (int i = 0; i < 18; i++) {

        pa_sims[i].index            = index_inputs[i];
        pa_sims[i].blocksize        = blocksize_inputs[i];
        pa_sims[i].associativity    = assoclvl_inputs[i];
        pa_sims[i].branch_pred      = brnchpred_inputs[i];

        branch_predict_taken = brnchpred_inputs[i];
        iplc_sim_init(index_inputs[i], blocksize_inputs[i], assoclvl_inputs[i]);

        while(fgets(buffer, 80, trace_file) != NULL) {

            iplc_sim_parse_instruction(buffer);
            if(dump_pipeline) {
                iplc_sim_dump_pipeline();
            }

        }

        iplc_sim_finalize();

        cpi_outputs[i] = (instruction_count == 0)   ? 0 : (pipeline_cycles / instruction_count);
        cmr_outputs[i] = (cache_access == 0)        ? 0 : (cache_miss / cache_access);

        if (pa_sims[i].cpi + pa_sims[i].cmr < pa_sims[m].cpi + pa_sims[m].cmr) {
            m = i;
        }

        pa_sims[i].cpi = cpi_outputs[i];
        pa_sims[i].cmr = cmr_outputs[i];

    }

    pretty_print_table("Simulation Performance analysis", ':', pa_sims, m,
        "cache size", "block size", "associativity", "branch prediction", "CPI", "cache miss rate",
        3,3,3,4,p1+4,p2+4);

}

/* calcualtes and prints stats in the counts of the parsed instructions */
void calc_inst_stats() {

    char* padding = "------------------------";
    int w1 = 15;
    int w2 = 10;
    int w3 = 12;
    int total_count = inst_stats.rtype + inst_stats.sw + 
                    inst_stats.lw + inst_stats.branch + 
                    inst_stats.jump + inst_stats.syscall + inst_stats.nop;


    printf("\n");
    printf("%s\n", "Instruction Statistics");
    printf("%c%.*s%c%.*s%c%.*s%c\n", '+', w1, padding, '+', w2, padding, '+', w3, padding, '+');
    printf("%c%*s%2c%c%*s%2c%c%*s%2c%c\n", '|', w1-2, "instruction", ' ', '|', w2-2, "count", ' ', '|', w3-2, "percent", ' ', '|');
    printf("%c%.*s%c%.*s%c%.*s%c\n", '+', w1, padding, '+', w2, padding, '+', w3, padding, '+');

    printf("%c%*s%2c%c%*d%2c%c%*.*f%-3c%c\n", '|', w1-2, "rtype"   , ' ', '|', w2-2, inst_stats.rtype  , ' ', '|', w3-3, 3, (double) 100*inst_stats.rtype/total_count  , '%', '|');
    printf("%c%*s%2c%c%*d%2c%c%*.*f%-3c%c\n", '|', w1-2, "sw"      , ' ', '|', w2-2, inst_stats.sw     , ' ', '|', w3-3, 3, (double) 100*inst_stats.sw/total_count     , '%', '|');
    printf("%c%*s%2c%c%*d%2c%c%*.*f%-3c%c\n", '|', w1-2, "lw"      , ' ', '|', w2-2, inst_stats.lw     , ' ', '|', w3-3, 3, (double) 100*inst_stats.lw/total_count     , '%', '|');
    printf("%c%*s%2c%c%*d%2c%c%*.*f%-3c%c\n", '|', w1-2, "branch"  , ' ', '|', w2-2, inst_stats.branch , ' ', '|', w3-3, 3, (double) 100*inst_stats.branch/total_count , '%', '|');
    printf("%c%*s%2c%c%*d%2c%c%*.*f%-3c%c\n", '|', w1-2, "jump"    , ' ', '|', w2-2, inst_stats.jump   , ' ', '|', w3-3, 3, (double) 100*inst_stats.jump/total_count   , '%', '|');
    printf("%c%*s%2c%c%*d%2c%c%*.*f%-3c%c\n", '|', w1-2, "syscall" , ' ', '|', w2-2, inst_stats.syscall, ' ', '|', w3-3, 3, (double) 100*inst_stats.syscall/total_count, '%', '|');
    printf("%c%*s%2c%c%*d%2c%c%*.*f%-3c%c\n", '|', w1-2, "nop"     , ' ', '|', w2-2, inst_stats.nop    , ' ', '|', w3-3, 3, (double) 100*inst_stats.nop/total_count    , '%', '|');

    printf("%c%.*s%c%.*s%c%.*s%c\n", '+', w1, padding, '+', w2, padding, '+', w3, padding, '+');

}

/************************************************************************************************/
/* MAIN Function ********************************************************************************/
/************************************************************************************************/

//*****Main Function*****//
int main(int argc, char* argv[]) {
    // Arguments: [-pa <tracefile>]

    char trace_file_name[1024];
    FILE *trace_file = NULL;
    char buffer[80];
    int index = 10;
    int blocksize = 1;
    int assoc = 1;

    if (argc == 1) {
        // When no other arguments are given, default to asking the user for the input information.

        printf("Please enter the tracefile: ");
        scanf("%s", trace_file_name);
        
        trace_file = fopen(trace_file_name, "r");
        
        if (trace_file == NULL) {
            printf("fopen failed for %s file\n", trace_file_name);
            exit(-1);
        }
        
        printf("Enter Cache Size (index), Blocksize and Level of Assoc \n");
        scanf( "%d %d %d", &index, &blocksize, &assoc );
        
        printf("Enter Branch Prediction: 0 (NOT taken), 1 (TAKEN): ");
        scanf("%d", &branch_predict_taken);
        
        iplc_sim_init(index, blocksize, assoc);
        
        while (fgets(buffer, 80, trace_file) != NULL) {
            iplc_sim_parse_instruction(buffer);
            if (dump_pipeline)
                iplc_sim_dump_pipeline();
        }
        
        iplc_sim_finalize();

    } else {
        /*
        When there are arguemnts, check that they are the correct arguemnts.
        */

        if (argc == 3) {
            if (strcmp(argv[1],"-pa") == 0) {

                /*
                When -pa is specified, run the performance analysis on pre-set input variables.
                The output is then summarized for the simulation.
                */

                trace_file = fopen(argv[2], "r");

                if (trace_file == NULL) {
                    //todo: problems
                } else {
                    // all good
                }

                pa_run_t pa_sims[18];

                run_pa(trace_file, pa_sims, 6, 6);

                calc_inst_stats();
            } else {
                //todo: problems
            }
        } else {
            //todo: problems
        }
    }
    return 0;
}
