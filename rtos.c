// Basic RTOS Framework - Spring 2022
// No memory protection, no privilege enforcement
// J Losh

// Student Name:
// TO DO: Add your name on this line.  Do not include your ID number in the file.

// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 6 Pushbuttons and 5 LEDs, UART
// LEDs on these pins:
// Blue:   PF2 (on-board)
// Red:    PA2
// Orange: PA3
// Yellow: PA4
// Green:  PE0
// PBs on these pins
// PB0:    PC4
// PB1:    PC5
// PB2:    PC6
// PB3:    PC7
// PB4:    PD6
// PB5:    PD7
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "wait.h"
#include "clock.h"

// REQUIRED: correct these bitbanding references for the off-board LEDs
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400043FC - 0x40000000) * 32 + 2 * 4))) // off-board
#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC - 0x40000000) * 32 + 3 * 4))) // off-board
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC - 0x40000000) * 32 + 4 * 4))) // off-board
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC - 0x40000000) * 32 + 2 * 4))) // on-board
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC - 0x40000000) * 32 + 0 * 4))) // off-board

// Bit band Alias for push buttons
#define SW0   (*((volatile uint32_t *)(0x42000000 + (0x400063FC - 0x40000000) * 32 + 4 * 4)))
#define SW1   (*((volatile uint32_t *)(0x42000000 + (0x400063FC - 0x40000000) * 32 + 5 * 4)))
#define SW2   (*((volatile uint32_t *)(0x42000000 + (0x400063FC - 0x40000000) * 32 + 6 * 4)))
#define SW3   (*((volatile uint32_t *)(0x42000000 + (0x400063FC - 0x40000000) * 32 + 7 * 4)))
#define SW4   (*((volatile uint32_t *)(0x42000000 + (0x400073FC - 0x40000000) * 32 + 6 * 4)))
#define SW5   (*((volatile uint32_t *)(0x42000000 + (0x400073FC - 0x40000000) * 32 + 7 * 4)))

// LED masks
#define GREEN_LED_MASK  1
#define BLUE_LED_MASK   4
#define RED_LED_MASK    4
#define ORANGE_LED_MASK 8
#define YELLOW_LED_MASK 16

// Push button masks
#define SW0_MASK 16
#define SW1_MASK 32
#define SW2_MASK 64
#define SW3_MASK 128
#define SW4_MASK 64
#define SW5_MASK 128

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 2

typedef struct _semaphore
{
    uint16_t count;
    uint16_t queueSize;
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
    char     name[15];
} semaphore;

semaphore semaphores[MAX_SEMAPHORES];

#define keyPressed 1
#define keyReleased 2
#define flashReq 3
#define resource 4

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore
#define STATE_HOLD       99 // thread has been destroyed but can be restarted

#define ONE_KIB              1024
#define KERNEL_MEM_SPACE     2 * ONE_KIB * sizeof(uint32_t)
#define SRAM_BASE            0x20000000
#define DEBUG                1
#define MAX_TASKS            9       // maximum number of valid tasks
#define XPSR_THUMB_BIT_POS   24
#define SYSTICK_RELOAD_VAL   3999
#define MAX_PRIORITY_LEVELS  8
#define MAX_CHARS            80
#define MAX_FIELDS           8

#define CASE_SVC_YIELD 1
#define CASE_SVC_SLEEP 2
#define CASE_SVC_WAIT  3
#define CASE_SVC_POST  4

uint8_t   taskCurrent = 0;     // index of last dispatched task
uint8_t   taskCount   = 0;     // total number of valid tasks
uint32_t  pidCounter = 0;     // incremented on each thread created


bool preemption_bit;
bool prio_scheduler_bit;

uint32_t* heap_base = (uint32_t*) (SRAM_BASE + KERNEL_MEM_SPACE);

/* ****** PING PONG BUFFER DECLARATION ****** */

uint64_t time_buffer_1[MAX_TASKS];
uint64_t time_buffer_2[MAX_TASKS];

uint64_t* active_buffer;
uint64_t* back_buffer;

bool buffer_ptr;

/* ****** PING PONG BUFFER DECLARATION ****** */




/* ****** PRIORITY SCHEDULER ARRAYS DECLARATION ****** */

typedef struct _task_index
{
    uint8_t task[MAX_TASKS];
    uint8_t size;
    uint8_t next;
} task_index;

task_index task_indexes[MAX_PRIORITY_LEVELS];

/* ****** PRIORITY SCHEDULER ARRAYS DECLARATION ****** */




/* ****** TERMINAL STRUCT DECLARATION ****** */

typedef struct _USER_DATA
{
    char    buffer[MAX_CHARS+1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char    fieldType[MAX_FIELDS];
} USER_DATA;

/* ****** TERMINAL STRUCT DECLARATION ****** */




/* ****** FUNCTION DECLARATIONS ****** */

void     post(int8_t);
void     switch_buffers(void);
void     init_time_arrays(void);
void     start_clock(void);
void     stop_clock(void);
fn       get_hold_task(uint8_t pid);
uint8_t  stringToNum(const char*);
uint32_t getFieldInteger(USER_DATA*, uint8_t);
fn       get_fun_ptr(uint8_t);
char*    getFieldString(USER_DATA*, uint8_t);
uint8_t  get_pid(char*);
char*    reverseNum(uint8_t num);
void     numToString(uint8_t num);
void     display_semaphore_usage(void);
void     init_task_index_arr(void);
void     display_current_processes(void);
void     getsUart0(USER_DATA*);
void     parseFields(USER_DATA*);
bool     is_command(USER_DATA*, const char*, uint8_t);
bool     strCompare(const char*, const char*);
uint8_t  strLength(const char*);
uint8_t  countArguments(USER_DATA* dataPtr, uint8_t);

uint32_t __get_XPSR(void);
uint32_t __get_PSP(void);
uint8_t  __get_svc_num(void);
uint32_t __get_R0_from_PSP(void);

void __soft_pop(void);
void __set_process_stack_ptr(void*);
void __set_asp(void);
void __soft_push(void);
void __set_XPSR(void);
void __set_PC(fn);
void __set_REG(uint32_t);

/* ****** FUNCTION DECLARATIONS ****** */


// REQUIRED: add store and management for the memory used by the thread stacks
//           thread stacks must start on 1 kiB boundaries so mpu can work correctly

struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    uint32_t pid;                  // PID
    fn pFn;                        // function pointer
    void *spInit;                  // original top of stack
    void *sp;                      // current stack pointer
    int8_t priority;               // 0=highest to 7=lowest
    uint32_t ticks;                // ticks until sleep complete
    char name[16];                 // name of task used in ps command
    uint8_t s;                     // index of semaphore that is blocking the thread
} tcb[MAX_TASKS];

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

// REQUIRED: initialize systick for 1ms system timer
void initRtos()
{
    uint8_t i;
    // no tasks running
    taskCount = 0;
    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;
        tcb[i].pFn = 0;
    }
}

int roundRobinScheduler()
{
    bool ok;
    static uint8_t task = 0xFF;
    ok = false;
    while (!ok)
    {
        task++;
        if (task >= MAX_TASKS)
            task = 0;
        ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
    }
    return task;
}

// REQUIRED: Implement prioritization to 8 levels
int rtosScheduler()
{
    uint8_t task;
    uint8_t highest_priority = tcb[0].priority;
    uint8_t i = 1;
    uint8_t next;
    uint8_t task_num;


    /* Find the highest priority of task that can run */
    while(i < MAX_TASKS)
    {
        if(tcb[i].priority < highest_priority)
        {
            if(tcb[i].state == STATE_READY || tcb[i].state == STATE_UNRUN)
            {
                highest_priority = tcb[i].priority;
            }
        }

        i += 1;
    }


    /* To implement a modulo counter */
    if(task_indexes[highest_priority].next >= task_indexes[highest_priority].size)
    {
        task_indexes[highest_priority].next = 0;
    }


    /* Find the task that can run out of the tasks having
    highest priority */
    while(true)
    {
        next = task_indexes[highest_priority].next;
        task_num = task_indexes[highest_priority].task[next];

        if(tcb[task_num].state == STATE_READY || tcb[task_num].state == STATE_UNRUN)
        {
            task = task_num;
            break;
        }

        task_indexes[highest_priority].next += 1;
    }


    /* Increment once more to point to the next task */
    task_indexes[highest_priority].next += 1;

    return task;
}

bool createThread(fn task, const char name[], uint8_t priority, uint32_t stackBytes)
{
    bool ok = false;
    uint8_t i = 0;
    uint8_t j = 0;
    bool found = false;
    char word[100];
    static uint32_t accum_bytes = 0;


    /* Accumulate bytes and add to heap base for stack allocation */
    accum_bytes += stackBytes;

    // REQUIRED:
    // store the thread name
    // allocate stack space and store top of stack in sp and spInit
    // add task if room in task list
    if (taskCount < MAX_TASKS)
    {
        // make sure task not already in list (prevent reentrancy)
        while (!found && (i < MAX_TASKS))
        {
            found = (tcb[i++].pFn == task);
        }
        if (!found)
        {
            // find first available tcb record
            i = 0;
            while (tcb[i].state != STATE_INVALID) {i++;}
            tcb[i].state      = STATE_UNRUN;
            tcb[i].pid        = pidCounter++;
            tcb[i].pFn        = task;
            tcb[i].sp         = (void*) heap_base + accum_bytes;
            tcb[i].spInit     = tcb[i].sp;
            tcb[i].priority   = priority;

            /* Init arrays for priority scheduling */
            task_indexes[priority].task[task_indexes[priority].size] = i;
            task_indexes[priority].size += 1;

            #ifdef DEBUG

            sprintf(word, "\n\rSTACK_TOP = 0x%p, THREAD_NAME = %s\n\r", tcb[i].sp, name);
            putsUart0(word);

            #endif

            /* Store name of task in the task control block */
            while(name[j] != '\0')
            {
                tcb[i].name[j] = name[j];
                j += 1;
            }

            // increment task count
            taskCount++;
            ok = true;
        }
    }
    return ok;
}

// REQUIRED: modify this function to restart a thread
void restartThread(fn task)
{
    uint8_t i = 0;
    static uint8_t pid = MAX_TASKS;

    while(i < MAX_TASKS)
    {
        if(tcb[i].pFn == task && tcb[i].state == STATE_HOLD)
        {
            tcb[i].state = STATE_UNRUN;
            tcb[i].sp = tcb[i].spInit;
            tcb[i].pid = pid;
            pid += 1;
            break;
        }

        i += 1;
    }
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
// NOTE: see notes in class for strategies on whether stack is freed or not
void destroyThread(fn task)
{
    uint8_t i = 0;

    while(i < MAX_TASKS)
    {
        if(tcb[i].pFn == task)
        {
            tcb[i].state = STATE_HOLD;
            break;
        }

        i += 1;
    }


    /* Remove the task from semaphore queue to prevent automatic restart */
    if(semaphores[tcb[i].s].processQueue[0] == i)
    {
        semaphores[tcb[i].s].processQueue[0] = semaphores[tcb[i].s].processQueue[1];
        semaphores[tcb[i].s].queueSize -= 1;
    }
    else if(semaphores[tcb[i].s].processQueue[1] == i)
    {
        semaphores[tcb[i].s].queueSize -= 1;
    }

    /* When task destroyed, shared semaphores may get exhausted. Therefore
    posting back */
    post(tcb[i].s);
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(fn task, uint8_t priority)
{
    uint8_t i = 0;

    while(i < MAX_TASKS)
    {
        if(tcb[i].pFn == task)
        {
            tcb[i].priority = priority;
            break;
        }

        i += 1;
    }
}

bool createSemaphore(uint8_t semaphore, uint8_t count, char* name)
{
    uint8_t i = 0;

    bool ok = (semaphore < MAX_SEMAPHORES);
    {
        while(name[i] != '\0')
        {
            semaphores[semaphore].name[i] = name[i];

            i += 1;
        }

        semaphores[semaphore].count = count;
        semaphores[semaphore].queueSize = 0;
    }
    return ok;
}

// REQUIRED: modify this function to start the operating system
// by calling scheduler, setting PSP, ASP bit, and PC
void startRtos()
{
    /* Init preemption */
    preemption_bit = false;


    /* Start the scheduler as priority based */
    prio_scheduler_bit = true;


    /* Schedule next ready task */
    taskCurrent = rtosScheduler();


    /* Change STATE_UNRUN to STATE_READY */
    tcb[taskCurrent].state = STATE_READY;


    /* Save current PSP stack position to the stack pointer belonging to
    active thread */
    __set_process_stack_ptr(tcb[taskCurrent].sp);


    /* Set ASP bit in CONTROL reg to change SP to PSP; run application code */
    __set_asp();


    /* Get function pointer to active thread and run task */
    fn task = tcb[taskCurrent].pFn;
    task();
}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
void yield()
{
    __asm(" SVC #1");
}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
void sleep(uint32_t tick)
{
    __asm(" SVC #2");
}

// REQUIRED: modify this function to wait a semaphore using pendsv
void wait(int8_t s)
{
    __asm(" SVC #3");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(int8_t s)
{
    __asm(" SVC #4");
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{
    uint8_t task_num = 0;

    /* Always switch buffers in systick */
    switch_buffers();

    while(task_num < MAX_TASKS)
    {
        if(tcb[task_num].state == STATE_DELAYED)
        {
            if(tcb[task_num].ticks > 0)
            {
                tcb[task_num].ticks -= 1;

                if(tcb[task_num].ticks == 0)
                {
                    tcb[task_num].state = STATE_READY;
                }
            }
        }

        task_num += 1;
    }

    if(preemption_bit)
    {
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
    }
}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
    /* ***** SAVE CONTEXT ***** */

        /* Software push of PSP; save context R4-11 */
        __soft_push();


        /* Save current PSP stack position to the stack pointer belonging to
        active thread after saving entire context */
        tcb[taskCurrent].sp = (void*) __get_PSP();

    /* ***** SAVE CONTEXT ***** */


    /* Function to disable the timer and read current timer value */
    stop_clock();


    /* Call scheduler; scheduler returns task which is ready or unrun */
    if(prio_scheduler_bit)
    {
        taskCurrent = rtosScheduler();
    }
    else
    {
        taskCurrent = roundRobinScheduler();
    }


    /* Function to enable timer and init timer TAV_REG with value
    zero each time a task is scheduled*/
    start_clock();


    /* If task already run or in STATE_READY, undo above steps */
    if(tcb[taskCurrent].state == STATE_READY)
    {
        /* Save active thread stack position on PSP*/
        __set_process_stack_ptr(tcb[taskCurrent].sp);


        /* Restore context */
        __soft_pop();
    }
    else
    {
        /* If task is never run or in STATE_UNRUN; scheduler function returns
        tasks that are either ready or unrun */


        /* ***** PREPARE FOR HARDWARE POP ***** */

            /* Set PSP to unrun thread stack top */
            __set_process_stack_ptr(tcb[taskCurrent].sp);


            /* Validate XPSR and push to new stack; THUMB bit should be
            set to support thumb instruction set*/
            uint32_t xpsr_val = __get_XPSR();

            if(!(xpsr_val &= (1 << XPSR_THUMB_BIT_POS)))
            {
                __set_XPSR();


                tcb[taskCurrent].state = STATE_READY;
            }


            /* Push PC to new stack */
            __set_PC(tcb[taskCurrent].pFn);


            /* Save dummy registers */
            __set_REG(0x0000000A);  /* LR */
            __set_REG(0x0000000B);  /* R12 */
            __set_REG(0x0000000C);  /* R3 */
            __set_REG(0x0000000D);  /* R2 */
            __set_REG(0x0000000E);  /* R1 */
            __set_REG(0x0000000F);  /* R0 */


            /* Save current PSP stack position to the stack pointer belonging to
            active thread */
            tcb[taskCurrent].sp = (void*) __get_PSP();

        /* ***** PREPARE FOR HARDWARE POP ***** */
    }
}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{
    /* Get SVC number for processing case statements */
    uint8_t svc_num = __get_svc_num();

    switch(svc_num)
    {
        case CASE_SVC_YIELD: /* Block executed for case yield */
        {
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;

            break;
        }
        case CASE_SVC_SLEEP: /* Block executed for case sleep; timeout is serviced in the SysTickIsr */
        {
            /* Get timeout param from reg R0 which was pushed on PSP on
            entering SVC Handler */
            tcb[taskCurrent].ticks = __get_R0_from_PSP();


            /* Set the active thread as delayed; state is changed in SysTickIsr when
            timeout reaches zero */
            tcb[taskCurrent].state = STATE_DELAYED;

            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;

            break;
        }
        case CASE_SVC_WAIT: /* Block executed for case wait */
        {
            /* Get timeout param from reg R0 which was pushed on PSP on
            entering SVC Handler */
            uint8_t l_semaphore = __get_R0_from_PSP();

            if(semaphores[l_semaphore].count > 0)
            {
                semaphores[l_semaphore].count -= 1;
            }
            else
            {
                tcb[taskCurrent].state = STATE_BLOCKED;


                /* Store blocked task in process queue */
                semaphores[l_semaphore].processQueue[semaphores[l_semaphore].queueSize] = taskCurrent;
                semaphores[l_semaphore].queueSize += 1;


                /* Save semaphore that is blocking the thread */
                tcb[taskCurrent].s = l_semaphore;

                NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
            }

            break;
        }
        case CASE_SVC_POST: /* Block executed for case post */
        {
            /* Get timeout param from reg R0 which was pushed on PSP on
            entering SVC Handler */
            uint8_t l_semaphore = __get_R0_from_PSP();

            semaphores[l_semaphore].count += 1;

            if(semaphores[l_semaphore].queueSize > 0)
            {
                uint8_t ind = 0;


                /* Unblock recently blocked task from process queue */
                tcb[semaphores[l_semaphore].processQueue[0]].state = STATE_READY;


                /* Aging the queue; Task which was blocked is now ready to be run. Hence
                remove it from the wait queue */
                while(ind < semaphores[l_semaphore].queueSize)
                {
                    semaphores[l_semaphore].processQueue[ind] = semaphores[l_semaphore].processQueue[ind + 1];

                    ind += 1;
                }

                semaphores[l_semaphore].queueSize -= 1;
                semaphores[l_semaphore].count -= 1;
            }

            break;
        }
    }
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
// REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
//           6 pushbuttons
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Initialize ports A, F, C, D, E
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0 | SYSCTL_RCGCGPIO_R3 | SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R4 | SYSCTL_RCGCGPIO_R5;
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    _delay_cycles(3);


    /* GPIO Init */
    GPIO_PORTA_DIR_R |= RED_LED_MASK | ORANGE_LED_MASK | YELLOW_LED_MASK;
    GPIO_PORTE_DIR_R |= GREEN_LED_MASK;
    GPIO_PORTF_DIR_R |= BLUE_LED_MASK;

    GPIO_PORTC_DIR_R &= ~(SW0_MASK | SW1_MASK | SW2_MASK | SW3_MASK);
    GPIO_PORTD_DIR_R &= ~(SW4_MASK | SW5_MASK);

    GPIO_PORTA_DR2R_R |= RED_LED_MASK | ORANGE_LED_MASK | YELLOW_LED_MASK;
    GPIO_PORTE_DR2R_R |= GREEN_LED_MASK;
    GPIO_PORTF_DR2R_R |= BLUE_LED_MASK;

    GPIO_PORTC_DR2R_R |= SW0_MASK | SW1_MASK | SW2_MASK | SW3_MASK;
    GPIO_PORTD_DR2R_R |= SW4_MASK | SW5_MASK;

    GPIO_PORTA_DEN_R |= RED_LED_MASK | ORANGE_LED_MASK | YELLOW_LED_MASK;
    GPIO_PORTE_DEN_R |= GREEN_LED_MASK;
    GPIO_PORTF_DEN_R |= BLUE_LED_MASK;

    GPIO_PORTC_DEN_R |= SW0_MASK | SW1_MASK | SW2_MASK | SW3_MASK;

    GPIO_PORTC_PUR_R |= SW0_MASK | SW1_MASK | SW2_MASK | SW3_MASK;
    /* GPIO Init */


    /* Unlock PD7 */
    GPIO_PORTD_LOCK_R = GPIO_LOCK_KEY;
    GPIO_PORTD_CR_R |= SW5_MASK;
    GPIO_PORTD_DEN_R |= SW4_MASK | SW5_MASK;
    GPIO_PORTD_PUR_R |= SW4_MASK | SW5_MASK;
    /* Unlock PD7 */


    /* TIMER INIT */
    TIMER1_CTL_R  &= ~TIMER_CTL_TAEN;                                     // turn-off timer before reconfiguring
    TIMER1_CFG_R   = TIMER_CFG_32_BIT_TIMER;                               // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R  = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TACDIR;          // configure for periodic mode (count up)
    TIMER1_TAILR_R = 40000000;                                           // set load value to 40e6 for 1 Hz interrupt rate
    TIMER1_IMR_R   = TIMER_IMR_TATOIM;                                     // turn-on interrupts
    TIMER1_CTL_R  |= TIMER_CTL_TAEN;                                      // turn-on timer
    NVIC_EN0_R    |= 1 << (INT_TIMER1A-16);                                 // turn-on interrupt 37 (TIMER1A) in NVIC
    /* TIMER INIT */


    /* SYSTICK INIT */
    NVIC_ST_CTRL_R |= NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_ENABLE;
    NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_CLK_SRC;
    NVIC_ST_RELOAD_R = SYSTICK_RELOAD_VAL;
    /* SYSTICK INIT */
}

// REQUIRED: add code to return a value from 0-63 indicating which of 6 PBs are pressed
uint8_t readPbs()
{
    if(SW0 == 0 && SW1 == 0 && SW2 == 0 && SW3 == 0 && SW4 == 0 && SW5 == 0)
    {
        return 63;
    }
    else if(SW0 == 0)
    {
        return 1;
    }
    else if(SW1 == 0)
    {
        return 2;
    }
    else if(SW2 == 0)
    {
        return 4;
    }
    else if(SW3 == 0)
    {
        return 8;
    }
    else if(SW4 == 0)
    {
        return 16;
    }
    else if(SW5 == 0)
    {
        return 32;
    }

    return 0;
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------

void init_ping_pong(void)
{
    buffer_ptr = false;

    active_buffer = time_buffer_1;

    back_buffer = time_buffer_2;
}

void switch_buffers(void)
{
    if(buffer_ptr)
    {
        buffer_ptr = false;

        active_buffer = time_buffer_1;

        back_buffer = time_buffer_2;

    }
    else
    {
        buffer_ptr = true;

        active_buffer = time_buffer_2;

        back_buffer = time_buffer_1;
    }
}

void init_time_arrays(void)
{
    uint8_t i = 0;

    while(i < MAX_TASKS)
    {
        time_buffer_1[i] = 0;
        time_buffer_2[i] = 0;

        i += 1;
    }
}

void stop_clock(void)
{
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;

    active_buffer[taskCurrent] += TIMER1_TAV_R;
}

void start_clock(void)
{
    TIMER1_TAV_R = 0;

    TIMER1_CTL_R |= TIMER_CTL_TAEN;
}

void timer1Isr(void)
{
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
}

fn get_hold_task(uint8_t pid)
{
    uint8_t i = 0;

    while(i < MAX_TASKS)
    {
        if(tcb[i].pid == pid && tcb[i].state == STATE_HOLD)
        {
            return tcb[i].pFn;
        }

        i += 1;
    }

    return 0;
}

uint8_t stringToNum(const char* numString)
{
    uint8_t sizeOfString = 0;
    uint8_t singleNum    = 0;
    uint8_t multiplier   = 1;
    uint8_t finalNum     = 0;

    sizeOfString = strLength(numString);

    while(sizeOfString > 0)
    {
        singleNum     = numString[sizeOfString - 1] - '0';
        finalNum     += multiplier * singleNum;
        multiplier   *= 10;
        sizeOfString -= 1;
    }

    return finalNum;
}

fn get_fun_ptr(uint8_t pid)
{
    uint8_t i = 0;

    while(i < MAX_TASKS)
    {
        if(tcb[i].pid == pid && tcb[i].state != STATE_HOLD)
        {
            return tcb[i].pFn;
        }

        i += 1;
    }

    return 0;
}

uint32_t getFieldInteger(USER_DATA* dataPtr, uint8_t fieldNumber)
{
    if(fieldNumber <= dataPtr -> fieldCount)
    {
            return stringToNum(&(dataPtr -> buffer[dataPtr -> fieldPosition[fieldNumber] + 1]));
    }
    return 0;
}

char* getFieldString(USER_DATA* dataPtr, uint8_t fieldNumber)
{
    if(fieldNumber <= dataPtr -> fieldCount)
    {
        return &(dataPtr -> buffer[dataPtr -> fieldPosition[fieldNumber] + 1]);
    }
    return NULL;
}

uint8_t  get_pid(char* task_name)
{
    uint8_t i = 0;

    while(i < MAX_TASKS)
    {
        if(strCompare(task_name, tcb[i].name))
        {
            return tcb[i].pid;
        }

        i += 1;
    }

    return 0;
}

char* reverseNum(uint8_t num)
{
    uint8_t i = 0;
    uint8_t remainder;
    static char arr[10];

    if(num == 0)
    {
        arr[0] = 0 + '0';
        arr[1] = '\0';

        return arr;
    }

    while (num != 0)
    {
        remainder = num % 10;
        arr[i] = remainder + '0';
        num /= 10;
        i += 1;
    }
    arr[i] = '\0';

    return arr;
}

void numToString(uint8_t num)
{
    char* revArr = reverseNum(num);
    uint8_t size = strLength(revArr);

    while(size > 0)
    {
        putcUart0(revArr[size - 1]);
        size -= 1;
    }
}

void display_semaphore_usage(void)
{
    char word[100];
    uint8_t i = 1;
    uint8_t index;
    uint8_t task;
    uint8_t j;

    sprintf(word, "\r-----------------------------------------------------------\n");
    putsUart0(word);

    sprintf(word, "\rINDEX\t NAME\t\t COUNT\t QUEUE SIZE\t WAIT LIST\n");
    putsUart0(word);

    sprintf(word, "\r-----------------------------------------------------------\n");
    putsUart0(word);

    while(i < MAX_SEMAPHORES)
    {
        if(i == 1)
        {
            index = keyPressed;
        }
        else if(i == 2)
        {
            index = keyReleased;
        }
        else if(i == 3)
        {
            index = flashReq;
        }
        else if(i == 4)
        {
            index = resource;
        }

        sprintf(word, "\r%d\t %s\t %d\t    %d\t\t  ", index, semaphores[i].name, semaphores[i].count, semaphores[i].queueSize);
        putsUart0(word);


        /* Block executed for printing name of the task in queue */
        j = 0;

        while(j < semaphores[i].queueSize)
        {
            task = semaphores[i].processQueue[j];

            sprintf(word, "%s", tcb[task].name);
            putsUart0(word);

            j += 1;
        }

        putcUart0('\n');

        i += 1;
    }
}

uint32_t calculate_total_cpu_time(void)
{
    uint8_t i = 0;

    uint32_t total_time = 0;

    while(i < MAX_TASKS)
    {
        /* Add to total time only if its not in STATE_HOLD */
        if(tcb[i].state != STATE_HOLD)
        {
            total_time += back_buffer[i];
        }

        i += 1;
    }

    return total_time;
}

void display_current_processes(void)
{
    char word[100];
    uint8_t i = 0;
    char state;
    uint64_t total_time;
    uint64_t usage_CPU_per_task;

    sprintf(word, "\r--------------------------------------------------\n");
    putsUart0(word);

    sprintf(word, "\rPID\t PROCESS\t STAT\t %%CPU\t PRIORITY\n");
    putsUart0(word);

    sprintf(word, "\r--------------------------------------------------\n");
    putsUart0(word);


    /* Obtain total running time of tasks */
    total_time = calculate_total_cpu_time();


    while(i < MAX_TASKS)
    {
        if(tcb[i].state == STATE_INVALID)
        {
            state = 'I';
        }
        else if(tcb[i].state == STATE_UNRUN)
        {
            state = 'U';
        }
        else if(tcb[i].state == STATE_READY)
        {
            state = 'R';
        }
        else if(tcb[i].state == STATE_DELAYED)
        {
            state = 'D';
        }
        else if(tcb[i].state == STATE_BLOCKED)
        {
            state = 'B';
        }
        else if(tcb[i].state == STATE_HOLD)
        {
            state = 'H';
        }


        /* Precaution; buffer sometimes retains garbage value */
        if(tcb[i].state == STATE_HOLD)
        {
            back_buffer[i] = 0;
        }

        /* Calculate CPU usage per task */
        usage_CPU_per_task = ((back_buffer[i] * 10000) / total_time);

        sprintf(word, "\r%d\t %-15s %c\t %llu.%02llu%%\t   %d\n", tcb[i].pid, tcb[i].name, state, (usage_CPU_per_task / 100), (usage_CPU_per_task % 100), tcb[i].priority);
        putsUart0(word);

        i += 1;
    }
}

void initiate_system_reset(void)
{
    NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;

    while(true);
}

uint8_t strLength(const char* word)
{
    uint8_t strLen = 0;

    while(word[strLen] != ' ' && word[strLen] != '\0')
    {
        strLen += 1;
    }
    return strLen;
}

bool strCompare(const char* inputParam, const char* word)
{
    uint8_t i                = 0;
    uint8_t inputParamLength = strLength(inputParam);
    uint8_t wordLength       = strLength(word);

    if(inputParamLength != wordLength)
    {
        return 0;
    }

    while(i < wordLength)
    {
        if(word[i] != inputParam[i])
        {
            return 0;
        }
        i += 1;
    }
    return 1;
}

uint8_t countArguments(USER_DATA* dataPtr, uint8_t offset)
{
    uint8_t count = 0;

    while(dataPtr -> fieldType[offset] != '\0')
    {
        if(dataPtr -> fieldType[offset] == 'n' || dataPtr -> fieldType[offset] == 'a')
        {
            count += 1;
        }
        offset += 1;
    }
    return count;
}

bool is_command(USER_DATA* dataPtr, const char* command, uint8_t minArgs)
{
    if(countArguments(dataPtr, 1) != minArgs)
    {
        return 0;
    }

    if(strCompare(&(dataPtr -> buffer[dataPtr -> fieldPosition[0]]), command))
    {
        return 1;
    }

    return 0;
}

void getsUart0(USER_DATA *dataPtr)
{
    uint8_t count = 0;
    char c;

    while(true)
        {
            dataPtr -> buffer[count] = getcUart0();
            c = dataPtr -> buffer[count];

            if((c == 8 || c == 127) && count == 0)
            {
                continue;
            }

            putcUart0(c);

            if(c >= 32 && c <= 122)
            {
                count += 1;
            } else if((c == 8 || c == 127) && count > 0)
            {
                count -= 1;
            } else if(c == 13 || c == 10)
            {
                dataPtr -> buffer[count] = '\0';
                break;
            }

            if(count == MAX_CHARS)
            {
                dataPtr -> buffer[count] = '\0';
                break;
            }
        }
}

void parseFields(USER_DATA* dataPtr)
{
    uint8_t i = 1;
    uint8_t j = 1;

    dataPtr -> fieldCount       = 1;
    dataPtr -> fieldType[0]     = 'a';
    dataPtr -> fieldPosition[0] = 0;

    while(true)
    {
        if(dataPtr -> buffer[i] == '\0' || dataPtr -> fieldCount == MAX_FIELDS - 1)
        {
            dataPtr ->  fieldPosition[j] = i;
            dataPtr ->  fieldType[j] = '\0';
            return;
        }

        if((dataPtr -> buffer[i] < 'A' || dataPtr -> buffer[i] > 'Z') && (dataPtr -> buffer[i] < 'a' || dataPtr -> buffer[i] > 'z') && (dataPtr -> buffer[i] < '0' || dataPtr -> buffer[i] > '9'))
        {
            dataPtr -> buffer[i]         = ' ';
            dataPtr ->  fieldPosition[j] = i;

            if(dataPtr -> buffer[i + 1] >= 'A' && dataPtr -> buffer[i + 1] <= 'z')
            {
                dataPtr -> fieldType[j] = 'a';
                dataPtr -> fieldCount  += 1;
                j += 1;
            }
            else if(dataPtr -> buffer[i + 1] >= '0' && dataPtr -> buffer[i + 1] <= '9')
            {
                dataPtr -> fieldType[j] = 'n';
                dataPtr -> fieldCount  += 1;
                j += 1;
            }
        }
        i += 1;
    }
}

void init_task_index_arr(void)
{
    uint8_t i = 0;

    while(i < MAX_PRIORITY_LEVELS)
    {
        task_indexes[i].size = 0;
        task_indexes[i].next = 0;

        i += 1;
    }
}

/* Routine to retrieve process stack ptr from REG R0*/
uint32_t __get_R0_from_PSP(void)
{
    __asm volatile(" MRS R1, PSP");
    __asm volatile(" LDR R0, [R1]");
}

/* Routine to retrieve SVC number */
uint8_t __get_svc_num(void)
{
    __asm volatile(" MRS R0, PSP");
    __asm volatile(" ADD R0, #24");
    __asm volatile(" LDR R0, [R0]");
    __asm volatile(" SUB R0, R0, #2");
    __asm volatile(" LDRB R0, [R0]");
}

/* Routine to push dummy values at memory locations pointed by PSP; Prepares for
   context switch */
void __set_REG(uint32_t val)
{
    __asm volatile(" MRS R1, PSP");
    __asm volatile(" SUB R1, R1, #4");
    __asm volatile(" MSR PSP, R1");
    __asm volatile(" STR R0, [R1]");
}

/* Routine to push value at (PSP - 8); Important for context switch;
   Hardware pop */
void __set_PC(fn fun_ptr)
{
    __asm volatile(" MRS R1, PSP");
    __asm volatile(" SUB R1, R1, #4");
    __asm volatile(" MSR PSP, R1");
    __asm volatile(" STR R0, [R1]");
}

/* Routine to preserve thumb state exec mode */
void __set_XPSR(void)
{
    __asm volatile(" MOV R0, #0x01000000");
    __asm volatile(" MRS R1, PSP");
    __asm volatile(" SUB R1, R1, #4");
    __asm volatile(" MSR PSP, R1");
    __asm volatile(" STR R0, [R1]");
}

/* Routine to retrieve REG XPSR value from R0 */
uint32_t __get_XPSR(void)
{
    __asm volatile(" MRS R0, XPSR");
}

/* Routine to retrieve REG PSP value from R0 */
uint32_t __get_PSP(void)
{
    __asm volatile(" MRS R0, PSP");
}

/* Routine to perform manual software pop */
void __soft_pop(void)
{
    __asm volatile(" MRS R0, PSP");
    __asm volatile(" LDM R0!, {R4-R11}");
    __asm volatile(" MSR PSP, R0");
}

/* Routine to perform manual software push */
void __soft_push(void)
{
    __asm volatile(" MRS R0, PSP");
    __asm volatile(" STMDB R0!, {R4-R11}");
    __asm volatile(" MSR PSP, R0");
}

/* Routine to increment process stack ptr */
void __increment_process_stack_ptr(uint8_t offset)
{
    __asm volatile(" MRS R1, PSP");
    __asm volatile(" ADD R1, R1, R0");
    __asm volatile(" MSR PSP, R1");
}

/* Routine to set REG PSP according to tcb*/
void __set_process_stack_ptr(void* stack_ptr)
{
    __asm volatile(" MSR PSP, R0");
}

/* Routine to set ASP bit in REG CONTROL */
void __set_asp(void)
{
    __asm volatile(" MOV R0, #2");
    __asm volatile(" MSR CONTROL, R0");
}

// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle_1()
{
    while(true)
    {
        BLUE_LED = 1;
        waitMicrosecond(1000);
        BLUE_LED = 0;
        yield();
    }
}

void idle()
{
    while(true)
    {
        ORANGE_LED = 1;
        waitMicrosecond(1000);
        ORANGE_LED = 0;
        yield();
    }
}

void flash4Hz()
{
    while(true)
    {
        GREEN_LED ^= 1;
        sleep(125);
    }
}

void oneshot()
{
    while(true)
    {
        wait(flashReq);
        YELLOW_LED = 1;
        sleep(1000);
        YELLOW_LED = 0;
    }
}

void partOfLengthyFn()
{
    // represent some lengthy operation
    waitMicrosecond(990);
    // give another process a chance to run
    yield();
}

void lengthyFn()
{
    uint16_t i;
    while(true)
    {
        wait(resource);
        for (i = 0; i < 5000; i++)
        {
            partOfLengthyFn();
        }
        RED_LED ^= 1;
        post(resource);
    }
}

void readKeys()
{
    uint8_t buttons;
    while(true)
    {
        wait(keyReleased);
        buttons = 0;
        while (buttons == 0)
        {
            buttons = readPbs();
            yield();
        }
        post(keyPressed);
        if ((buttons & 1) != 0)
        {
            YELLOW_LED ^= 1;
            RED_LED = 1;
        }
        if ((buttons & 2) != 0)
        {
            post(flashReq);
            RED_LED = 0;
        }
        if ((buttons & 4) != 0)
        {
            restartThread(flash4Hz);
        }
        if ((buttons & 8) != 0)
        {
            destroyThread(flash4Hz);
        }
        if ((buttons & 16) != 0)
        {
            setThreadPriority(lengthyFn, 4);
        }
        yield();
    }
}

void debounce()
{
    uint8_t count;
    while(true)
    {
        wait(keyPressed);
        count = 10;
        while (count != 0)
        {
            sleep(10);
            if (readPbs() == 0)
                count--;
            else
                count = 10;
        }
        post(keyReleased);
    }
}

void uncooperative()
{
    while(true)
    {
        while (readPbs() == 32)
        {
        }
        yield();
    }
}

void important()
{
    while(true)
    {
        wait(resource);
        BLUE_LED = 1;
        sleep(1000);
        BLUE_LED = 0;
        post(resource);
    }
}

// REQUIRED: add processing for the shell commands through the UART here
void shell()
{
    USER_DATA data;
    bool valid = false;
    char word[50];
    char* process_name;
    fn fun_ptr;
    uint8_t pid;

    while (true)
    {
        putsUart0("\r\narjun@tm4c123gxl$ ");
        getsUart0(&data);

        if(data.buffer[0] == '\0')
        {
            continue;
        }

#ifdef DEBUG

        putsUart0("\r\nYou entered-> ");
        putsUart0(data.buffer);

        putsUart0("\r\n");

        parseFields(&data);

        uint8_t i = 0;
        uint8_t j = 0;

        while(j < data.fieldCount) {
            putcUart0(data.fieldType[j]);
            putsUart0("\t");
            while(i <= data.fieldPosition[j + 1]) {
                putcUart0(data.buffer[i]);
                i += 1;
            }
            putsUart0("\r\n");
            j += 1;
        }

#endif

        valid = false;

        if(is_command(&data, "ps", 0))
        {
            /* This command displays all the created threads, their PIDs,
            status and CPU usage*/
            display_current_processes();

            valid = true;
        }
        else if(is_command(&data, "reboot", 0))
        {
            putsUart0("\r\nCommand Accepted. Resetting controller...");
            waitMicrosecond(1000000);

            /* This command resets the M4F processor by writing
            to the NVIC APINT register */
            initiate_system_reset();

            valid = true;
        }
        else if(is_command(&data, "ipcs", 0))
        {
            /* This command displays the details regarding semaphore usage */
            display_semaphore_usage();
            valid = true;
        }
        else if(is_command(&data, "pidof", 1))
        {
            process_name = getFieldString(&data, 1);

            pid = get_pid(process_name);

            sprintf(word, "\r\nPID of %s: %d", process_name, pid);
            putsUart0(word);
            valid = true;
        }
        else if(is_command(&data, "kill", 1))
        {
            /* This command gets the function pointer to process to be destroyed
            based on the PID of the process */
            fun_ptr = get_fun_ptr(getFieldInteger(&data, 1));

            if(fun_ptr != 0)
            {
                destroyThread(fun_ptr);
            }
            else
            {
                putsUart0("\r\nThread does not exist\n");
            }

            valid = true;
        }
        else if(is_command(&data, "run", 1))
        {
            /* This command restarts threads that have been destroyed or
            put in STATE_HOLD */
            process_name = getFieldString(&data, 1);

            pid = get_pid(process_name);

            fun_ptr = get_hold_task(pid);

            if(fun_ptr != 0)
            {
                restartThread(fun_ptr);
            }
            else
            {
                putsUart0("\r\nTask does not exist or already scheduled\n");
            }

            valid = true;
        }
        else if(is_command(&data, "scheduler", 1))
        {
            /* This command switches control between the RoundRobin and
            Priority Scheduler */
            if(strCompare("prio", getFieldString(&data, 1)))
            {
                prio_scheduler_bit = true;
                valid = true;
            }
            else if(strCompare("rr", getFieldString(&data, 1)))
            {
                prio_scheduler_bit = false;
                valid = true;
            }
            else
            {
                valid = false;
            }
        }
        else if(is_command(&data, "preemption", 1))
        {
            /* This command is used to turn on and off the preemption bit */
            if(strCompare("on", getFieldString(&data, 1)))
            {
                preemption_bit = true;
                valid = true;
            }
            else if(strCompare("off", getFieldString(&data, 1)))
            {
                preemption_bit = false;
                valid = true;
            }
            else
            {
                valid = false;
            }
        }


        if(!valid)
        {
            putsUart0("\r\nInvalid command");
        }
        else
        {
            putsUart0("\r\nCommand Accepted");
        }
    }
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    bool ok;
    char word[50];

    // Initialize hardware
    initHw();
    initUart0();
    initRtos();
    init_task_index_arr();
    init_time_arrays();
    init_ping_pong();

    // Power-up flash
    GREEN_LED = 1;
    waitMicrosecond(250000);
    GREEN_LED = 0;
    waitMicrosecond(250000);

    // Initialize semaphores
    createSemaphore(keyPressed, 1, "KEY PRESSED");
    createSemaphore(keyReleased, 0, "KEY RELEASED");
    createSemaphore(flashReq, 5, "FLASH REQUEST");
    createSemaphore(resource, 1, "RESOURCE");

    sprintf(word, "\r--------------------------------------------------\n");
    putsUart0(word);

    sprintf(word, "\rMEMORY_ALLOCATION\n\r");
    putsUart0(word);

    sprintf(word, "\r--------------------------------------------------\n");
    putsUart0(word);

    sprintf(word, "\n\rHEAP_BASE: 0x%p\n\r", heap_base);
    putsUart0(word);

    // Add required idle process at lowest priority
    ok =  createThread(idle, "Idle", 7, 1024);

    // Add other processes
    ok &= createThread(lengthyFn, "LengthyFn", 6, 1024);
    ok &= createThread(flash4Hz, "Flash4Hz", 4, 1024);
    ok &= createThread(oneshot, "OneShot", 2, 1024);
    ok &= createThread(readKeys, "ReadKeys", 6, 1024);
    ok &= createThread(debounce, "Debounce", 6, 1024);
    ok &= createThread(important, "Important", 0, 1024);
    ok &= createThread(uncooperative, "Uncoop", 6, 1024);
    ok &= createThread(shell, "Shell", 6, 4096);

    sprintf(word, "\n\r--------------------------------------------------\n");
    putsUart0(word);

    sprintf(word, "\rEND_DEBUG\r");
    putsUart0(word);

    sprintf(word, "\n\r--------------------------------------------------\n");
    putsUart0(word);

    // Start up RTOS
    if (ok)
        startRtos(); // never returns
    else
        RED_LED = 1;

    return 0;
}
