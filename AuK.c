
/*-----------------------------------------------------------------
 AuK variables and routines
 ------------------------------------------------------------------*/

#include "xc.h"
#include "AuK.h"

// Used to inform of the internal scheduler period in seconds
float s_tic_period; 

/* Returns the number of seconds between two tics of scheduler timer 
 The tic period is correctly initialized when init_tmr1_int_frequency is
 invoqued */

float tic_period()
{
   return(s_tic_period); 
}



int init_tmr1_int_frequency(float Fcyc, float scheduler_period)
{
    float PR1_count; //Expected timer1 instruction counts to interrupt.
        
    PR1_count = Fcyc * scheduler_period;
    if(PR1_count < 65535.0)
    {
        T1CONbits.TCKPS = 0;
        PR1 = (unsigned int)PR1_count;
    }
    else if((PR1_count/8.0) < 65535.0)
        {
            T1CONbits.TCKPS = 0b01;
            PR1 = (unsigned int)(PR1_count/8);
        }
        else if((PR1_count/64.0) < 65535.0)
            {
                T1CONbits.TCKPS = 0b10;
                PR1 = (unsigned int)(PR1_count/64);
            }
            else if((PR1_count/256.0) < 65535.0)
                {
                    T1CONbits.TCKPS = 0b11;
                    PR1 = (unsigned int)(PR1_count/256);
                }
                else
                    return(-1); //Incompatible data
        
    switch(T1CONbits.TCKPS)
    {
        case 0: s_tic_period = 1.0/(Fcyc / (float)PR1);
                break;
        case 1: s_tic_period = 1.0/((Fcyc / 8.0) / (float)PR1);
                break;
        case 2: s_tic_period = 1.0/((Fcyc / 64.0) / (float)PR1);
                break;
        case 3: s_tic_period = 1.0/((Fcyc / 256.0) / (float)PR1);
                break;
        default: s_tic_period = scheduler_period;
                break;
    }
    
    return(0);    
}

void init_tmr1(void)
{
    TMR1 = 0;    
    
    /* One interrupt each 10 ms if Fcy = Fosc/2 = 59.904 MHz */
    //PR1 = 2340; 
    
    T1CONbits.TSIDL = 1; /* Stop on idle mode */
    T1CONbits.TGATE = 0; /* Internal clock source */
    T1CONbits.TCS = 0;
    //T1CONbits.TCKPS = 0b11; /* 256 prescaler */

    IPC0bits.T1IP = 7; /* Priority level */
    IFS0bits.T1IF = 0; /* Clear interrupt flag */
    /* Disable T1 interrupts. It'll be enabled when idle_task will be invoked 
     to turn on uKernel */
    IEC0bits.T1IE = 0; 

    T1CONbits.TON = 0; /* Start timer */
}

void init_int2(void)
{
    RPINR1 = 0; // INT2 tied to vss
    IFS1bits.INT2IF = 0;
    
    // It could be set here because it is called only when AuK is working.
    IEC1bits.INT2IE = 1;
    
    IPC7bits.INT2IP = 7;
    
}

/* Variables for managin process memory */
/****************************************************************************/
unsigned int mem[max_memory_size_words];  // Word array for Task stacks
int total_mem_free_words;
unsigned int indx_free_mem = 0;
/****************************************************************************/

/* Variables for managing process */
/****************************************************************************/

/* Task states */
enum T_state{running, blocked, ready, free_slot};

/* Time control*/
unsigned long current_tic;

/* Task control block */
struct TCB
{
    int stack_pointer;
    int state;
    int priority;
    unsigned long tic_to_wake_up;
    int delayed;
};

/* Task descriptor.
 Last slot (max_TCB_indx) will be used for Idle task. */
struct TCB TCB_Descriptor[max_TCB_indx + 1];

/* Maneuver variables used in asm code */

unsigned int temp_stack_pointer;
unsigned int temp_W0;

unsigned int task_stack_address;
unsigned int new_task_stack_pointer;
unsigned int task_code_address_page;
unsigned int task_code_address_offset;

unsigned int adr_current_task_stack_pointer;

/* Could be used if it is necessary to return from AuK */
unsigned int system_stack_pointer;


/* Task management variables */
int current_TCB_indx;

/* Asm functions */
extern void asmStackFrame(void);

/* Critical region management to avoid context switching. */
/* Variable to determine whether context switching can occur.*/

int context_switching_forbidden;

/* Variable to know when a context switch must be launched after a critical
 region has been ended*/
int tic_produced_during_critical_region;

/****************************************************************************/



/* Return address of the memory space whith the specified words.
 * If there is no space available to satisfy the request then -1 is returned. */
int GiveMeMemory(int num_words)
{
    if((total_mem_free_words - num_words) < 0)
        return(-1);
    else
        total_mem_free_words -= num_words;
    
    int return_address = (unsigned int)&mem[indx_free_mem];
    
    indx_free_mem += num_words;
    
    return(return_address);
}


/*##########################################################################*/
/*##########################################################################*/
/****************************************************************************
 * Structure and functions for managing process identifiers queues.
 * Processes identifiers are ordered in the queue by priority (highest priority 
 * first) and  within a priority they are ordered by first-in-first-out (FIFO).
 * 
 * The process identifier is the same as the array-index used.
 * 
 * Queues are managed as chainned lists.
 * 
 * The space used for links of the chain is an array of size equal to the 
 * maximum process number. There is no space for idle process information.
 * 
 * ¡¡¡Attention!!! 
 * Queues must be accessed on an exclusive basis. 
 * Whoever uses them must guarantee exclusive access to them.
****************************************************************************/


/* Function that initialises a queue of processe identifiers. 
 * As the buffer to save the chained links of process is a separated 
 * array, it must be supplied as a pointer to correctly initialize
 * the structure.
 */
void init_task_queue(Ttask_queue *queue, int *buffer_for_chain)
{
    queue->task_queue = buffer_for_chain;
    queue->out = max_TCB_indx;
    queue->total_in_queue = 0;
}


//Function to insert a new process identifier in the queue, according to 
//priority and FIFO within the same priority.
void insert_in_task_queue(Ttask_queue *queue, int TCB_indx)
{
    int located, counter;
    int insert_position, prev_link;
    int new_priority;
    
    // Task must be ordered by priority and FIFO within priorities.
        
    new_priority = TCB_Descriptor[TCB_indx].priority;
        
    // If there exists suspended tasks, then, it must be located the
    // insertion point
    if(queue->total_in_queue > 0)
    {
        // Verify first element        
        if(new_priority > TCB_Descriptor[queue->out].priority)
        {
            // Must be the first element
            queue->task_queue[TCB_indx] = queue->out;
            queue->out = TCB_indx;
            queue->total_in_queue++;
        }
        else 
            if (queue->total_in_queue == 1)
            {
                // Must be the second element
                queue->task_queue[queue->out] = TCB_indx;
                queue->task_queue[TCB_indx] = max_TCB_indx;
                queue->total_in_queue++;
            }
            else
            {
                // Total in queue > 1
                located = FALSE;
                prev_link = queue->out;
                insert_position = queue->task_queue[queue->out];
                counter = 2;

                while(!located)
                {
                    if(new_priority > TCB_Descriptor[insert_position].priority)
                    {
                        located = TRUE;
                        queue->task_queue[TCB_indx] = insert_position;
                        queue->task_queue[prev_link] = TCB_indx;
                        queue->total_in_queue++;
                    }
                    else
                    {
                        if(counter == queue->total_in_queue)
                        {
                            located = TRUE;
                            // Insert in last position
                            queue->task_queue[insert_position] = TCB_indx;
                            queue->task_queue[TCB_indx] = max_TCB_indx;
                            queue->total_in_queue++;
                        }
                        else
                        {
                            // Continue looking for insert position
                            prev_link = insert_position;
                            insert_position = 
                                    queue->task_queue[insert_position];
                            counter++;
                        }
                    }
                }
            }   
    }
    else
    {
        // It is the first element to suspend
        queue->out = TCB_indx;
        queue->task_queue[TCB_indx] = max_TCB_indx;
        queue->total_in_queue++;
    }
    
}

//This routine supplies the number of processes in the queue.
int how_many_in_queue(Ttask_queue *queue)
{
    return(queue->total_in_queue);
}

/* This routine returns the first process in the process queue and removes it 
 * from the queue. 
 * It must be ensured before invoking it that there is at least one process 
 * in it. */

int extract_first_task_in_queue(Ttask_queue *queue)
{
    int response;
    
    response = queue->out;    
    queue->out = queue->task_queue[queue->out];
    queue->task_queue[response] = max_TCB_indx;
    queue->total_in_queue--;
    return(response);
}

/* This routine returns the priority of the first process in the queue.
 
 returns -1 if there are no processes in queue. */

int task_queue_first_element_priority(Ttask_queue *queue)
{
    if(queue->total_in_queue > 0)
        return(TCB_Descriptor[queue->out].priority);
    else
        return(-1);
}
/*##########################################################################*/
/*##########################################################################*/


/****************************************************************************/
/****************************************************************************/

/* Queue of processes in "ready" state. Thay are ordered by priorities and 
 * FIFO within priorities. */

int buffer_for_ready_chain[max_TCB_indx];
Ttask_queue tasks_ready;

int buffer_for_semaphores_chains[max_TCB_indx];


void init_task_manager()
{
    int x;
    
    /* Init task control blocks (TCB array) */
    for (x=0; x<= max_TCB_indx; x++)
    {
        TCB_Descriptor[x].stack_pointer = 0;
        TCB_Descriptor[x].state = free_slot;
    }
    
    /* Initialice queue of processes in "ready" state. */
    init_task_queue(&tasks_ready, buffer_for_ready_chain);
    
    /* Initialize context switching management. */
    context_switching_forbidden = FALSE;
    tic_produced_during_critical_region = FALSE;
    
    /* ATTENTION 
     * Avoid interrupt nesting. Priority interrupts are only used to 
     * resolve conflicts between simultaneous pending interrupts. */
    INTCON1bits.NSTDIS = 1;
    
    init_tmr1(); //It begins to work when start_AuK() is invoqued.
    init_int2();
}


int init_AuK(float Fcyc, float scheduler_period)
{
    int x;
    
    
    total_mem_free_words = max_memory_size_words;
            
    x = init_tmr1_int_frequency(Fcyc, scheduler_period);
    
    if(x == 0)
    {
        init_task_manager();
        return(0);
    }
    else
        return( x );
}



int give_me_my_id(void)
{
    return(current_TCB_indx);
}

/* Count tics. This function will be called from T1 timer interrupt routine.
 * It is responsible for waking up dormant processes on a timed basis. */

void count_tic()
{
    int x;
    
    current_tic++;
    
    for(x = 0; x < max_TCB_indx; x++)
    {
        if(TCB_Descriptor[x].delayed)
        {
            if(TCB_Descriptor[x].tic_to_wake_up == current_tic)
            {
                TCB_Descriptor[x].delayed = FALSE;
                TCB_Descriptor[x].state = ready;
                insert_in_task_queue(&tasks_ready, x);
            }
        }
    }
}

/* Set adr_new_task_stack_pointer to the new active task */
void scheduler()
{
    
    if(context_switching_forbidden)
        tic_produced_during_critical_region = TRUE;
    else
    {
        if (current_TCB_indx == max_TCB_indx)
        {
            /* Current process is idle process */
            if(how_many_in_queue(&tasks_ready) > 0)
            {
                /* Put idle process in ready state */
                TCB_Descriptor[current_TCB_indx].state = ready;
                
                current_TCB_indx = extract_first_task_in_queue(&tasks_ready); 
            }
            /* This is not neccessary but it remains as a comment
             * 
             else
                // Only idle process is ready
                current_TCB_indx = max_TCB_indx;
             */ 
        }
        else
        {
            /* Verify if current process is running or 
             * it was blocked in a semaphore */
            if (TCB_Descriptor[current_TCB_indx].state == running)
            {
                /* Test priority of first process in ready queue */
                if (task_queue_first_element_priority(&tasks_ready) > 
                        TCB_Descriptor[current_TCB_indx].priority)
                {
                    /* Context switching is needed */
                    TCB_Descriptor[current_TCB_indx].state = ready;
                    /* Current process is not idle process and it could be
                       selected for running in the future */
                    insert_in_task_queue(&tasks_ready, current_TCB_indx);

                    current_TCB_indx = extract_first_task_in_queue(&tasks_ready);    

                }
                /* else  Current process has higher priority than ready processes,
                        then it must continue working. */
            }
            else
            {
                /* Current process was blocked. Improbable but possible 
                 * if T1interrupt enters before INT2 when a process has just 
                 * blocked on a semaphore.*/
                
                if(how_many_in_queue(&tasks_ready) > 0)
                    current_TCB_indx = extract_first_task_in_queue(&tasks_ready);    
                else
                    // Only idle process is ready
                    current_TCB_indx = max_TCB_indx;
            }
        }
        TCB_Descriptor[current_TCB_indx].state = running;

        adr_current_task_stack_pointer = 
            (unsigned int)&TCB_Descriptor[current_TCB_indx].stack_pointer;
    }
}
    


/*-------------------------------------*/
/* Code_address must contain address of the new task code.
 * Code address_page could be obtained by means of __builtin_tblpage(func)
 * Code address_offset could be obtained by means of __builtin_tbloffset(func)
 * Where "func" is the name of the function which contains the code of 
 * the new task.
 * Mem_needed must contain  the nedded words for the task stack 
 * priority must contain the task priority. The larger the number 
 * the higher the priority. Priority must be in the interval 
 * [0, (max_TCB_indx -1)] */
int create_task(unsigned int code_address_page, 
                unsigned int code_address_offset,
                unsigned int mem_needed,
                unsigned int priority)
{
    int TCB_indx, answer;
    
    /* The allowable priorities are those in the interval 
     * [0, (max_TCB_indx -1)] */
    if((priority >= max_TCB_indx) || (priority < 0))
        return(-2);
    
    /* Find a free TCB */
    TCB_indx = 0;
    while((TCB_Descriptor[TCB_indx].state != free_slot) 
            && TCB_indx < max_TCB_indx)
        TCB_indx++;
    
    if (TCB_indx >= max_TCB_indx) // max_TCB_indx is reserved for idle task
        /* There is no free TCB */
        return(-1);
    
    answer = GiveMeMemory(mem_needed);
    if(answer < 0)
        return(-3);
    task_stack_address = (unsigned int)answer;
    task_code_address_page = code_address_page;
    task_code_address_offset = code_address_offset;
    
    /* Create new task stack frame */
    asmStackFrame();
    
    /* Save new task Stack pointer from asm routine */
    TCB_Descriptor[TCB_indx].stack_pointer = new_task_stack_pointer;
    TCB_Descriptor[TCB_indx].state = ready;
    TCB_Descriptor[TCB_indx].priority = priority;
    
    insert_in_task_queue(&tasks_ready, TCB_indx);
    return(TCB_indx);
}

/* This routine launches AuK and also contains the idle process code. */

void start_AuK(void)
{
    current_TCB_indx = max_TCB_indx; // Idle task
    
    /* Idle task only needs 82 words of stack to correctly save its state
     * when Timer 1 interrupt happens. 100 words are reserved to add a security
     * margin */
    
    TCB_Descriptor[current_TCB_indx].stack_pointer = GiveMeMemory(100);
    TCB_Descriptor[current_TCB_indx].state = running;
    TCB_Descriptor[current_TCB_indx].priority = 0;
    
    /* Change to idle task stack */
    system_stack_pointer = WREG15; /* This could be used to return from AuK */
    WREG15 = TCB_Descriptor[current_TCB_indx].stack_pointer;
    adr_current_task_stack_pointer = 
            (unsigned int) &TCB_Descriptor[current_TCB_indx].stack_pointer;
    
    /* Activate timer 1 interrupt */
    IEC0bits.T1IE = 1; /* Enable T1 interrupts */
    T1CONbits.TON = 1; /* Start timer */
    
    /* Idle task code */
    
    /* Produce the first context change to start the most priority task */
    if((PR1 - TMR1) > 5)
    {
        // Change context to benefit other task
        IFS1bits.INT2IF = 1;
        Nop();
        Nop();
    }
    
    while(1); /* Idle task infinite loop */
    
    /* In case it should be necessary to return from AuK. 
     * 
     * WREG15 = system_stack_pointer;  
     * 
     */
    
}


/* Semaphores (Brinch Hansen inspired but including priorities) */
/*
   This function makes a TEST and SET operation on the file register 
   addressed by its first parameter (W0). Test and set operation is ever
   produced in bit 0 of the file register pointed by the first parameter.
   If this bit was 0 (non locked) the function
   replays with a 0 in its second parameter (W1), otherwise it replays with 
   a 1.
   The result of calling this function is set to 1 (locked in 
   any circumstance) the bit 0 of file register pointed by the first 
   parameter.
    
   ¡¡¡ATTENTION!!!
   TEST and SET operation is executed in only one instruction cycle.
   Therefore, it is uninterrumpible, hence it is impossible for race
   conditions to appear in relation to the bit in which TEST and SET 
   operates.
    
 */
extern void TSlock(int address_of_lock_variable, int address_of_answer);

/* This function writes a 0 on the less significant bit of a file register
   used as a lock with a test a set instruction.
   This action should be interpreted as unlock a resource.
   The file register used as a lock is passed as a parameter (W0).
 */
extern void TSunlock(int address_of_lock_variable);

/* The semaphore is initialized and its counter is set with the value of
 the second parameter. */
void init_semaphore(Tsemaphore *sem, int init_count)
{
    sem->semaphore_counter = init_count;
    /* All semaphores use the same buffer for the chain of blocked process
     because a process can only be blocked in a unique semaphore. */
    init_task_queue(&sem->BlockedTasks, buffer_for_semaphores_chains);
}

/* It decerements the counter of the semaphore. If the counter value was 
 * already zero, the task is blocked and the counter remains at zero. 
 * Tasks blocked in the semaphore are ordered by priority and FIFO 
 * within prioiries. */
void wait(Tsemaphore *sem)
{
    unsigned int answer; // to be used in TSlock
     
    /* This is an unusual use of the "Test and Set" instructions. We take 
     * advantage of it to guarantee an assignment in an instruction cycle 
     * and avoid race conditions. */
    
    TSlock((unsigned int)&context_switching_forbidden, (unsigned int)&answer);
    
    // Executing from here means context switching is forbidden.
    
    if(sem->semaphore_counter > 0)
    {
        sem->semaphore_counter--;
        
        if(tic_produced_during_critical_region)
        {
            tic_produced_during_critical_region = FALSE;
            TSunlock((unsigned int)&context_switching_forbidden);
            
            /* A clock tic was produced during critical region then
             context switch is invoked */
            IFS1bits.INT2IF = 1;
            Nop();
            Nop();
        }
        else
            TSunlock((unsigned int)&context_switching_forbidden);
        
    }
    else
    {        
        /* These two critical instructions must be executed without 
         * the possibility of a context switch while they have not 
         * been completed. */
        
        insert_in_task_queue(&sem->BlockedTasks, current_TCB_indx);        
        TCB_Descriptor[current_TCB_indx].state = blocked;  
        
        TSunlock((unsigned int)&context_switching_forbidden);
        
        // Change context
        IFS1bits.INT2IF = 1;
        Nop();
        Nop();
    }
}


/* This wait variant allows an interrupt to be enabled as the last activity in 
 * its execution. It makes it easier to prevent the driver and interrupt 
 * handler from using a semaphore at the same time. While the driver is 
 * running, interrupts on the associated device must be disabled. When the 
 * driver executes wait_within_driver, this service finishes enabling the 
 * specified interrupt before blocking the driver.*/

void wait_within_driver(Tsemaphore *sem, int *IEC_register, int IEC_bit)
{
    unsigned int answer; // to be used in TSlock
     
    /* This is an unusual use of the "Test and Set" instructions. We take 
     * advantage of it to guarantee an assignment in an instruction cycle 
     * and avoid race conditions. */
    
    TSlock((unsigned int)&context_switching_forbidden, (unsigned int)&answer);
    
    // Executing from here means context switching is forbidden.
    
    if(sem->semaphore_counter > 0)
    {
        sem->semaphore_counter--;
        
        if(tic_produced_during_critical_region)
        {
            tic_produced_during_critical_region = FALSE;
            TSunlock((unsigned int)&context_switching_forbidden);
            
            /* Enable requested interruption */
            *IEC_register |= (1<< IEC_bit);
            
            /* A clock tic was produced during critical region then
             context switch is invoked */
            
            IFS1bits.INT2IF = 1;
            Nop();
            Nop();
        }
        else
        {
            TSunlock((unsigned int)&context_switching_forbidden);
            /* Enable requested interruption */
            *IEC_register |= (1<< IEC_bit);
        }
        
    }
    else
    {        
        /* These two critical instructions must be executed without 
         * the possibility of a context switch while they have not 
         * been completed. */
        
        insert_in_task_queue(&sem->BlockedTasks, current_TCB_indx);        
        TCB_Descriptor[current_TCB_indx].state = blocked;  
        
        TSunlock((unsigned int)&context_switching_forbidden);
        
        /* Enable requested interruption */
        *IEC_register |= (1<< IEC_bit);
        
        // Change context
        IFS1bits.INT2IF = 1;
        Nop();
        Nop();
    }
}

/* If there are no blocked tasks in the semaphore, signal increments the 
 semaphore counter. If there exist almost a blocked Task, signal unblock
 the first task in the semaphore blocked task queue. */
void signal(Tsemaphore *sem)
{
    unsigned int answer; // to be used in RClock
    int priority_of_awakened_task, id_awakened_task;
    
    /* This is an unusual use of the "Test and Set" instructions. We take 
     * advantage of it to guarantee an assignment in an instruction cycle 
     * and avoid race conditions. */
    
    TSlock((unsigned int)&context_switching_forbidden, (unsigned int)&answer);
    // Executing from here means context switching is forbidden.
    
      
    if(sem->semaphore_counter > 0)
    {
        sem->semaphore_counter++;
        
        if(tic_produced_during_critical_region)
        {
            tic_produced_during_critical_region = FALSE;
            TSunlock((unsigned int)&context_switching_forbidden);
            
            /* A clock tic was produced during critical region then
             context switch is invoked */
            IFS1bits.INT2IF = 1;
            Nop();
            Nop();
        }
        else
            TSunlock((unsigned int)&context_switching_forbidden);
    }
    else
    {   //semaphore_counter == 0
        if(how_many_in_queue(&sem->BlockedTasks) == 0)
        {
            sem->semaphore_counter++;
            
            if(tic_produced_during_critical_region)
            {
                tic_produced_during_critical_region = FALSE;
                TSunlock((unsigned int)&context_switching_forbidden);
                
                /* A clock tic was produced during critical region then
                 context switch is invoked */
                IFS1bits.INT2IF = 1;
                Nop();
                Nop();
            }
            else
                TSunlock((unsigned int)&context_switching_forbidden);
        }
        else
        {
            id_awakened_task = extract_first_task_in_queue(&sem->BlockedTasks);
            
            TCB_Descriptor[id_awakened_task].state = ready;
            insert_in_task_queue(&tasks_ready, id_awakened_task);
            priority_of_awakened_task =
                    TCB_Descriptor[id_awakened_task].priority;                        
            
            if(tic_produced_during_critical_region)
            {
                tic_produced_during_critical_region = FALSE;
                TSunlock((unsigned int)&context_switching_forbidden);
                /* A clock tic was produced during critical region then
                 context switch is invoked */
                IFS1bits.INT2IF = 1;
                Nop();
                Nop();
            }
            else            
            {  
                TSunlock((unsigned int)&context_switching_forbidden);
                
                // Check if the awakened process has higher priority 
                // than the current one
                if(priority_of_awakened_task > 
                        TCB_Descriptor[current_TCB_indx].priority)
                {
                    // We trigger context switching to allow the highest 
                    // priority task to be executed
                    IFS1bits.INT2IF = 1;
                    Nop();
                    Nop();
                }
            }
        }
    }
}





/* Mutex */
/* Mutex are objects of exclusive access by tasks, with priority according 
 * to the immediate priority ceiling real time scheduler protocol */

/* Initialize a mutex and assigns to it the priority provided in the 
 * second parameter. */
void mutex_init (Tmutex *new_mutex, int priority)
{
    //new_mutex->sem.semaphore_counter = 1;
    init_semaphore(&new_mutex->sem, 1);
    new_mutex->mutex_priority = priority;
    new_mutex->task_old_priority = 0;
    new_mutex->holder_task = max_TCB_indx; /* Assigned to idle task */
}

/* If the mutex is locked by other task, the calling task is blocked in a 
 * semaphore local to the mutex (according to its current priority).
 * If the mutex is free, then it is blocked and the task adquires de priority 
 * of the mutex if it is higher than its current priority. */
void mutex_lock(Tmutex *mutex)
{
    wait(&mutex->sem);
    
    mutex->holder_task = current_TCB_indx;
    if(mutex->mutex_priority > TCB_Descriptor[current_TCB_indx].priority)
    {
        mutex->task_old_priority = TCB_Descriptor[current_TCB_indx].priority;
        TCB_Descriptor[current_TCB_indx].priority = mutex->mutex_priority;
    }
}

/* The mutex is unlocked and the task adquires its previous priority. 
 * If there exists blocked tasks in the same mutex, there will awaken one 
 * of them according to their priorities and arrival order within priorities. 
 *
 * It is no allowed to unlock a mutex by a task that no has this mutex locked.*/
void mutex_unlock(Tmutex *mutex)
{
    /* Unlock only allowed to the holder task */
    if(mutex->holder_task == current_TCB_indx)
    {
        mutex->holder_task = max_TCB_indx; /* Assigned to idle task */
    
        if(mutex->task_old_priority > 0)
        {
            TCB_Descriptor[current_TCB_indx].priority = mutex->task_old_priority;
            mutex->task_old_priority = 0;
        }

        signal(&mutex->sem);    
    }
}

/* Time services */

/* Returns current tic (see init_tmr1 in code) */
unsigned long clock()
{
    int temp_IPL;
    unsigned long tic;
    
    temp_IPL =  SRbits.IPL;
    
    /* It is necessary to briefly inhibit interruptions because 
     * this service affects time management variables handled by the scheduler. 
     * In principle, if the timer 1 interrupt is activated during the execution 
     * of the service, it will not be lost.*/
    
    /* Scheduler and counting tics critical region */
    SRbits.IPL = 7;
    //IEC0bits.T1IE = 0;
    
    tic = current_tic;
                
    /* Exit critical region */
    SRbits.IPL = temp_IPL;
    //IEC0bits.T1IE = 1; /* Enable T1 interrupts */
        
    return tic;
}

/* Blocks current task until global tic.  */
void delay_until(unsigned long tic)
{    
    int temp_IPL;
       
    
    temp_IPL =  SRbits.IPL;
    
    /* It is necessary to briefly inhibit interruptions because 
     * this service affects time management variables handled by the scheduler. 
     * In principle, if the timer 1 interrupt is activated during the execution 
     * of the service, it will not be lost.*/
    
    /* Scheduler and counting tics critical region */
    SRbits.IPL = 7;
        
    TCB_Descriptor[current_TCB_indx].tic_to_wake_up = tic;
    TCB_Descriptor[current_TCB_indx].delayed = TRUE;
    TCB_Descriptor[current_TCB_indx].state = blocked;
    
    /* Exit critical region */
    SRbits.IPL = temp_IPL;
    
    // Change context
    IFS1bits.INT2IF = 1;
    Nop();
    Nop();
    
}
/*-----------------------------------------------------------------
 End of AuK variables and routines
 ------------------------------------------------------------------*/


/*******************************************************/
/*******************************************************/
/*******************************************************/
/* ¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡¡ ATTENTION !!!!!!!!!!!!!!!!!!!!!!! */

/* This set of routines is provided to facilitate the development 
 * of more abstract tools than those currently provided in the microkernel, 
 * but is not recommended to use them in ordinary systems.*/


/* This routine block current active task. */
void block_me(void)
{
    TCB_Descriptor[current_TCB_indx].state = blocked;
    
    // Change context
    IFS1bits.INT2IF = 1;
    Nop();
    Nop();
}

/* This routine returns whether task_id is blocked. */
int is_blocked(int task_id)
{
    if(TCB_Descriptor[task_id].state == blocked)
        return(TRUE);
    else
        return(FALSE);
}

/* This routine unblock task_id. */
void unblock(int task_id)
{
    TCB_Descriptor[task_id].state = ready;
    insert_in_task_queue(&tasks_ready, task_id);
}

/*******************************************************/
/*******************************************************/
/*******************************************************/
