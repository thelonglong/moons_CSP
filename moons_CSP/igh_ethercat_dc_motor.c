/* Copyright 2021 Tronlong Elec. Tech. Co. Ltd. All Rights Reserved. */

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <stdbool.h>
#include <signal.h>
#include <sys/mman.h>
#include <time.h>
#include <sys/resource.h>
#include <errno.h>
#include "ecrt.h"
#include "parameter_parser.h"

/* Application parameters */
#define FREQUENCY 50
#define CLOCK_TO_USE CLOCK_REALTIME
#define MEASURE_TIMING

/****************************************************************************/

#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)

#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
                       (B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

/****************************************************************************/

/* Slave num*/
#define SLAVE_NUM 1

/* Slave */ // lolo_question
#define SLAVE_0_ALIAS 0
#define SLAVE_0_POSITION 0
#define SLAVE_0_VID_PID 0x00000168, 0x0000000b // vendor id product code
// #define SLAVE_1_ALIAS                 0
// #define SLAVE_1_POSITION              1
// #define SLAVE_1_VID_PID               0x00000168, 0x00000002

//
#define MOTOR_INTERPOLATION_TIME_PERIOD 1

/* Opration model */
#define MOTOR_MODEL_PROFILE_POSITION 0x01 /* Profile Velocity Mode */
#define MOTOR_MODEL_CSP 8                 /* Profile Velocity Mode */
#define MOTOR_MODEL_CSV 9
// #define MOTOR_MODEL_CSP 0x08              /* Profile Velocity Mode */
/* Control word */
#define MOTOR_MODEL_CONTROL_WORD_HALT 0x1 << 8

// /* Profile acceleration */
// #define MOTOR_PROFILE_ACCELERATION          (20 * 1000)     /* 20s */

// /* Profile deceleration */
// #define MOTOR_PROFILE_DECELERATION          (20 * 1000)     /* 20s */

// /* Target speed */
// #define MOTOR_TARGET_SPEED_FORWARD          10000           /* uint: 0.1 rpm */
// #define MOTOR_TARGET_SPEED_REVERSE          -10000          /* uint: 0.1 rpm */

/* Profile acceleration */
#define MOTOR_PROFILE_ACCELERATION 50000 /* 20s */

/* Profile deceleration */
#define MOTOR_PROFILE_DECELERATION 50000 /* 20s */

/* Profile Velocity */
#define MOTOR_PROFILE_Velocity 20000 /* 20s */
/* Target speed */
#define MOTOR_TARGET_SPEED_FORWARD 20000  /* uint: 0.1 rpm */
#define MOTOR_TARGET_SPEED_REVERSE -20000 /* uint: 0.1 rpm */
#define MOTOR_CSP_POS 20                  /* uint: 0.1 rpm */
/* Profile Pos */
#define MOTOR_PROFILE_Pos 100000 /* 20s */

//=================================PDO====================================================

/* Offsets for PDO entries */
static struct _SlaveOffset
{ // entry Name
    // RXPDO //TXPDO
    // normal param
    unsigned int ctrl_word;
    unsigned int operation_mode;

    // profile v mode para
    // unsigned int target_v;
    // unsigned int profile_acc;
    // unsigned int profile_decel;
    // unsigned long int v_start;
    // unsigned long int v_stop;

    // profile p add
    // unsigned long int profile_v;
    long int target_pos;

    // CSV
    int target_v;

    // unsigned long int quick_stop_decel;
    // long int quick_stop_code;

    // CSP
    long int current_pos;
    short int Interpolation_Time_period;
        // normal param
        unsigned int err_code;
    unsigned int status_word;
    unsigned int current_modes;
    unsigned long int current_v;
    // unsigned long int di;

    // unsigned int slave_di;
    // unsigned int slave_do;
} slave_offset[SLAVE_NUM];

struct _SlaveInfo
{
    uint32_t alias;
    uint32_t position;
    uint32_t vendor_id;
    uint32_t product_code;
};

struct _SlaveConfig
{
    /* Slave configuration */
    ec_slave_config_t *sc;
    ec_slave_config_state_t sc_state;
    struct _SlaveOffset offset;
    int cur_pos;
    int pos_sum;
};

struct _Domain
{
    ec_domain_t *domain;
    ec_domain_state_t domain_state;
    /* Domain process data */
    uint8_t *domain_pd;
};

const static ec_pdo_entry_reg_t domain_regs[] = {
    // /* Slave 0 */
    // {SLAVE_0_ALIAS, SLAVE_0_POSITION, SLAVE_0_VID_PID, 0x60fe, 0, &slave_offset[0].slave_do},
    // {SLAVE_0_ALIAS, SLAVE_0_POSITION, SLAVE_0_VID_PID, 0x60fd, 0, &slave_offset[0].slave_di},

    /* Slave 0 */ // Rx
    {SLAVE_0_ALIAS, SLAVE_0_POSITION, SLAVE_0_VID_PID, 0x6040, 0, &slave_offset[0].ctrl_word},
    {SLAVE_0_ALIAS, SLAVE_0_POSITION, SLAVE_0_VID_PID, 0x6060, 0, &slave_offset[0].operation_mode},
    // profile v add
    //{SLAVE_0_ALIAS, SLAVE_0_POSITION, SLAVE_0_VID_PID, 0x60ff, 0, &slave_offset[0].target_v}, // lolo edit
    // {SLAVE_0_ALIAS, SLAVE_0_POSITION, SLAVE_0_VID_PID, 0x6083, 0, &slave_offset[0].profile_acc},
    // {SLAVE_0_ALIAS, SLAVE_0_POSITION, SLAVE_0_VID_PID, 0x6084, 0, &slave_offset[0].profile_decel},
    // profile p add
    {SLAVE_0_ALIAS, SLAVE_0_POSITION, SLAVE_0_VID_PID, 0x60B0, 0, &slave_offset[0].target_pos}, // LOLO EDIT
    // {SLAVE_0_ALIAS, SLAVE_0_POSITION, SLAVE_0_VID_PID, 0x6081, 0, &slave_offset[0].profile_v},
    // {SLAVE_0_ALIAS, SLAVE_0_POSITION, SLAVE_0_VID_PID, 0x6085, 0, &slave_offset[0].quick_stop_decel},
    // {SLAVE_0_ALIAS, SLAVE_0_POSITION, SLAVE_0_VID_PID, 0x605A, 0, &slave_offset[0].quick_stop_code},

    // CSP ADD2
    
        {SLAVE_0_ALIAS, SLAVE_0_POSITION, SLAVE_0_VID_PID, 0x60C2, 0, &slave_offset[0].Interpolation_Time_period}, // LOLO EDIT

    // TX
    {SLAVE_0_ALIAS, SLAVE_0_POSITION, SLAVE_0_VID_PID, 0x603F, 0, &slave_offset[0].err_code},
    {SLAVE_0_ALIAS, SLAVE_0_POSITION, SLAVE_0_VID_PID, 0x6041, 0, &slave_offset[0].status_word},
    {SLAVE_0_ALIAS, SLAVE_0_POSITION, SLAVE_0_VID_PID, 0x6061, 0, &slave_offset[0].current_modes},
    // {SLAVE_0_ALIAS, SLAVE_0_POSITION, SLAVE_0_VID_PID, 0x606c, 0, &slave_offset[0].current_v}, // lolo edit
    // {SLAVE_0_ALIAS, SLAVE_0_POSITION, SLAVE_0_VID_PID, 0x60FD, 0, &slave_offset[0].di},        // lolo edit
    // CSP ADD1
    {SLAVE_0_ALIAS, SLAVE_0_POSITION, SLAVE_0_VID_PID, 0x6064, 0, &slave_offset[0].current_pos}, // lolo edit
    // CSV
    //{SLAVE_0_ALIAS, SLAVE_0_POSITION, SLAVE_0_VID_PID, 0x60FF, 0, &slave_offset[0].target_v}, // lolo edit
    //    /* Slave 1 */
    //    {SLAVE_1_ALIAS, SLAVE_1_POSITION, SLAVE_1_VID_PID, 0x6040, 0, &slave_offset[1].ctrl_word},
    //    {SLAVE_1_ALIAS, SLAVE_1_POSITION, SLAVE_1_VID_PID, 0x6060, 0, &slave_offset[1].operation_mode},
    //    {SLAVE_1_ALIAS, SLAVE_1_POSITION, SLAVE_1_VIsD_PID, 0x60FF, 0, &slave_offset[1].target_speed},
    //    {SLAVE_1_ALIAS, SLAVE_1_POSITION, SLAVE_1_VID_PID, 0x6083, 0, &slave_offset[1].acceleration},
    //    {SLAVE_1_ALIAS, SLAVE_1_POSITION, SLAVE_1_VID_PID, 0x6084, 0, &slave_offset[1].deceleration},
    //    {SLAVE_1_ALIAS, SLAVE_1_POSITION, SLAVE_1_VID_PID, 0x6041, 0, &slave_offset[1].status_word},
    //    {SLAVE_1_ALIAS, SLAVE_1_POSITION, SLAVE_1_VID_PID, 0x606B, 0, &slave_offset[1].current_speed},
    {}};

ec_pdo_entry_info_t slave_pdo_entries[] = {
    // like control word ,target SLAVE_0_POSITION/
    {0x6040, 0x00, 16}, // index subIndex bit
    {0x6060, 0x00, 16}, // operation 16!!! inCSP ,Profile V in 8
    // profile v add
    //{0x60ff, 0x00, 32},
    // {0x6083, 0x00, 32},
    // {0x6084, 0x00, 32},
    // profile p add
    {0x60B0, 0x00, 32}, // 16
    // {0x6081, 0x00, 32},
    // {0x6085, 0x00, 32},
    // {0x605A, 0x00, 16},
    {0x60C2, 0x00, 8}, // 16
    // TX
    {0x603F, 0x00, 16},
    {0x6041, 0x00, 16},
    {0x6061, 0x00, 8},
    //{0x606c, 0x00, 32},
    // {0x60fd, 0x00, 32},
    // CSP
    {0x6064, 0x00, 32},
    // CSV
    //{0x60FF, 0x00, 32},
};

ec_pdo_info_t slave_pdos[] = {
    {0x1600, 4, slave_pdo_entries + 0}, // RxPDO_1603 Mapping
    {0x1a00, 4, slave_pdo_entries + 4}, // TxPDO
};

ec_sync_info_t slave_syncs[] = { // index direction /SMConfig
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DEFAULT},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DEFAULT},
    {2, EC_DIR_OUTPUT, 1, slave_pdos + 0, EC_WD_DEFAULT},
    {3, EC_DIR_INPUT, 1, slave_pdos + 1, EC_WD_DEFAULT},
    {0xff}};

struct _SlaveInfo slave_info[] = {
    //    {SLAVE_0_ALIAS, SLAVE_0_POSITION, SLAVE_0_VID_PID},
    //    {SLAVE_1_ALIAS, SLAVE_1_POSITION, SLAVE_1_VID_PID}
    {SLAVE_0_ALIAS, SLAVE_0_POSITION, SLAVE_0_VID_PID}

};
//=================================end====================================================
static unsigned int counter = 0;
static unsigned int blink = 0;
// static unsigned int counter_CSP = 5000;

static unsigned int counter_CSP = 0;

static int counter_CSP_pos = 0;
//
static unsigned int set_point_flag = 0; // lolo
/* Ethercat master */
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};
static unsigned int error_code = 1;
static short int T_interpolation =0;
/* quit flag */
volatile bool g_quit = false;

/* Domain */
struct _Domain domain;

/* Period */
const struct timespec cycletime = {0, PERIOD_NS};

/**
 * Stop servo
 */
void stop_servo()
{
    int i;
    /* Receive process data */
    ecrt_master_receive(master);
    ecrt_domain_process(domain.domain);

    // for (i = 0; i < SLAVE_NUM; i++) {
    EC_WRITE_U16(domain.domain_pd + slave_offset[i].ctrl_word, MOTOR_MODEL_CONTROL_WORD_HALT);
    // }

    /* Send process data */
    /* queues all domain datagrams in the master's datagram queue. */
    ecrt_domain_queue(domain.domain);
    /* Sends all datagrams in the queue. */
    ecrt_master_send(master);
}

/**
 * Check process data state
 */
void check_domain_state(struct _Domain *domain)
{
    ec_domain_state_t ds;
    ecrt_domain_state(domain->domain, &ds);

    if (ds.working_counter != domain->domain_state.working_counter)
    {
        printf("--> check_domain_state: Domain: WC %u.\n", ds.working_counter);
    }

    if (ds.wc_state != domain->domain_state.wc_state)
    {
        printf("--> check_domain_state: Domain: State %u.\n", ds.wc_state);
    }

    domain->domain_state = ds;
}

struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
    struct timespec result;

    if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC)
    {
        result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
    }
    else
    {
        result.tv_sec = time1.tv_sec + time2.tv_sec;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
    }

    return result;
}

/**
 * Check for master state
 */
void check_master_state()
{
    ec_master_state_t ms;
    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding)
    {
        printf("--> check_master_state: %u slave(s).\n", ms.slaves_responding);
    }

    if (ms.al_states != master_state.al_states)
    {
        printf("--> check_master_state: AL states: 0x%02X.\n", ms.al_states);
    }

    if (ms.link_up != master_state.link_up)
    {
        printf("--> check_master_state: Link is %s.\n", ms.link_up ? "up" : "down");
    }

    master_state = ms;
}

/**
 * Check for slave configuration state
 */
void check_slave_config_state(struct _SlaveConfig *slave_config)
{
    ec_slave_config_state_t s;
    int i;

    for (i = 0; i < SLAVE_NUM; i++)
    {
        memset(&s, 0, sizeof(s));

        ecrt_slave_config_state(slave_config[i].sc, &s);

        if (s.al_state != slave_config[i].sc_state.al_state)
        {
            printf("--> check_slave_config_state: slave[%d]: State 0x%02X.\n", i, s.al_state);
        }

        if (s.online != slave_config[i].sc_state.online)
        {
            printf("--> check_slave_config_state: slave[%d]: %s.\n", i, s.online ? "online" : "offline");
        }
        if (s.operational != slave_config[i].sc_state.operational)
        {
            printf("--> check_slave_config_state: slave[%d]: %soperational.\n", i, s.operational ? "" : "Not ");
        }

        slave_config[i].sc_state = s;
    }
}

/**
 * Cycle task
 */
void cyclic_task(struct _SlaveConfig *slave_config, struct _Domain *domain, bool direction)
{
    /* Used to determine the value of the status word */
    uint16_t command[2] = {0x004F, 0x004F, 0x0400};
    uint16_t status;
    uint16_t mod;
    int i;
    struct timespec wakeupTime;
    uint32_t count = 0;
#ifdef MEASURE_TIMING
    struct timespec startTime, endTime, lastStartTime = {};
    uint32_t period_ns = 0, exec_ns = 0, latency_ns = 0,
             latency_min_ns = 0, latency_max_ns = 0,
             period_min_ns = 0, period_max_ns = 0,
             exec_min_ns = 0, exec_max_ns = 0;
#endif

    /* Get current time */
    clock_gettime(CLOCK_TO_USE, &wakeupTime);

    while (!g_quit)
    {
        /* Period time: 1ms */
        wakeupTime = timespec_add(wakeupTime, cycletime);
        clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);

#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &startTime);
        latency_ns = DIFF_NS(wakeupTime, startTime);
        period_ns = DIFF_NS(lastStartTime, startTime);
        exec_ns = DIFF_NS(lastStartTime, endTime);
        lastStartTime = startTime;

        if (latency_ns > latency_max_ns)
        {
            latency_max_ns = latency_ns;
        }
        if (latency_ns < latency_min_ns)
        {
            latency_min_ns = latency_ns;
        }
        if (period_ns > period_max_ns)
        {
            period_max_ns = period_ns;
        }
        if (period_ns < period_min_ns)
        {
            period_min_ns = period_ns;
        }
        if (exec_ns > exec_max_ns)
        {
            exec_max_ns = exec_ns;
        }
        if (exec_ns < exec_min_ns)
        {
            exec_min_ns = exec_ns;
        }
#endif

        /* Receive process data */
        ecrt_master_receive(master);
        ecrt_domain_process(domain->domain);

        /* Check process data state (optional) */
        check_domain_state(domain);

        /* Check for master state (optional) */
        check_master_state();

        /* Check for slave configuration state (optional) */
        check_slave_config_state(slave_config);

#ifdef MEASURE_TIMING
        /* Do not print frequently, or it will affect real-time performance */
        count++;
        if (count == FREQUENCY)
        {
            // output timing stats
            printf("period     min: %10u(ns) ... max: %10u(ns)\n",
                   period_min_ns, period_max_ns);
            printf("exec       min: %10u(ns) ... max: %10u(ns)\n",
                   exec_min_ns, exec_max_ns);
            printf("latency    min: %10u(ns) ... max: %10u(ns)\n",
                   latency_min_ns, latency_max_ns);
            period_max_ns = 0;
            period_min_ns = 0xffffffff;
            exec_max_ns = 0;
            exec_min_ns = 0xffffffff;
            latency_max_ns = 0;
            latency_min_ns = 0xffffffff;

            count = 0;
        }
#endif
        //=============================================================================================================================================
        //================================================使能==

        for (i = SLAVE_NUM - 1; i >= 0; i--)
        {
            /* Read state word */
            status = EC_READ_U16(domain->domain_pd + slave_offset[i].status_word);
            // enable step1
            if ((status & command[i]) == 0x0000)
            { // lolo_question the mean? //4f
                /* Set operation mode to velocity Mode */
                // EC_WRITE_U8(domain->domain_pd + slave_offset[i].operation_mode, MOTOR_MODEL_CSP);  // lolo edit v mode 使能前设置模式比较合理
                EC_WRITE_U16(domain->domain_pd + slave_offset[i].ctrl_word, 0x0006); // lolo add
                // status = EC_READ_U16(domain->domain_pd + slave_offset[i].status_word);
                //  printf("current stautus %d \n",status);
                /* Set acceleration and  deceleration */ // lolo_question set accel ,how many.counts/s^2 ,maybe to small
                //    EC_WRITE_U32(domain->domain_pd + slave_offset[i].acceleration, MOTOR_PROFILE_ACCELERATION);
                //    EC_WRITE_U32(domain->domain_pd + slave_offset[i].deceleration, MOTOR_PROFILE_DECELERATION);
                command[i] = 0x000F;
                printf("step1");
            }
            // halt
            else if ((status & 0x0008) == 0x0008)
            {
                EC_WRITE_U16(domain->domain_pd + slave_offset[i].ctrl_word, 0x0080);
                // printf("current ctrl_word 0x0080");
                status = EC_READ_U16(domain->domain_pd + slave_offset[i].status_word);
                // printf("current stautus %d \n",status);
                command[i] = 0x000F;
                printf("stepX");
            }
            // enable step2
            else if ((status & command[i]) == 0x0001)
            {
                EC_WRITE_U8(domain->domain_pd + slave_offset[i].operation_mode, MOTOR_MODEL_CSP); // lolo edit v mode
                EC_WRITE_U8(domain->domain_pd + slave_offset[i].Interpolation_Time_period, MOTOR_INTERPOLATION_TIME_PERIOD); // lolo edit v mode
//LOLO add interpolation time period

                EC_WRITE_U16(domain->domain_pd + slave_offset[i].ctrl_word, 0x0007);
                // printf("current ctrl_word 0x0007");
                status = EC_READ_U16(domain->domain_pd + slave_offset[i].status_word);
                // printf("current stautus %d \n", status);
                printf("step2");
                command[i] = 0x000F;
            }
            // enable step3
            else if ((status & command[i]) == 0x0003)
            {
                // slave_config[i].cur_pos = EC_READ_U32(domain->domain_pd + slave_offset[i].current_pos);
                // slave_config[i].pos_sum = slave_config[i].cur_pos;
                EC_WRITE_U16(domain->domain_pd + slave_offset[i].ctrl_word, 0x00F); // lolo_change from 0x000F
                // printf("current ctrl_word 0x000F");
                status = EC_READ_U16(domain->domain_pd + slave_offset[i].status_word);
                printf("current stautus %d \n", status);
                printf("step3");
                // EC_WRITE_U8(domain->domain_pd + slave_offset[i].operation_mode, MOTOR_MODEL_CSP); // lolo edit v mode
                command[i] = 0x000F;
                //================================================转动电机===============================================================================
                //    }
            }
            //               /* Read state word */
            //    status = EC_READ_U16(domain->domain_pd + slave_offset[i].status_word);
            //    if ((status & command[i]) == 0x0040) { //lolo_question the mean?
            //        /* Set operation mode to velocity Mode */
            //        EC_WRITE_U8(domain->domain_pd + slave_offset[i].operation_mode, 8);
            //        /* Set acceleration and  deceleration *///lolo_question set accel ,how many.counts/s^2 ,maybe to small
            //     //    EC_WRITE_U32(domain->domain_pd + slave_offset[i].acceleration, MOTOR_PROFILE_ACCELERATION);
            //     //    EC_WRITE_U32(domain->domain_pd + slave_offset[i].deceleration, MOTOR_PROFILE_DECELERATION);
            //        EC_WRITE_U16(domain->domain_pd + slave_offset[i].ctrl_word, 0x0006);
            //        command[i] = 0x006F;
            //    }  else if ((status & 0x0008) == 0x0008) {
            //        EC_WRITE_U16(domain->domain_pd + slave_offset[i].ctrl_word, 0x0080);
            //        command[i] = 0x006F;
            //    } else if ((status & command[i]) == 0x0021) {
            //        EC_WRITE_U16(domain->domain_pd + slave_offset[i].ctrl_word, 0x0007);
            //        command[i] = 0x006F;
            //    } else if ((status & command[i]) == 0x0023) {
            //         // slave_config[i].cur_pos = EC_READ_U32(domain->domain_pd + slave_offset[i].current_pos);
            //         // slave_config[i].pos_sum = slave_config[i].cur_pos;

            //       EC_WRITE_U16(domain->domain_pd + slave_offset[i].ctrl_word, 0x000F);
            //        command[i] = 0x006F;
            //        //================================================转动电机=============================================================================
            //    }

            else if ((status & command[i]) == 0x0007)
            {
                // command[i] = 0x000F;
                // EC_WRITE_U16(domain->domain_pd + slave_offset[i].ctrl_word, 0x00F);               // lolo_change from 0x000F
                // EC_WRITE_U8(domain->domain_pd + slave_offset[i].operation_mode, MOTOR_MODEL_CSP); // lolo edit v mode
                //  Profile p mode
                //  set mode of opearation
                //  set motion params
                //  EC_WRITE_S32(domain->domain_pd + slave_offset[i].target_pos, MOTOR_PROFILE_Pos);
                //  EC_WRITE_S32(domain->domain_pd + slave_offset[i].profile_v, MOTOR_PROFILE_Velocity);
                //  EC_WRITE_S32(domain->domain_pd + slave_offset[i].profile_acc, MOTOR_PROFILE_ACCELERATION);
                //  EC_WRITE_S32(domain->domain_pd + slave_offset[i].profile_decel, MOTOR_PROFILE_DECELERATION);

                //
                // // //profile v mode
                // //set mode of opearation
                // EC_WRITE_U8(domain->domain_pd + slave_offset[i].operation_mode, 3); // lolo edit v mode
                // //set motion param
                // EC_WRITE_S32(domain->domain_pd + slave_offset[i].target_v, MOTOR_PROFILE_Velocity);
                // EC_WRITE_S32(domain->domain_pd + slave_offset[i].Acc, MOTOR_PROFILE_ACCELERATION);
                // EC_WRITE_S32(domain->domain_pd + slave_offset[i].Decel, MOTOR_PROFILE_DECELERATION);

                /* Get Internal Speed */
                // status = EC_READ_U16(domain->domain_pd + slave_offset[i].status_word); /* Get Internal Speed */

                counter++;

                status = EC_READ_U16(domain->domain_pd + slave_offset[i].status_word);
                mod = EC_READ_U8(domain->domain_pd + slave_offset[i].current_modes);
                slave_config[i].cur_pos = EC_READ_U32(domain->domain_pd + slave_offset[i].current_pos); // lolo question try D DOUBLE?
                error_code = EC_READ_U16(domain->domain_pd + slave_offset[i].err_code);
                T_interpolation=EC_READ_U8(domain->domain_pd + slave_offset[i].Interpolation_Time_period); // lolo edit v mode
                if (counter_CSP < 5000) // lolo question if success try !=0
                {
                    counter_CSP_pos = slave_config[i].cur_pos + 500;
                    EC_WRITE_S32(domain->domain_pd + slave_offset[i].target_pos, 5 * counter_CSP); //        /* Set acceleration and  deceleration *///lolo_question set accel ,how many.counts/s^2 ,maybe to small

                    counter_CSP++;
                }
                if (counter == FREQUENCY)
                {
                }
                else
                { // do this at 1 Hz
                    printf("command pos %d \n", counter_CSP_pos);

                    printf("feedb pos %d \n", slave_config[i].cur_pos);
                    printf("now status %d \n", status);
                    // printf("command pos %d/n", slave_config[i].pos_sum);
                    printf("current mod %d  \n", mod);
                    printf("current CSP_COUNTER %u \n", counter_CSP);
                    printf("error : %d \n", error_code);
                    printf("T_interpolation : %u \n", T_interpolation);
                    // // run
                    // EC_WRITE_U16(domain->domain_pd + slave_offset[i].ctrl_word, 0x00F);
                    // EC_WRITE_U8(domain->domain_pd + slave_offset[i].operation_mode, 9);
                    // if(slave_config[i].pos_sum <60000)
                    // {slave_config[i].pos_sum += 200;}

                    // PP MODE only run 1 time
                    // if ((status & command[2] == 0x0400) && mod == 1)
                    // {
                    //     if (set_point_flag == 0)
                    //     {
                    //         printf("the setting profile Pos mod be done");
                    //         EC_WRITE_U16(domain->domain_pd + slave_offset[i].ctrl_word, 0x05F); // new point
                    //         set_point_flag = 1;
                    //     }
                    // }

                    // if (counter)
                    // {
                    //     counter--;
                    // }
                    // else
                    // { // do this at 1 Hz
                    //     counter = FREQUENCY;

                    //     // calculate new process data
                    //     printf("feedb pos %d \n", slave_config[i].cur_pos);
                    //     printf("now status %d \n", status);
                    //     // printf("command pos %d/n", slave_config[i].pos_sum);
                    //     printf("current mod %d  \n", mod);
                    //     printf("current CSP_COUNTER %u \n", counter_CSP);
                    //     printf("error : %d \n", error_code);
                    // }
                }
                // if (counter == 10)
                // {

                // }
                // else
                // { // do this at 1 Hz
                //     counter++;

                // }

                /* Determine whether to turn forward or reverse */
                //    if (direction == 0) {
                //        /* forward */
                //        if (slave_config[i].cur_speed == 0) {
                //            EC_WRITE_S32(domain->domain_pd + slave_offset[i].target_speed, MOTOR_TARGET_SPEED_FORWARD);

                //        } else if(slave_config[i].cur_speed == MOTOR_TARGET_SPEED_FORWARD) {
                //            EC_WRITE_S32(domain->domain_pd + slave_offset[i].target_speed, 0);
                //            printf("now decel /n");
                //        }
                //    } else if (direction == 1) {
                //        /* reverse */
                //        if (slave_config[i].cur_speed == 0) {
                //            EC_WRITE_S32(domain->domain_pd + slave_offset[i].target_speed, MOTOR_TARGET_SPEED_REVERSE);
                //        } else if (slave_config[i].cur_speed == MOTOR_TARGET_SPEED_REVERSE) {
                //            EC_WRITE_S32(domain->domain_pd + slave_offset[i].target_speed, 0);

                //        }
                //    }
            }
        }
        //=============================================================================================================================================
        // if (counter) {
        //     counter--;
        // } else { // do this at 1 Hz
        //     counter = FREQUENCY;

        //     // calculate new process data
        //     blink = !blink;

        // }

        // #if 1
        //     // write process data
        //     EC_WRITE_U32(domain->domain_pd + slave_offset[0].slave_do, blink ? 0x06 : 0x09);
        // #endif

        /* Send process data */
        ecrt_domain_queue(domain->domain);
        ecrt_master_send(master);

#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &endTime);
#endif
    }
}

static void sig_handle(int signal)
{
    g_quit = true;

    stop_servo();
}

int main(int argc, char **argv)
{
    //=========================param init=============================
    int status = 0, ret = -1;
    int i;
    struct _Params params;
    struct _SlaveConfig slave_config[SLAVE_NUM];

    memset(&params, 0, sizeof(params));
    //=========================param parser=============================
    if (parse_parameter(&params, argc, argv) == false)
    {
        printf("Please try --help to see usage.\n");
        exit(2);
    }
    //=========================防止内存交换 ===========================
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    {
        perror("mlockall failed");
        return -1;
    }

    /* Ctrl+c handler */
    signal(SIGINT, sig_handle);

    /* Request EtherCAT master */
    master = ecrt_request_master(0);
    if (!master)
    {
        printf("--> main: Request master failed.\n");
        return -1;
    }
    printf("--> main: Request master success.\n");
    //======================================================
    memset(&slave_config, 0, sizeof(slave_config));
    memset(&domain, 0, sizeof(domain));

    /* Create domain */
    domain.domain = ecrt_master_create_domain(master);
    if (!domain.domain)
    {
        status = -1;
        printf("--> main: Create domain failed.\n");
        goto err_leave;
    }
    printf("--> main: Create domain success.\n");

    /* Get slave configuration */
    for (i = 0; i < SLAVE_NUM; i++)
    {
        slave_config[i].sc = ecrt_master_slave_config(master, slave_info[i].alias,
                                                      slave_info[i].position, slave_info[i].vendor_id,
                                                      slave_info[i].product_code);
        if (!slave_config[i].sc)
        {
            status = -1;
            printf("--> main: Get slave configuration failed.\n");
            goto err_leave;
        }
    }
    printf("--> main: Get slave configuration success.\n");
    //========================================================================================
    /* Configuring PDO */

    //  ecrt_slave_config_pdos('从站配置指针', EC_END, '从站同步管理信息')
    for (i = 0; i < SLAVE_NUM; i++)
    {
        ret = ecrt_slave_config_pdos(slave_config[i].sc, EC_END, slave_syncs);
        if (ret != 0)
        {
            status = -1;
            printf("--> main: Configuration PDO failed.\n");
            goto err_leave;
        }
    }
    printf("--> main: Configuration PDO success.\n");
    //========================================================================================
    /* Registers a bunch of PDO entries for a domain. */
    //.调用 ecrt_domain_reg_pdo_entry_list()，通过输入数据域指针和从站 PDO 入口注册信息，为数据域注册一系列 PDO 入口
    ret = ecrt_domain_reg_pdo_entry_list(domain.domain, domain_regs);
    if (ret != 0)
    {
        status = -1;
        printf("--> main: Failed to register bunch of PDO entries for domain.\n");
        goto err_leave;
    }
    printf("--> main: success to register bunch of PDO entries for domain.\n");

    /* Activate EtherCAT master */
    ret = ecrt_master_activate(master);
    if (ret < 0)
    {
        status = -1;
        printf("--> main: Activate master failed.\n");
        goto err_leave;
    }
    printf("--> main: Activate master success.\n");

    /* Get Pointer to the process data memory */
    domain.domain_pd = ecrt_domain_data(domain.domain);
    if (!domain.domain_pd)
    {
        status = -1;
        printf("--> main: Get pointer to the process data memory failed.\n");
        goto err_leave;
    }
    printf("--> main: Get pointer to the process data memory success.\n");

    /* Prio is a value in the range -20 to 19, set -19 with highest priority. */
    pid_t pid = getpid();
    if (setpriority(PRIO_PROCESS, pid, -19))
        printf("--> main: Warning: Failed to set priority: %s\n", strerror(errno));

    /* Start cyclic function. */
    slave_config[i].pos_sum = 0;
    printf("--> main: Enter cycle task now...\n");
    cyclic_task(slave_config, &domain, params.direction);

err_leave:
    /* Releases EtherCAT master */
    ecrt_release_master(master);
    printf("--> main: Release master.\n");

    return status;
}
