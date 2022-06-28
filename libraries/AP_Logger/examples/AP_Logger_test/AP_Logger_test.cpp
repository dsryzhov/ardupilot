/*
 * Example of AP_Logger library.
 * originally based on code by Jordi MuÒoz and Jose Julio
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <GCS_MAVLink/GCS_Dummy.h>
#include <stdio.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define LOG_TEST_MSG 1
struct PACKED log_Test {
    LOG_PACKET_HEADER;
    uint16_t v1, v2, v3, v4;
    int32_t  l1, l2;
};

static const struct LogStructure log_structure[] = {
    LOG_COMMON_STRUCTURES,
    { LOG_TEST_MSG, sizeof(log_Test),       
      "TEST",
      "HHHHii",
      "V1,V2,V3,V4,L1,L2",
      "------",
      "------"
    }
};



#define NUM_PACKETS 500

static uint16_t log_num;

class AP_LoggerTest {
public:
    void setup();
    void loop();
    void one_hz_print(void);
    void ins_update(void);
    void update_logging1();

private:

    AP_InertialSensor ins;
#if HAL_EXTERNAL_AHRS_ENABLED
    AP_ExternalAHRS eAHRS;
#endif // HAL_EXTERNAL_AHRS_ENABLED

    static const AP_Scheduler::Task scheduler_tasks[];

    AP_Int32 log_bitmask;
    AP_Logger logger{log_bitmask};
    AP_Scheduler scheduler{nullptr};

};

static AP_BoardConfig board_config;
static AP_LoggerTest loggertest;

#define SCHED_TASK(func, _interval_ticks, _max_time_micros, _priority) SCHED_TASK_CLASS(AP_LoggerTest, &loggertest, func, _interval_ticks, _max_time_micros, _priority)

const AP_Scheduler::Task AP_LoggerTest::scheduler_tasks[] = {
    SCHED_TASK(ins_update,             50,   1000, 3),
    SCHED_TASK(one_hz_print,            1,   1000, 6),
//    SCHED_TASK(update_logging1,        1,    1000,  45),
//    SCHED_TASK_CLASS(AP_Logger,           &loggertest.logger,           periodic_tasks, 50,  300, 108),

};

void AP_LoggerTest::update_logging1(void) {
    hal.console->printf("\nWrite vibration");
    //ins.Write_Vibration();    
}

void AP_LoggerTest::one_hz_print(void)
{
    hal.console->printf("one_hz: t=%lu\n", (unsigned long)AP_HAL::millis());
}


/*
  update inertial sensor, reading data 
 */
void AP_LoggerTest::ins_update(void)
{
    //ins_counter++;
   ins.update();
}


void AP_LoggerTest::setup(void)
{
//    board_config.init();
//    ins.init(scheduler.get_loop_rate_hz());
    hal.console->printf("Logger Log Test 1.0\n");
//    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks), (uint32_t)-1);
    log_num = 0;
    

    log_bitmask = (uint32_t)-1;
    logger.Init(log_structure, ARRAY_SIZE(log_structure));
    logger.set_vehicle_armed(true);
    
    logger.Write_Message("AP_Logger Test");

    
#ifdef DEBUG_RATES
    hal.console->printf("| Type | Size | 10Hz(bs) | 25Hz(bs) | 400Hz(Kbs) |\n");
    for (uint16_t i = 0; i < ARRAY_SIZE(log_structure); i++) {
        LogStructure log = log_structure[i];
        hal.console->printf("| %-6s | %3d | %4d | %4d | %2dk |\n", log.name, log.msg_len,
            log.msg_len * 10, log.msg_len * 25, log.msg_len * 400 / 1000);
    }
#endif
    // Test
    hal.scheduler->delay(20);

    // We start to write some info (sequentialy) starting from page 1
    // This is similar to what we will do...
    log_num = logger.find_last_log();
    hal.console->printf("Using log number %u\n", log_num);
    hal.console->printf("Writing to flash... wait...\n");

    uint32_t total_micros = 0;
    uint16_t i;

    for (i = 0; i < NUM_PACKETS; i++) {
        uint32_t start = AP_HAL::micros();
        // note that we use g++ style initialisers to make larger
        // structures easier to follow        
        struct log_Test pkt = {
            LOG_PACKET_HEADER_INIT(LOG_TEST_MSG),
            v1    : (uint16_t)(2000 + i),
            v2    : (uint16_t)(2001 + i),
            v3    : (uint16_t)(2002 + i),
            v4    : (uint16_t)(2003 + i),
            l1    : (int32_t)(i * 5000),
            l2    : (int32_t)(i * 16268)
        };
        logger.WriteBlock(&pkt, sizeof(pkt));
        total_micros += AP_HAL::micros() - start;
        hal.scheduler->delay(20);
    }


    hal.console->printf("Average write time %.1f usec/byte\n", 
                       (double)total_micros/((double)i*sizeof(struct log_Test)));

    uint64_t now = AP_HAL::micros64();
    hal.console->printf("Testing Write\n");
    logger.Write("MARY",
                 "TimeUS,GoodValue",
                 "sm",
                 "F0",
                 "Qf",
                 now,
                 -1.5673);
    hal.console->printf("Testing WriteCritical\n");
    logger.WriteCritical("BOB",
                         "TimeUS,GreatValue",
                         "sm",
                         "F0",
                         "Qf",
                         now,
                         17.3);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    logger.flush();
#endif

    //logger.set_vehicle_armed(false);
    
}

void AP_LoggerTest::loop(void)
{
    //scheduler.loop();
    hal.console->printf("\nTest complete.\n");
    hal.scheduler->delay(20000);
}

/*
  compatibility with old pde style build
 */
void setup(void);
void loop(void);

void setup()
{
    loggertest.setup();
}

void loop()
{
    loggertest.loop();
}

const struct AP_Param::GroupInfo        GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};
GCS_Dummy _gcs;

AP_HAL_MAIN();
