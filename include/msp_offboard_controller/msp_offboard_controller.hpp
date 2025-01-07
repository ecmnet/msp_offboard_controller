
#define OFFBOARD_RATE 25

#define MIN_YAW_FOLLOW_VELOCITY 0.10f

#define MSG_PREFIX "/msp"

#define MSP_CMD_PUB  MSG_PREFIX "/in/vehicle_command"
#define MSP_LOG_PUB  "/msp/in/log_message"
#define MSP_TRJ_PUB  MSG_PREFIX "/in/trajectory_setpoint"

#define MSP_POS_SUB     MSG_PREFIX "/out/vehicle_local_position"
#define MSP_STATUS_SUB  MSG_PREFIX "/out/vehicle_status"
#define MSP_CMD_SUB                "/msp/out/vehicle_command"


// MSP Command defintion
typedef enum MSP_CMD
{
    MSP_CMD_OFFBOARD_SETLOCALPOS=73,
} MSP_CMD;

// Todo: Replace using PX4 includes
typedef enum MAV_SEVERITY
{
   MAV_SEVERITY_EMERGENCY=0, /* System is unusable. This is a "panic" condition. | */
   MAV_SEVERITY_ALERT=1, /* Action should be taken immediately. Indicates error in non-critical systems. | */
   MAV_SEVERITY_CRITICAL=2, /* Action must be taken immediately. Indicates failure in a primary system. | */
   MAV_SEVERITY_ERROR=3, /* Indicates an error in secondary/redundant systems. | */
   MAV_SEVERITY_WARNING=4, /* Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning. | */
   MAV_SEVERITY_NOTICE=5, /* An unusual event has occurred, though not an error condition. This should be investigated for the root cause. | */
   MAV_SEVERITY_INFO=6, /* Normal operational messages. Useful for logging. No action is required for these messages. | */
   MAV_SEVERITY_DEBUG=7, /* Useful non-operational messages that can assist in debugging. These should not occur during normal operation. | */
   MAV_SEVERITY_ENUM_END=8, /*  | */
} MAV_SEVERITY;

