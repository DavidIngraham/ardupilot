

// Command Response Codes
enum response_codes {
  RESPONSE_WRITE_OK =0x00,
  RESPONSE_REALTIME_DATA,
  RESPONSE_PAGE_DATA,
  RESPONSE_CONFIG_ERROR,
  RESPONSE_PAGE10_OK,
  RESPONSE_CAN_DATA,
  // Error Responses
  ERR_UNDERRUN = 0X80,
  ERR_OVERRUN,
  ERR_CRC_FAILURE,
  ERR_UNRECOGNIZED_COMMAND,
  ERR_OUT_OF_RANGE,
  ERR_SERIAL_BUSY,
  ERR_FLASH_LOCKED,
  ERR_SEQ_FAIL_1,
  ERR_SEQ_FAIL_2,
  ERR_CAN_QUEUE_FULL,
  ERR_CAN_TIMEOUT,
  ERR_CAN_FAILURE,
  ERR_PARITY,
  ERR_FRAMING,
  ERR_SERIAL_NOISE,
  ERR_TXMODE_RANGE,
  ERR_UNKNOWN
};

// Engine Status Bitmask
#define ENG_STATUS_READY             (1<<0)
#define ENG_STATUS_CRANKING          (1<<1)
#define ENG_STATUS_WARMUP_ENRICHMENT (1<<2)
#define ENG_STATUS_WARMUP            (1<<3)
#define ENG_STATUS_TPS_ACCEL         (1<<4)
#define ENG_STATUS_DECCEL_MODE       (1<<5)
#define ENG_STATUS_MAP_ACCEL         (1<<6)
#define ENG_STATUS_IDLE_ON           (1<<7)

// Fuel Injector Status Bitmask
#define SQUIRT_FIRING_1              (1<<0)
#define SQUIRT_FIRING_2              (1<<1)
#define SQUIRT_SCHED_1               (1<<2)
#define SQUIRT_NOW_1                 (1<<3)
#define SQUIRT_SCHED_2               (1<<4)
#define SQUIRT_NOW_2                 (1<<5)
