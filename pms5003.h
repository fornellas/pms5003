#ifndef PMS5003_H
#define PMS5003_H

#include <inttypes.h>

//
// Defines
//

#define PMS5003_STABLE_MODE_INTERVAL_MS 2300

//
// Enums & Structs
//

enum pms5003_data_mode {
	PMS5003_DATA_MODE_PASSIVE,
	PMS5003_DATA_MODE_ACTIVE,
};

struct pms5003_measurement {
	uint16_t pm1_0_cf1;
	uint16_t pm2_5_cf1;
	uint16_t pm10_cf1;
	uint16_t pm1_0_atm_env;
	uint16_t pm2_5_atm_env;
	uint16_t pm10_atm_env;
	uint16_t gt_0_3um;
	uint16_t gt_0_5um;
	uint16_t gt_1_0um;
	uint16_t gt_2_5um;
	uint16_t gt_5_0um;
	uint16_t gt_10um;
};

enum pms5003_sleep_mode {
	PMS5003_SLEEP,
	PMS5003_WAKEUP,
};

enum pms5003_error {
	PMS5003_ERROR_NONE,
	PMS5003_ERROR_WRITE_SERIAL,
	PMS5003_ERROR_UNEXPECTED_RESPONSE,
	PMS5003_ERROR_INVALID_START_CHAR,
	PMS5003_ERROR_UNEXPECTED_FRAME_LENGTH,
	PMS5003_ERROR_BAD_CHECKSUM,
	PMS5003_ERROR_READ_SERIAL,
	PMS5003_ERROR_EMPTY_DATA,
};

//
// Functions
//

enum pms5003_error pms5003_set_data_mode(
	enum pms5003_data_mode data_mode,
	int (*write_serial)(uint8_t),
	int (*read_serial)(uint8_t *)
);

enum pms5003_error pms5003_get_passive_measurement(
	struct pms5003_measurement *measurement,
	int (*write_serial)(uint8_t),
	int (*read_serial)(uint8_t *)
);

enum pms5003_error pms5003_get_active_measurement(
	struct pms5003_measurement *measurement,
	int (*read_serial)(uint8_t *)
);


enum pms5003_error pms5003_sleep_set(
	enum pms5003_sleep_mode sleep_mode,
	int (*write_serial)(uint8_t),
	int (*read_serial)(uint8_t *)
);

char *pms5003_strerror(enum pms5003_error pms5003_errno);

#endif