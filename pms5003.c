#include "pms5003.h"

//
// Definitions
//

#define START_CHAR_1 0x42
#define START_CHAR_2 0x4d

#define COMMAND_READ_IN_PASSIVE_MODE 0xe2
#define COMMAND_CHANGE_MODE 0xe1
#define COMMAND_SLEEP_SET 0xe4

static char *error_strings[] = {
	[PMS5003_ERROR_NONE] = "No error",
	[PMS5003_ERROR_WRITE_SERIAL] = "Error writing serial data",
	[PMS5003_ERROR_UNEXPECTED_RESPONSE] = "Unexpected response",
	[PMS5003_ERROR_INVALID_START_CHAR] = "Invalid start character",
	[PMS5003_ERROR_UNEXPECTED_FRAME_LENGTH] = "Unexpected frame length",
	[PMS5003_ERROR_BAD_CHECKSUM] = "Bad checksum",
	[PMS5003_ERROR_READ_SERIAL] = "Error reading serial data",
	[PMS5003_ERROR_EMPTY_DATA] = "Empty data (sensor not ready?)",
};

//
// Structs
//

__attribute__((packed)) struct pms5003_response_data {
	uint8_t command;
	uint8_t data;
};

__attribute__((packed)) struct pms5003_measurement_data {
	uint8_t pm1_0_cf1_high;
	uint8_t pm1_0_cf1_low;
	uint8_t pm2_5_cf1_high;
	uint8_t pm2_5_cf1_low;
	uint8_t pm10_cf1_high;
	uint8_t pm10_cf1_low;
	uint8_t pm1_0_atm_env_high;
	uint8_t pm1_0_atm_env_low;
	uint8_t pm2_5_atm_env_high;
	uint8_t pm2_5_atm_env_low;
	uint8_t pm10_atm_env_high;
	uint8_t pm10_atm_env_low;
	uint8_t gt_0_3um_high;
	uint8_t gt_0_3um_low;
	uint8_t gt_0_5um_high;
	uint8_t gt_0_5um_low;
	uint8_t gt_1_0um_high;
	uint8_t gt_1_0um_low;
	uint8_t gt_2_5um_high;
	uint8_t gt_2_5um_low;
	uint8_t gt_5_0um_high;
	uint8_t gt_5_0um_low;
	uint8_t gt_10um_high;
	uint8_t gt_10um_low;
	uint8_t reserved_high;
	uint8_t reserved_low;
};

//
// Helper Functions
//

static int send_command(
	uint8_t command,
	uint16_t data, int (*write_serial)(uint8_t)
);

static int send_command(
	uint8_t command,
	uint16_t data, int (*write_serial)(uint8_t)
) {
	uint16_t checksum;

	checksum = 0;

	if(write_serial(START_CHAR_1))
		return PMS5003_ERROR_WRITE_SERIAL;
	checksum += START_CHAR_1;
	if(write_serial(START_CHAR_2))
		return PMS5003_ERROR_WRITE_SERIAL;
	checksum += START_CHAR_2;

	if(write_serial(command))
		return PMS5003_ERROR_WRITE_SERIAL;
	checksum += command;

	if(write_serial(data >> 8))
		return PMS5003_ERROR_WRITE_SERIAL;
	checksum += data >> 8;
	if(write_serial(data & 0xff))
		return PMS5003_ERROR_WRITE_SERIAL;
	checksum += data & 0xff;

	if(write_serial(checksum >> 8))
		return PMS5003_ERROR_WRITE_SERIAL;
	if(write_serial(checksum & 0xff))
		return PMS5003_ERROR_WRITE_SERIAL;

	return PMS5003_ERROR_NONE;
}

static int read_packet_data(
	uint8_t *data,
	uint8_t len,
	int (*read_serial)(uint8_t *)
);

static int read_packet_data(
	uint8_t *data,
	uint8_t len,
	int (*read_serial)(uint8_t *)
) {
	uint8_t c;
	uint16_t checksum, frame_checksum;
	uint16_t frame_length;

	checksum = 0;

	// Start chars
	if(read_serial(&c))
		return PMS5003_ERROR_READ_SERIAL;
	if(c != START_CHAR_1)
		return PMS5003_ERROR_INVALID_START_CHAR;
	checksum += c;
	if(read_serial(&c))
		return PMS5003_ERROR_READ_SERIAL;
	if(c != START_CHAR_2)
		return PMS5003_ERROR_INVALID_START_CHAR;
	checksum += c;

	// Frame Length
	if(read_serial(&c))
		return PMS5003_ERROR_READ_SERIAL;
	frame_length = ((uint16_t)c) << 8;
	checksum += c;
	if(read_serial(&c))
		return PMS5003_ERROR_READ_SERIAL;
	frame_length |= c;
	checksum += c;
	if(frame_length != len + 2)
		return PMS5003_ERROR_UNEXPECTED_FRAME_LENGTH;

	// Data
	for(int i=0 ; i < len ; i++) {
		if(read_serial(&c))
			return PMS5003_ERROR_READ_SERIAL;
		*(data + i) = c;
		checksum += c;
	}

	// Checksum
	if(read_serial(&c))
		return PMS5003_ERROR_READ_SERIAL;
	frame_checksum = ((uint16_t)c)<<8;
	if(read_serial(&c))
		return PMS5003_ERROR_READ_SERIAL;
	frame_checksum |= c;
	if(checksum != frame_checksum)
		return PMS5003_ERROR_BAD_CHECKSUM;

	return PMS5003_ERROR_NONE;
}

static int read_response_data(
	struct pms5003_response_data *response_data,
	int (*read_serial)(uint8_t *)
);

static int read_response_data(
	struct pms5003_response_data *response_data,
	int (*read_serial)(uint8_t *)
) {
	return read_packet_data((uint8_t *)response_data, sizeof(struct pms5003_response_data), read_serial);
}

static int read_measurement_data(
	struct pms5003_measurement_data *measurement_data,
	int (*read_serial)(uint8_t *)
);

static int read_measurement_data(
	struct pms5003_measurement_data *measurement_data,
	int (*read_serial)(uint8_t *)
) {
	return read_packet_data((uint8_t *)measurement_data, sizeof(struct pms5003_measurement_data), read_serial);
}

static void extract_measurement(struct pms5003_measurement_data *measurement_data, struct pms5003_measurement *measurement);

static void extract_measurement(struct pms5003_measurement_data *measurement_data, struct pms5003_measurement *measurement) {
	measurement->pm1_0_cf1 = (uint16_t)measurement_data->pm1_0_cf1_high << 8 | measurement_data->pm1_0_cf1_low;
	measurement->pm2_5_cf1 = (uint16_t)measurement_data->pm2_5_cf1_high << 8 | measurement_data->pm2_5_cf1_low;
	measurement->pm10_cf1 = (uint16_t)measurement_data->pm10_cf1_high << 8 | measurement_data->pm10_cf1_low;
	measurement->pm1_0_atm_env = (uint16_t)measurement_data->pm1_0_atm_env_high << 8 | measurement_data->pm1_0_atm_env_low;
	measurement->pm2_5_atm_env = (uint16_t)measurement_data->pm2_5_atm_env_high << 8 | measurement_data->pm2_5_atm_env_low;
	measurement->pm10_atm_env = (uint16_t)measurement_data->pm10_atm_env_high << 8 | measurement_data->pm10_atm_env_low;
	measurement->gt_0_3um = (uint16_t)measurement_data->gt_0_3um_high << 8 | measurement_data->gt_0_3um_low;
	measurement->gt_0_5um = (uint16_t)measurement_data->gt_0_5um_high << 8 | measurement_data->gt_0_5um_low;
	measurement->gt_1_0um = (uint16_t)measurement_data->gt_1_0um_high << 8 | measurement_data->gt_1_0um_low;
	measurement->gt_2_5um = (uint16_t)measurement_data->gt_2_5um_high << 8 | measurement_data->gt_2_5um_low;
	measurement->gt_5_0um = (uint16_t)measurement_data->gt_5_0um_high << 8 | measurement_data->gt_5_0um_low;
	measurement->gt_10um = (uint16_t)measurement_data->gt_10um_high << 8 | measurement_data->gt_10um_low;
}

//
// Public API
//

enum pms5003_error pms5003_set_data_mode(
	enum pms5003_data_mode data_mode,
	int (*write_serial)(uint8_t),
	int (*read_serial)(uint8_t *)
) {
	int ret;
	struct pms5003_response_data response_data;

	if((ret = send_command(COMMAND_CHANGE_MODE, data_mode, write_serial)))
		return ret;

	if((ret = read_response_data(&response_data, read_serial)))
		return ret;

	if(response_data.command != COMMAND_CHANGE_MODE)
		return PMS5003_ERROR_UNEXPECTED_RESPONSE;

	if(response_data.data != data_mode)
		return PMS5003_ERROR_UNEXPECTED_RESPONSE;

	return PMS5003_ERROR_NONE;
}

enum pms5003_error pms5003_get_passive_measurement(
	struct pms5003_measurement *measurement,
	int (*write_serial)(uint8_t),
	int (*read_serial)(uint8_t *)
) {
	int ret;
	struct pms5003_measurement_data measurement_data;

	if((ret = send_command(COMMAND_READ_IN_PASSIVE_MODE, 0, write_serial)))
		return ret;

	return pms5003_get_active_measurement(measurement, read_serial);
}

enum pms5003_error pms5003_get_active_measurement(
	struct pms5003_measurement *measurement,
	int (*read_serial)(uint8_t *)
) {
	int ret;
	struct pms5003_measurement_data measurement_data;

	if((ret = read_measurement_data(&measurement_data, read_serial)))
		return ret;

	extract_measurement(&measurement_data, measurement);

	// TODO add sleep: new measurements only work after ~35ms

	return PMS5003_ERROR_NONE;
}

enum pms5003_error pms5003_sleep_set(
	enum pms5003_sleep_mode sleep_mode,
	int (*write_serial)(uint8_t),
	int (*read_serial)(uint8_t *)
) {
	int ret;
	struct pms5003_response_data response_data;

	if((ret = send_command(COMMAND_SLEEP_SET, sleep_mode, write_serial)))
		return ret;

	if(sleep_mode == PMS5003_SLEEP) {
		if((ret = read_response_data(&response_data, read_serial)))
			return ret;

		if(response_data.command != COMMAND_SLEEP_SET)
			return PMS5003_ERROR_UNEXPECTED_RESPONSE;

		if(response_data.data != sleep_mode)
			return PMS5003_ERROR_UNEXPECTED_RESPONSE;
	}

	// TODO add sleep:
	// - For PMS5003_SLEEP, it takes ~35ms before it can accept new commands.
	// - For PMS5003_WAKEUP, it takes ~1400ms before it can accept new commands.
	//   - Datasheet says to wait 30s due to fan performance.

	return PMS5003_ERROR_NONE;
}

char *pms5003_strerror(enum pms5003_error pms5003_errno) {
	return error_strings[pms5003_errno];
}