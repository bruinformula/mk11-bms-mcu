#ifndef INC_DATAFRAMES_H_
#define INC_DATAFRAMES_H_

#include <stdbool.h>
#include <stdint.h>

typedef union CAN1_DATAFRAME {
	struct __attribute__((packed)) {
		int16_t pack_current;         // Bytes 0–1, signed *0.1 A
		uint16_t pack_voltage;        // Bytes 2–3, unsigned *0.1 V
		uint8_t pack_soc;             // Byte 4, *0.5 %

		// Byte 5 — Status Flags
		uint8_t mpi2_state     : 1;
		uint8_t mpi3_state     : 1;
		uint8_t reserved       : 1;
		uint8_t mpo2_state     : 1;
		uint8_t mpo3_state     : 1;
		uint8_t mpo4_state     : 1;
		uint8_t mp_enable      : 1;
		uint8_t mpo1_state     : 1;

		// Byte 6 — More Flags
		uint8_t discharge_relay: 1;
		uint8_t charge_relay   : 1;
		uint8_t charger_safety : 1;
		uint8_t mil_state      : 1;
		uint8_t mpi1_state     : 1;
		uint8_t always_on      : 1;
		uint8_t is_ready       : 1;
		uint8_t charging_on	   : 1;

		uint8_t checksum;             // Byte 7
	} data;
	uint8_t array[8];
} CAN1_DATAFRAME;



typedef union CAN2_DATAFRAME {
	struct __attribute__((packed)) {
		uint16_t pack_dcl;            // Bytes 0–1, unsigned
		uint8_t pack_ccl;             // Byte 2
		uint8_t reserved0;            // Byte 3: blank

		int8_t pack_high_temp;        // Byte 4
		int8_t pack_low_temp;         // Byte 5
		uint8_t reserved1;            // Byte 6: blank
		uint8_t checksum;             // Byte 7
	} data;
	uint8_t array[8];
} CAN2_DATAFRAME;



typedef union CAN3_DATAFRAME {
	struct __attribute__((packed)) {
		uint16_t low_cell_voltage;    // Bytes 0–1, *0.0001 V
		uint16_t high_cell_voltage;   // Bytes 2–3, *0.0001 V
		uint8_t reserved0;            // Byte 4: blank
		uint8_t reserved1;            // Byte 5: blank
		uint8_t reserved2;            // Byte 6: blank
		uint8_t checksum;             // Byte 7
	} data;
	uint8_t array[8];
} CAN3_DATAFRAME;


/** CHARGER CAN MESSAGES **/
typedef union CHARGER_MSG_1806e7f4 {
	struct __attribute__((packed)) {
		uint16_t pack_voltage;        // Bytes 0-1, unsigned *0.1 V
		int16_t pack_ccl;             // Byte 2-3, signed *0.1A
		uint8_t charge_enable;            // Byte 4: charge enable, 0 to charge and 1 to stop charging
		uint8_t reserved0;            // Byte 5: blank
		uint8_t reserved1;            // Byte 6: blank
		uint8_t reserved2;             // Byte 7
	} data;
	uint8_t array[8];
} msg_1806e7f4;

typedef union CHARGER_MSG_1806e5f4 {
	struct __attribute__((packed)) {
		uint16_t high_cell_voltage;   // Bytes 2–3, *0.0001 V
		uint16_t pack_ccl;             // Byte 2-3, signed *0.1A
		uint8_t reserved0;            // Byte 4: blank
		uint8_t reserved1;            // Byte 5: blank
		uint8_t reserved2;            // Byte 6: blank
		uint8_t reserved3;             // Byte 7
	} data;
	uint8_t array[8];
} msg_1806e5f4;

typedef union CHARGER_MSG_1806e9f4 {
	struct __attribute__((packed)) {
		uint16_t high_cell_voltage;   // Bytes 2–3, *0.0001 V
		uint16_t pack_ccl;             // Byte 2-3, signed *0.1A
		uint8_t reserved0;            // Byte 4: blank
		uint8_t reserved1;            // Byte 5: blank
		uint8_t reserved2;            // Byte 6: blank
		uint8_t reserved3;             // Byte 7
	} data;
	uint8_t array[8];
} msg_1806e9f4;

typedef union CHARGER_MSG_18ff50e5 {
	struct __attribute__((packed)) {
		uint8_t reserved0;            // Byte 0: blank
		uint8_t reserved1;            // Byte 1: blank
		uint8_t reserved2;            // Byte 2: blank
		uint8_t reserved3;             // Byte 3: blank
		uint8_t reserved4;            // Byte 4: blank
		uint8_t reserved5;            // Byte 5: blank
		uint8_t reserved6;            // Byte 6: blank
		uint8_t reserved7;             // Byte 7: blank
	} data;
	uint8_t array[8];
} msg_18ff50e5;

#endif /* INC_DATAFRAMES_H_ */
