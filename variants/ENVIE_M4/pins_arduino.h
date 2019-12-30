#include "mbed_config.h"

#define SERIAL_HOWMANY		2
#define SERIAL1_TX			SERIAL_TX
#define SERIAL1_RX			SERIAL_RX

#define SERIAL2_TX			PA_15
#define SERIAL2_RX			PF_6
#define SERIAL2_RTS			PF_8
#define SERIAL2_CTS			PF_9

#define SerialHCI			UART2

#define WIRE_HOWMANY		2
#define I2C_SDA1			PH_8
#define I2C_SCL1			PH_7