/**
 * @brief This file set generic IP COMMAND and REGISTR offset
 */

// IP Commands values
#define IP_INIT 'i'+0
#define IP_START 'i'+1
#define IP_STATUS 'i'+2

// IP register offset
#define REG_CMD			0x00
#define REG_STATUS    0x01
#define REG_PARAMS    0x04
#define REG_ARRAY_IN  0x20
#define REG_ARRAY_OUT 0x30
