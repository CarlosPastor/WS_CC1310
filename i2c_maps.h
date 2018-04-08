// Sensor maps
//


// MCP9808
#define MCP9808_I2C_ADR             0x18

#define TMP007_ACTUAL_TEMP          0x0005      /* Temperatura instantanea */
#define TMP007_ACTUAL_TEMP_LEN      2           /* Object Temp Result Register */


// TSL2591
#define TSL2591_I2C_ADR             0x29

#define TSL2591_CMMND               0xA0      /* default command */

#define TSL2591_ENABLE      0x00           /* Object Temp Result Register */
#define TSL2591_ENABLE_LEN  1           /* Object Temp Result Register */
#define TSL2591_ENABLE_VALUE  1           /* Object Temp Result Register */

#define TSL2591_CONFIG      0x01           /* Object Temp Result Register */
#define TSL2591_CONFIG_LEN  1           /* Object Temp Result Register */
#define TSL2591_STATUS      0x13           /* Object Temp Result Register */
#define TSL2591_STATUS_LEN  1           /* Object Temp Result Register */

#define TSL2591_C0DATAL     0x14           /* Object Temp Result Register */
#define TSL2591_C0DATAL_LEN 4           /* Object Temp Result Register */
#define TSL2591_C0DATAH     0x15           /* Object Temp Result Register */
#define TSL2591_C0DATAH_LEN 0           /* Object Temp Result Register */
#define TSL2591_C1DATAL     0x16           /* Object Temp Result Register */
#define TSL2591_C1DATAL_LEN 0           /* Object Temp Result Register */
#define TSL2591_C1DATAH     0x17           /* Object Temp Result Register */
#define TSL2591_C1DATAH_LEN 0           /* Object Temp Result Register */


// TSL2561
#define TSL2561_I2C_ADR     0x39

#define TSL2561_CMMND       0x80      /* default command */

#define TSL2561_ENABLE      0x00           /* Object Temp Result Register */
#define TSL2561_ENABLE_LEN  1           /* Object Temp Result Register */
#define TSL2561_ENABLE_VALUE  3           /* Object Temp Result Register */

#define TSL2561_CONFIG      0x01           /* Object Temp Result Register */
#define TSL2561_CONFIG_LEN  1           /* Object Temp Result Register */
#define TSL2561_STATUS      0x13           /* Object Temp Result Register */
#define TSL2561_STATUS_LEN  1           /* Object Temp Result Register */

#define TSL2561_C0DATAL     0x0C           /* Object Temp Result Register */
#define TSL2561_C0DATAL_LEN 4           /* Object Temp Result Register */
#define TSL2561_C0DATAH     0x0D           /* Object Temp Result Register */
#define TSL2561_C0DATAH_LEN 0           /* Object Temp Result Register */
#define TSL2561_C1DATAL     0x0E           /* Object Temp Result Register */
#define TSL2561_C1DATAL_LEN 0           /* Object Temp Result Register */
#define TSL2561_C1DATAH     0x0F           /* Object Temp Result Register */
#define TSL2561_C1DATAH_LEN 0           /* Object Temp Result Register */


// TMP007
#define TMP007_I2C_ADR      0x40

#define TMP007_CMMND        0x80      /* default command */

#define TMP007_TEMP       0x01           /* Object Temp Result Register */
#define TMP007_TEMP_LEN   2           /* Object Temp Result Register */

#define TMP007_CONFIG       0x02           /* Object Temp Result Register */
#define TMP007_CONFIG_LEN   2           /* Object Temp Result Register */

#define TMP007_OBJ_TEMP     0x03        /* object temp result regs */
#define TMP007_OBJ_LEN      2        /* object temp result regs */

#define TMP007_STATUS       0x04           /* Object Temp Result Register */
#define TMP007_STATUS_LEN   2           /* Object Temp Result Register */

#define TMP007_STATUS_ME       0x05           /* Object Temp Result Register */
#define TMP007_STATUS_ME_LEN   2           /* Object Temp Result Register */






// SHT31_D
#define SHT31_D_I2C_ADR             0x44
