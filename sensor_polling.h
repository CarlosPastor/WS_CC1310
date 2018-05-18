// 
// Data strucure definitions
//

#include <ti/drivers/I2C.h>

/* Example/Board Header files */
#include "Board.h"

// Guarda las variables de i2c

typedef struct I2C_vars
{

    uint8_t         tx[10];
    uint8_t         rx[10];
    I2C_Handle      i2c;
    I2C_Params      i2cParams;
    I2C_Transaction i2cTransaction;

} I2C_vars;

// Sensor 
typedef struct MCP9808
{
    int16_t temp;
} MCP9808;

typedef struct TSL2591
{
    int16_t CH0;
    int16_t CH1;
    float lux;
} TSL2591;

typedef struct TSL2561
{
    int16_t CH0;
    int16_t CH1;
    float lux;
} TSL2561;

typedef struct TMP007
{
    int16_t temp_IR;
} TMP007;

typedef struct VELM6070
{
    int16_t lux_UV;
} VELM6070;

typedef struct sensor_data
{

    MCP9808 MCP9808;
    TSL2591 TSL2591;
    TSL2561 TSL2561;
    TMP007 TMP007;
    VELM6070 VELM6070;
    
} sensor_data;
