//////////////////////////////////
// TI includes                  //
//////////////////////////////////

#include <stdint.h>
#include <stddef.h>
#include <unistd.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/display/Display.h>

/* Example/Board Header files */
#include "Board.h"

#include "sensor_polling.h"

//////////////////////////////////
// TI includes END              //
//////////////////////////////////

// stacksize for the thread
#define TASKSTACKSIZE       640


uint8_t VELM6070_INIT(I2C_vars * i2c, sensor_data * sensor_data);
int16_t VELM6070_READ(I2C_vars * i2c, sensor_data * sensor_data);

uint8_t TSL2561_INIT(I2C_vars * i2c, sensor_data * sensor_data);
int16_t TSL2561_READ(I2C_vars * i2c, sensor_data * sensor_data);

uint8_t TSL2591_INIT(I2C_vars * i2c, sensor_data * sensor_data);
int16_t TSL2591_READ(I2C_vars * i2c, sensor_data * sensor_data);

int16_t MCP9808_READ(I2C_vars * i2c, sensor_data * sensor_data);

//////////////////////////////////
// SENSOR MCP9808              //
//////////////////////////////////

#define MCP9808_I2C_ADR             0x18

#define MCP9808_ACTUAL_TEMP          0x0005      /* Temperatura instantanea */
#define MCP9808_ACTUAL_TEMP_LEN      2           /* Object Temp Result Register */

//////////////////////////////////
// SENSOR TSL2591               //
//////////////////////////////////

#define TSL2591_I2C_ADR             0x29

#define TSL2591_CMMND               0xA0      /* default command */

#define TSL2591_ENABLE      0x00           /* Object Temp Result Register */
#define TSL2591_ENABLE_LEN  1           /* Object Temp Result Register */
#define TSL2591_ENABLE_VALUE  3           /* Object Temp Result Register */

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

//////////////////////////////////
// SENSOR TSL2561               //
//////////////////////////////////
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

//////////////////////////////////
// SENSOR TMP007                //
//////////////////////////////////
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

//////////////////////////////////
// SENSOR VELM6070              //
//////////////////////////////////
#define VELM6070_I2C_CMD_ADR      0x38
#define VELM6070_I2C_CMD_VAL      0x06
#define VELM6070_I2C_D0_ADR       0x38
#define VELM6070_I2C_D1_ADR       0x39


static Display_Handle display;

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{

    I2C_vars I2C_vars;
    sensor_data sensor_data;

    unsigned int    i;
    uint16_t        D0;

    /*
    uint8_t         txBuffer[10];
    uint8_t         rxBuffer[10];
    I2C_Handle      i2c;
    I2C_Params      i2cParams;
    I2C_Transaction i2cTransaction;
    */


    /* Call driver init functions */
    Display_init();
    GPIO_init();
    I2C_init();

    /* Configure the LED pin */
    GPIO_setConfig(Board_GPIO_LED0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    /* Open the HOST display for output */
    display = Display_open(Display_Type_UART, NULL);
    if (display == NULL) {
        while (1);
    }

    /* Turn on user LED */
    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);
    Display_printf(display, 0, 0, "Starting example\n");


    /* Create I2C for usage */
    I2C_Params_init(&(I2C_vars.i2cParams));
    I2C_vars.i2cParams.bitRate = I2C_400kHz;
    I2C_vars.i2c = I2C_open(Board_I2C_TMP, &(I2C_vars.i2cParams));

    if (I2C_vars.i2c == NULL) {
        Display_printf(display, 0, 0, "Error Initializing I2C\n");
        while (1);
    }
    else {
        Display_printf(display, 0, 0, "I2C Initialized!\n");
    }

    //Inicializaciones
    VELM6070_INIT(&I2C_vars, &sensor_data);
    TSL2561_INIT(&I2C_vars, &sensor_data);
    TSL2591_INIT(&I2C_vars, &sensor_data);

    /* Take 50 samples and print them out onto the console */
    for (i = 0; i < 50; i++) {

        D0 = VELM6070_READ(&I2C_vars, &sensor_data);
        Display_printf(display, 0, 0, "VELM6070 UV: %d (C)\n", D0);

        D0 = TSL2561_READ(&I2C_vars, &sensor_data);
        Display_printf(display, 0, 0, "TSL2561 IR+VS: %d (C)\n", D0);

        D0 = TSL2591_READ(&I2C_vars, &sensor_data);
        Display_printf(display, 0, 0, "TSL2591 IR+VS: %d (C)\n", D0);

        D0 = MCP9808_READ(&I2C_vars, &sensor_data);
        Display_printf(display, 0, 0, "MCP9808 TEMPERATURE: %d (C)\n", D0);

        /* Sleep for 1 second */
        sleep(1);
    }

    /* Deinitialized I2C */
    I2C_close(I2C_vars.i2c);
    Display_printf(display, 0, 0, "I2C closed!\n");

    return (NULL);

}


uint8_t VELM6070_INIT(I2C_vars * i2c, sensor_data * sensor_data)
{

    i2c->tx[0] = VELM6070_I2C_CMD_VAL;
    i2c->i2cTransaction.slaveAddress = VELM6070_I2C_CMD_ADR;
    i2c->i2cTransaction.writeBuf = i2c->tx;
    i2c->i2cTransaction.writeCount = 1;
    i2c->i2cTransaction.readBuf = i2c->rx;
    i2c->i2cTransaction.readCount = 0;

    if (I2C_transfer(i2c->i2c, &i2c->i2cTransaction))
        Display_printf(display, 0, 0, "I2C Config OK\n");
    else
        Display_printf(display, 0, 0, "I2C Bus fault\n");

    return (NULL);

}

int16_t VELM6070_READ(I2C_vars * i2c, sensor_data * sensor_data)
{
    volatile int16_t temp;

    i2c->i2cTransaction.slaveAddress = VELM6070_I2C_D0_ADR;
    i2c->i2cTransaction.writeBuf = i2c->tx;
    i2c->i2cTransaction.writeCount = 0;
    i2c->i2cTransaction.readBuf = i2c->rx;
    i2c->i2cTransaction.readCount = 1;

    if (I2C_transfer(i2c->i2c, &i2c->i2cTransaction))
        temp = i2c->rx[0];
    else
    {
        Display_printf(display, 0, 0, "I2C Bus fault\n");
        return (NULL);
    }

    // desplazamiento al bit de arriba
    temp = temp << 8;

    i2c->i2cTransaction.slaveAddress = VELM6070_I2C_D1_ADR;
    i2c->i2cTransaction.writeBuf = i2c->tx;
    i2c->i2cTransaction.writeCount = 0;
    i2c->i2cTransaction.readBuf = i2c->rx;
    i2c->i2cTransaction.readCount = 1;

    if (I2C_transfer(i2c->i2c, &i2c->i2cTransaction))
    {
        temp |= i2c->rx[0];
        sensor_data->VELM6070.lux_UV = temp;
        return (NULL);
    }
    else
    {
        Display_printf(display, 0, 0, "I2C Bus fault\n");
        return (NULL);
    }


}

uint8_t TSL2561_INIT(I2C_vars * i2c, sensor_data * sensor_data)
{

    //TSL2561//////////////////////////////////////////////////////////////////////////////////////////

    i2c->tx[0] = TSL2561_CMMND + TSL2561_ENABLE;
    i2c->tx[1] = TSL2561_ENABLE_VALUE;
    i2c->i2cTransaction.slaveAddress = TSL2561_I2C_ADR;
    i2c->i2cTransaction.writeBuf = i2c->tx;
    i2c->i2cTransaction.writeCount = 1 + TSL2561_ENABLE_LEN;
    i2c->i2cTransaction.readBuf = i2c->rx;
    i2c->i2cTransaction.readCount = 0;

    if (I2C_transfer(i2c->i2c, &i2c->i2cTransaction))
        Display_printf(display, 0, 0, "I2C Config OK\n");
    else
        Display_printf(display, 0, 0, "I2C Bus fault\n");

    return (NULL);

}

int16_t TSL2561_READ(I2C_vars * i2c, sensor_data * sensor_data)
{

    volatile int16_t temp;

    //TSL2561//////////////////////////////////////////////////////////////////////////////////////////
    i2c->tx[0] = TSL2561_CMMND + TSL2561_C0DATAL;
    i2c->i2cTransaction.slaveAddress = TSL2561_I2C_ADR;
    i2c->i2cTransaction.writeBuf = i2c->tx;
    i2c->i2cTransaction.writeCount = 1;
    i2c->i2cTransaction.readBuf = i2c->rx;
    i2c->i2cTransaction.readCount = TSL2561_C0DATAL_LEN;

    if (I2C_transfer(i2c->i2c, &i2c->i2cTransaction))
    {
        ;
    }
    else
    {
        Display_printf(display, 0, 0, "I2C Bus fault\n");
        return (NULL);
    }

    temp = i2c->rx[1];
    // desplazamiento al bit de arriba
    temp = temp << 8;
    temp |= i2c->rx[0];

    temp = i2c->rx[3];
    // desplazamiento al bit de arriba
    temp = temp << 8;
    temp |= i2c->rx[2];

    sensor_data->TSL2561.lux = temp;

    return (NULL);

}

uint8_t TSL2591_INIT(I2C_vars * i2c, sensor_data * sensor_data)
{

    //TSL2591//////////////////////////////////////////////////////////////////////////////////////////

    i2c->tx[0] = TSL2591_CMMND + TSL2591_ENABLE;
    i2c->tx[1] = TSL2591_ENABLE_VALUE;
    i2c->i2cTransaction.slaveAddress = TSL2591_I2C_ADR;
    i2c->i2cTransaction.writeBuf = i2c->tx;
    i2c->i2cTransaction.writeCount = 1 + TSL2591_ENABLE_LEN;
    i2c->i2cTransaction.readBuf = i2c->rx;
    i2c->i2cTransaction.readCount = 0;

    if (I2C_transfer(i2c->i2c, &i2c->i2cTransaction))
        Display_printf(display, 0, 0, "I2C Bus fault\n");
    else
        Display_printf(display, 0, 0, "I2C Config OK\n");

    return (NULL);

}

int16_t TSL2591_READ(I2C_vars * i2c, sensor_data * sensor_data)
{

    volatile int16_t temp;

    //TSL2591//////////////////////////////////////////////////////////////////////////////////////////
    i2c->tx[0] = TSL2591_CMMND + TSL2591_C0DATAL;
    i2c->i2cTransaction.slaveAddress = TSL2591_I2C_ADR;
    i2c->i2cTransaction.writeBuf = i2c->tx;
    i2c->i2cTransaction.writeCount = 1;
    i2c->i2cTransaction.readBuf = i2c->rx;
    i2c->i2cTransaction.readCount = TSL2591_C0DATAL_LEN;

    if (I2C_transfer(i2c->i2c, &i2c->i2cTransaction))
    {
        ;
    }
    else
    {
        Display_printf(display, 0, 0, "I2C Bus fault\n");
        return 0;
    }

    temp = i2c->rx[1];
    // desplazamiento al bit de arriba
    temp = temp << 8;
    temp |= i2c->rx[0];

    temp = i2c->rx[3];
    // desplazamiento al bit de arriba
    temp = temp << 8;
    temp |= i2c->rx[2];

    sensor_data->TSL2591.lux = temp;

    return(temp);

}

int16_t MCP9808_READ(I2C_vars * i2c, sensor_data * sensor_data)
{

    volatile int16_t temp;

    //TSL2561//////////////////////////////////////////////////////////////////////////////////////////

    i2c->tx[0] = MCP9808_ACTUAL_TEMP;
    i2c->i2cTransaction.slaveAddress = MCP9808_I2C_ADR;
    i2c->i2cTransaction.writeBuf = i2c->tx;
    i2c->i2cTransaction.writeCount = 1;
    i2c->i2cTransaction.readBuf = i2c->rx;
    i2c->i2cTransaction.readCount = MCP9808_ACTUAL_TEMP_LEN;

    if (I2C_transfer(i2c->i2c, &i2c->i2cTransaction))
    {
        ;
    }
    else
    {
        Display_printf(display, 0, 0, "I2C Bus fault\n");
        return 0;
    }

    if ((i2c->rx[0] & 0x80) == 0x80)
    { //TA ³ TCRIT
    }
    if ((i2c->rx[0]  & 0x40) == 0x40)
    { //TA > TUPPER
    }
    if ((i2c->rx[0]  & 0x20) == 0x20)
    { //TA < TLOWER
    }

    i2c->rx[0] = i2c->rx[0] & 0x1F; //Clear flag bits

    if ((i2c->rx[0]  & 0x10) == 0x10)
    { //TA < 0°C
        i2c->rx[0]  = i2c->rx[0]  & 0x0F; //Clear SIGN
        temp = 256 - (i2c->rx[0] * 16 + (i2c->rx[1]  / 16));
    }
    else //TA ³ 0°C
        temp = (i2c->rx[0] * 16 + i2c->rx[1]  / 16);
    //Temperature = Ambient Temperature (°C)

    return(temp);

}


