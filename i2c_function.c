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

//////////////////////////////////
// TI includes END              //
//////////////////////////////////

// stacksize for the thread
#define TASKSTACKSIZE       640


uint8_t VELM6070_INIT(I2C_Handle i2c_periph, I2C_Transaction * i2c, uint8_t * tx, uint8_t * rx);
int16_t VELM6070_READ(I2C_Handle i2c_periph, I2C_Transaction * i2c, uint8_t * tx, uint8_t * rx);

uint8_t TSL2561_INIT(I2C_Handle i2c_periph, I2C_Transaction * i2c, uint8_t * tx, uint8_t * rx);
int16_t TSL2561_READ(I2C_Handle i2c_periph, I2C_Transaction * i2c, uint8_t * tx, uint8_t * rx);



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
    unsigned int    i;
    uint16_t        D0;
    uint8_t         txBuffer[10];
    uint8_t         rxBuffer[10];
    I2C_Handle      i2c;
    I2C_Params      i2cParams;
    I2C_Transaction i2cTransaction;

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
    Display_printf(display, 0, 0, "Starting the i2ctmp007 example\n");

    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c = I2C_open(Board_I2C_TMP, &i2cParams);
    if (i2c == NULL) {
        Display_printf(display, 0, 0, "Error Initializing I2C\n");
        while (1);
    }
    else {
        Display_printf(display, 0, 0, "I2C Initialized!\n");
    }


    //TSL2591////////////////////////////////////////////////////////////////////////////////////
/*
    txBuffer[0] = TSL2591_CMMND + TSL2591_ENABLE;
    txBuffer[1] = TSL2591_ENABLE_VALUE;
    i2cTransaction.slaveAddress = TSL2591_I2C_ADR;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1 + TSL2591_ENABLE_LEN;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    if(!(I2C_transfer(i2c, &i2cTransaction)))
        Display_printf(display, 0, 0, "I2C Bus fault\n");
    else
        Display_printf(display, 0, 0, "I2C Config OK\n");

*/
    /* Point to the T ambient register and read its 2 bytes */

/*

    txBuffer[0] = TMP007_OBJ_TEMP;// + TMP007_CMMND;
    i2cTransaction.slaveAddress = TMP007_I2C_ADR;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = TMP007_OBJ_LEN;

*/

    //Inicializaciones
    VELM6070_INIT(i2c, &i2cTransaction, txBuffer, rxBuffer);
    TSL2561_INIT(i2c, &i2cTransaction, txBuffer, rxBuffer);


    /* Take 50 samples and print them out onto the console */
    for (i = 0; i < 50; i++) {

        D0 = VELM6070_READ(i2c, &i2cTransaction, txBuffer, rxBuffer);
        Display_printf(display, 0, 0, "UV: %d (C)\n", D0);


        D0 = TSL2561_READ(i2c, &i2cTransaction, txBuffer, rxBuffer);
        Display_printf(display, 0, 0, "IR+VS: %d (C)\n", D0);

        D0 = MCP9808_READ(i2c, &i2cTransaction, txBuffer, rxBuffer);
        Display_printf(display, 0, 0, "temp: %d (C)\n", D0);

        /* Sleep for 1 second */
        sleep(1);
    }

    /* Deinitialized I2C */
    I2C_close(i2c);
    Display_printf(display, 0, 0, "I2C closed!\n");

    return (NULL);

}


uint8_t VELM6070_INIT(I2C_Handle i2c_periph, I2C_Transaction * i2c, uint8_t * tx, uint8_t * rx)
{

    tx[0] = VELM6070_I2C_CMD_VAL;
    i2c->slaveAddress = VELM6070_I2C_CMD_ADR;
    i2c->writeBuf = tx;
    i2c->writeCount = 1;
    i2c->readBuf = rx;
    i2c->readCount = 0;

    if(!(I2C_transfer(i2c_periph, i2c)))
        Display_printf(display, 0, 0, "I2C Bus fault\n");
    else
        Display_printf(display, 0, 0, "I2C Config OK\n");


    return (NULL);

}

int16_t VELM6070_READ(I2C_Handle i2c_periph, I2C_Transaction * i2c, uint8_t * tx, uint8_t * rx)
{
    volatile int16_t temp;

    i2c->slaveAddress = VELM6070_I2C_D0_ADR;
    i2c->writeBuf = tx;
    i2c->writeCount = 0;
    i2c->readBuf = rx;
    i2c->readCount = 1;

    if (I2C_transfer(i2c_periph, i2c))
        temp = rx[0];
    else
    {
        Display_printf(display, 0, 0, "I2C Bus fault\n");
        return 0;
    }


    // desplazamiento al bit de arriba
    temp = temp << 8;

    i2c->slaveAddress = VELM6070_I2C_D1_ADR;
    i2c->writeBuf = tx;
    i2c->writeCount = 0;
    i2c->readBuf = rx;
    i2c->readCount = 1;

    if (I2C_transfer(i2c_periph, i2c))
    {
        temp |= rx[0];
        return (temp);
    }
    else
    {
        Display_printf(display, 0, 0, "I2C Bus fault\n");
        return 0;
    }


}

uint8_t TSL2561_INIT(I2C_Handle i2c_periph, I2C_Transaction * i2c, uint8_t * tx, uint8_t * rx)
{

    //TSL2561//////////////////////////////////////////////////////////////////////////////////////////

    tx[0] = TSL2561_CMMND + TSL2561_ENABLE;
    tx[1] = TSL2561_ENABLE_VALUE;
    i2c->slaveAddress = TSL2561_I2C_ADR;
    i2c->writeBuf = tx;
    i2c->writeCount = 1 + TSL2561_ENABLE_LEN;
    i2c->readBuf = rx;
    i2c->readCount = 0;

    if(!(I2C_transfer(i2c_periph, i2c)))
        Display_printf(display, 0, 0, "I2C Bus fault\n");
    else
        Display_printf(display, 0, 0, "I2C Config OK\n");


    return (NULL);

}

int16_t TSL2561_READ(I2C_Handle i2c_periph, I2C_Transaction * i2c, uint8_t * tx, uint8_t * rx)
{

    volatile int16_t temp;

    //TSL2561//////////////////////////////////////////////////////////////////////////////////////////
    tx[0] = TSL2561_CMMND + TSL2561_C0DATAL;
    i2c->slaveAddress = TSL2561_I2C_ADR;
    i2c->writeBuf = tx;
    i2c->writeCount = 1;
    i2c->readBuf = rx;
    i2c->readCount = TSL2561_C0DATAL_LEN;

    if (I2C_transfer(i2c_periph, i2c))
    {
        ;
    }
    else
    {
        Display_printf(display, 0, 0, "I2C Bus fault\n");
        return 0;
    }

    temp = rx[1];
    // desplazamiento al bit de arriba
    temp = temp << 8;
    temp |= rx[0];

    temp = rx[3];
    // desplazamiento al bit de arriba
    temp = temp << 8;
    temp |= rx[2];

    return(temp);

}

int16_t MCP9808_READ(I2C_Handle i2c_periph, I2C_Transaction * i2c, uint8_t * tx, uint8_t * rx)
{

    volatile int16_t temp;

    //TSL2561//////////////////////////////////////////////////////////////////////////////////////////

    tx[0] = MCP9808_ACTUAL_TEMP;
    i2c->slaveAddress = MCP9808_I2C_ADR;
    i2c->writeBuf = tx;
    i2c->writeCount = 1;
    i2c->readBuf = rx;
    i2c->readCount = MCP9808_ACTUAL_TEMP_LEN;

    if (I2C_transfer(i2c_periph, i2c))
    {
        ;
    }
    else
    {
        Display_printf(display, 0, 0, "I2C Bus fault\n");
        return 0;
    }

    if ((rx[0] & 0x80) == 0x80)
    { //TA ³ TCRIT
    }
    if ((rx[0]  & 0x40) == 0x40)
    { //TA > TUPPER
    }
    if ((rx[0]  & 0x20) == 0x20)
    { //TA < TLOWER
    }

    rx[0] = rx[0] & 0x1F; //Clear flag bits

    if ((rx[0]  & 0x10) == 0x10)
    { //TA < 0°C
        rx[0]  = rx[0]  & 0x0F; //Clear SIGN
        temp = 256 - (rx[0] * 16 + (rx[1]  / 16));
    }
    else //TA ³ 0°C
        temp = (rx[0] * 16 + rx[1]  / 16);
    //Temperature = Ambient Temperature (°C)

    return(temp);

}


