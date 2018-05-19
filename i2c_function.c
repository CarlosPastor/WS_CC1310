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

int16_t TMP007_READ(I2C_vars * i2c, sensor_data * sensor_data);

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

#define TMP007_CMMND        0x80           /* default command */

#define TMP007_DIE_TEMP     0x0001  /* Die Temp Result Register */
#define TMP007_TEMP_LEN   2           

#define TMP007_CONFIG       0x02           /* Configuration Register */
#define TMP007_CONFIG_LEN   2           

#define TMP007_OBJ_TEMP     0x0003  /* Object Temp Result Register */
#define TMP007_OBJ_LEN      2        

#define TMP007_STATUS       0x04           /* Status Register */
#define TMP007_STATUS_LEN   2           

#define TMP007_STATUS_ME       0x05        /* Mask and enable Register */
#define TMP007_STATUS_ME_LEN   2           

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

        VELM6070_READ(&I2C_vars, &sensor_data);
        TSL2561_READ(&I2C_vars, &sensor_data);
        TSL2591_READ(&I2C_vars, &sensor_data);
        MCP9808_READ(&I2C_vars, &sensor_data);
        TMP007_READ(&I2C_vars, &sensor_data);

        Display_printf(display, 0, 0, "DATOS DE LOS SENSORES:\n ");
        // sensores de temperatura
        Display_printf(display, 0, 0, "\tMCP9808 TEMPERATURE: %d C \n", sensor_data.MCP9808.temp);

        Display_printf(display, 0, 0, "\tTMP007 TEMPERATURE: %d C \n", sensor_data.TMP007.temp_IR);
        // sensores de luminosidad
        Display_printf(display, 0, 0, "\tTSL2561 Intensidad de luz: %d lx \n", sensor_data.TSL2561.lux);
        Display_printf(display, 0, 0, "\tTSL2561 Irradiancia completa: %d \n", sensor_data.TSL2561.CH0);
        Display_printf(display, 0, 0, "\tTSL2561 Irradiancia IR: %d \n", sensor_data.TSL2561.CH1);

        Display_printf(display, 0, 0, "\tTSL2591 Intensidad de luz: %.3f lx \n", sensor_data.TSL2591.lux);
        Display_printf(display, 0, 0, "\tTSL2591 Irradiancia completa: %d \n", sensor_data.TSL2591.CH0);
        Display_printf(display, 0, 0, "\tTSL2591 Irradiancia IR: %d \n", sensor_data.TSL2591.CH1);

        Display_printf(display, 0, 0, "\tVELM6070 Factor de radiacion UV: %d (C)\n", sensor_data.VELM6070.lux_UV);
        // otros....



        /* Sleep for 1 second */
        sleep(1);
    }

    /* Deinitialized I2C */
    I2C_close(I2C_vars.i2c);
    Display_printf(display, 0, 0, "I2C closed!\n");

    return (NULL);

}

int16_t TMP007_READ(I2C_vars * i2c, sensor_data * sensor_data)
{
    volatile int16_t temp;

    i2c->tx[0] = TMP007_OBJ_TEMP;     /* Indico donde quiero leer */

    i2c->i2cTransaction.slaveAddress = TMP007_I2C_ADR;  /* Indico address */
    i2c->i2cTransaction.writeBuf = i2c->tx;
    i2c->i2cTransaction.writeCount = 1;
    i2c->i2cTransaction.readBuf = i2c->rx;
    i2c->i2cTransaction.readCount = 2;

    if (I2C_transfer(i2c->i2c, &i2c->i2cTransaction))
        ;
    else
    {
        Display_printf(display, 0, 0, "I2C Bus fault TMP007\n");
        return (NULL);
    }

    /* Extract degrees ºC from the received data; see TMP102 datasheet */
    temp = (i2c->rx[0] << 6) | (i2c->rx[1] >> 2);

    /*
    * If the MSB is set '1', then we have a 2's complement
    * negative value which needs to be sign extended
    */
    if (i2c->rx[0] & 0x80) {
        temp |= 0xF000;
    }
    /*
    * For simplicity, divide the temperature value by 32 to get rid of
    * the decimal precision; see TI's TMP007 datasheet
    */
    temp /= 32;


        temp |= i2c->rx[0];
        sensor_data->TMP007.temp_IR = temp;
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
    i2c->i2cTransaction.readCount = 2;

    if (I2C_transfer(i2c->i2c, &i2c->i2cTransaction))
    {
        ;
    }
    else
    {
        Display_printf(display, 0, 0, "I2C Bus fault\n");
        return (NULL);
    }

    // extraccion de los datos de cada canal:

    // canal 0
    temp = i2c->rx[1];
    // desplazamiento al bit de arriba
    temp = temp << 8;
    temp |= i2c->rx[0];

    sensor_data->TSL2561.CH0 = temp;

    //TSL2561//////////////////////////////////////////////////////////////////////////////////////////
    i2c->tx[0] = TSL2561_CMMND + TSL2561_C1DATAL;
    i2c->i2cTransaction.slaveAddress = TSL2561_I2C_ADR;
    i2c->i2cTransaction.writeBuf = i2c->tx;
    i2c->i2cTransaction.writeCount = 1;
    i2c->i2cTransaction.readBuf = i2c->rx;
    i2c->i2cTransaction.readCount = 2;

    if (I2C_transfer(i2c->i2c, &i2c->i2cTransaction))
    {
        ;
    }
    else
    {
        Display_printf(display, 0, 0, "I2C Bus fault\n");
        return (NULL);
    }

    // canal 1
    temp = i2c->rx[1];
    // desplazamiento al bit de arriba
    temp = temp << 8;
    temp |= i2c->rx[0];

    sensor_data->TSL2561.CH1 = temp;

    // El canal 0 contiene la informacion de el espectro completo
    //CH0 ADC low data byte
    //CH0 ADC high data byte
    // El canal 1 contiene la informacion IR unicamente
    //CH1 ADC low data byte
    //CH1 ADC high data byte
    // Usando la combinacion de esta informacion se obtiene la iluminancia en luxes


    // T, FN and CL package values
    uint16_t TSL2561_LUX_K1T=(0x0040);  ///< 0.125 * 2^RATIO_SCALE
    uint16_t TSL2561_LUX_B1T=(0x01f2); ///< 0.0304 * 2^LUX_SCALE
    uint16_t TSL2561_LUX_M1T=(0x01be); ///< 0.0272 * 2^LUX_SCALE
    uint16_t TSL2561_LUX_K2T=(0x0080); ///< 0.250 * 2^RATIO_SCALE
    uint16_t TSL2561_LUX_B2T=(0x0214); ///< 0.0325 * 2^LUX_SCALE
    uint16_t TSL2561_LUX_M2T=(0x02d1); ///< 0.0440 * 2^LUX_SCALE
    uint16_t TSL2561_LUX_K3T=(0x00c0); ///< 0.375 * 2^RATIO_SCALE
    uint16_t TSL2561_LUX_B3T=(0x023f);  ///< 0.0351 * 2^LUX_SCALE
    uint16_t TSL2561_LUX_M3T=(0x037b);  ///< 0.0544 * 2^LUX_SCALE
    uint16_t TSL2561_LUX_K4T=(0x0100);  ///< 0.50 * 2^RATIO_SCALE
    uint16_t TSL2561_LUX_B4T=(0x0270);  ///< 0.0381 * 2^LUX_SCALE
    uint16_t TSL2561_LUX_M4T=(0x03fe);  ///< 0.0624 * 2^LUX_SCALE
    uint16_t TSL2561_LUX_K5T=(0x0138);  ///< 0.61 * 2^RATIO_SCALE
    uint16_t TSL2561_LUX_B5T=(0x016f);  ///< 0.0224 * 2^LUX_SCALE
    uint16_t TSL2561_LUX_M5T=(0x01fc);  ///< 0.0310 * 2^LUX_SCALE
    uint16_t TSL2561_LUX_K6T=(0x019a);  ///< 0.80 * 2^RATIO_SCALE
    uint16_t TSL2561_LUX_B6T=(0x00d2);  ///< 0.0128 * 2^LUX_SCALE
    uint16_t TSL2561_LUX_M6T=(0x00fb);  ///< 0.0153 * 2^LUX_SCALE
    uint16_t TSL2561_LUX_K7T=(0x029a);  ///< 1.3 * 2^RATIO_SCALE
    uint16_t TSL2561_LUX_B7T=(0x0018);  ///< 0.00146 * 2^LUX_SCALE
    uint16_t TSL2561_LUX_M7T=(0x0012);  ///< 0.00112 * 2^LUX_SCALE
    uint16_t TSL2561_LUX_K8T=(0x029a);  ///< 1.3 * 2^RATIO_SCALE
    uint16_t TSL2561_LUX_B8T=(0x0000);  ///< 0.000 * 2^LUX_SCALE
    uint16_t TSL2561_LUX_M8T=(0x0000);  ///< 0.000 * 2^LUX_SCALE

    unsigned long chScale;
    unsigned long channel1;
    unsigned long channel0;

    uint16_t broadband, ir;

    broadband = sensor_data->TSL2561.CH0;
    ir = sensor_data->TSL2561.CH1;

    /* Make sure the sensor isn't saturated! */
    uint16_t clipThreshold;
    clipThreshold = 65000; //TSL2561_CLIPPING_402MS;

    /* Return 65536 lux if the sensor is saturated */
    if ((broadband > clipThreshold) || (ir > clipThreshold))
    {
      return 65536;
    }

    chScale = (1 << 10); //TSL2561_LUX_CHSCALE

    /* Scale for gain (1x or 16x) */
    chScale = chScale << 4; // only in default

    /* Scale the channel values */
    channel0 = (broadband * chScale) >> 10;
    channel1 = (ir * chScale) >> 10;

    /* Find the ratio of the channel values (Channel1/Channel0) */
    unsigned long ratio1 = 0;
    if (channel0 != 0) ratio1 = (channel1 << (9+1)) / channel0; //TSL2561_LUX_RATIOSCALE

    /* round the ratio value */
    unsigned long ratio = (ratio1 + 1) >> 1;

    unsigned int b, m;

    if ((ratio >= 0) && (ratio <= TSL2561_LUX_K1T))
      {b=TSL2561_LUX_B1T; m=TSL2561_LUX_M1T;}
    else if (ratio <= TSL2561_LUX_K2T)
      {b=TSL2561_LUX_B2T; m=TSL2561_LUX_M2T;}
    else if (ratio <= TSL2561_LUX_K3T)
      {b=TSL2561_LUX_B3T; m=TSL2561_LUX_M3T;}
    else if (ratio <= TSL2561_LUX_K4T)
      {b=TSL2561_LUX_B4T; m=TSL2561_LUX_M4T;}
    else if (ratio <= TSL2561_LUX_K5T)
      {b=TSL2561_LUX_B5T; m=TSL2561_LUX_M5T;}
    else if (ratio <= TSL2561_LUX_K6T)
      {b=TSL2561_LUX_B6T; m=TSL2561_LUX_M6T;}
    else if (ratio <= TSL2561_LUX_K7T)
      {b=TSL2561_LUX_B7T; m=TSL2561_LUX_M7T;}
    else if (ratio > TSL2561_LUX_K8T)
      {b=TSL2561_LUX_B8T; m=TSL2561_LUX_M8T;}

    unsigned long ltemp;
    ltemp = ((channel0 * b) - (channel1 * m));

    /* Do not allow negative lux value */
    if (ltemp < 0) ltemp = 0;

    /* Round lsb (2^(LUX_SCALE-1)) */
    ltemp += (1 << (14-1)); // TSL2561_LUX_LUXSCALE

    /* Strip off fractional portion */
    uint32_t lux = ltemp >> 14; // TSL2561_LUX_LUXSCALE

    sensor_data->TSL2561.lux = lux;

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

    // El canal 0 contiene la informacion de el espectro completo
    //CH0 ADC low data byte
    //CH0 ADC high data byte
    // El canal 1 contiene la informacion IR unicamente
    //CH1 ADC low data byte
    //CH1 ADC high data byte
    // Usando la combinacion de esta informacion se obtiene la iluminancia en luxes


    // extraccion de los datos de cada canal:

    // canal 0
    temp = i2c->rx[1];
    // desplazamiento al bit de arriba
    temp = temp << 8;
    temp |= i2c->rx[0];

    sensor_data->TSL2591.CH0 = temp;

    // canal 1
    temp = i2c->rx[3];
    // desplazamiento al bit de arriba
    temp = temp << 8;
    temp |= i2c->rx[2];

    sensor_data->TSL2591.CH1 = temp;

    // Calculo de la ilumanancia

    float TSL2591_LUX_DF    = 408.0F; ///< Lux cooefficient
    float TSL2591_LUX_COEFB = 1.64F;  ///< CH0 coefficient
    float TSL2591_LUX_COEFC = 0.59F;  ///< CH1 coefficient A
    float TSL2591_LUX_COEFD = 0.86F;  ///< CH2 coefficient B

    float    atime, again;
    float    cpl, lux1, lux2, lux;
    uint32_t chan0, chan1;

    // Check for overflow conditions first
    if ((sensor_data->TSL2591.CH0 == 0xFFFF) | (sensor_data->TSL2591.CH1 == 0xFFFF))
    {
      // Signal an overflow
      return 0;
    }

    // Note: This algorithm is based on preliminary coefficients
    // provided by AMS and may need to be updated in the future

    atime = 100.0F;
    again = 1.0F;

    // cpl = (ATIME * AGAIN) / DF
    cpl = (atime * again) / TSL2591_LUX_DF;

    // Original lux calculation (for reference sake)
    // lux1 = ( (float)ch0 - (TSL2591_LUX_COEFB * (float)ch1) ) / cpl;
    // lux2 = ( ( TSL2591_LUX_COEFC * (float)ch0 ) - ( TSL2591_LUX_COEFD * (float)ch1 ) ) / cpl;
    // lux = lux1 > lux2 ? lux1 : lux2;

    // Alternate lux calculation 1
    // See: https://github.com/adafruit/Adafruit_TSL2591_Library/issues/14
    lux = ( ((float)sensor_data->TSL2591.CH0 - (float)sensor_data->TSL2591.CH1 )) * (1.0F - ((float)sensor_data->TSL2591.CH1/(float)sensor_data->TSL2591.CH0) ) / cpl;

    // Alternate lux calculation 2
    //lux = ( (float)ch0 - ( 1.7F * (float)ch1 ) ) / cpl;
    sensor_data->TSL2591.lux = lux;

    return 0;

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

    sensor_data->MCP9808.temp = temp;

    return(temp);

}


