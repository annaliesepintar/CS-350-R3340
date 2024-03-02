/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>

/* Driver configuration */
#include "ti_drivers_config.h"

#include <ti/drivers/I2C.h>

#include <ti/drivers/UART2.h>
#define DISPLAY(x) UART2_write(uart, &output, x, NULL);

#include <ti/drivers/Timer.h>

// Driver Handles - Global variables
Timer_Handle timer0;
volatile unsigned char TimerFlag = 0;
volatile unsigned char ButtonFlag1 = 0;
volatile unsigned char ButtonFlag2 = 0;
volatile unsigned char HeatFlag = 0;
volatile unsigned char setpoint = 0;
volatile unsigned char heat = 0;
int16_t temperature;

typedef struct task
{
    int state;
    unsigned long period;
    unsigned long elapsedTime;
    void (*flag)(void);
    int (*TickFun) (int);

}task;

task tasks[2];

const unsigned char tasksNum = 2;
const unsigned char tasksPeriod = 100;    // task period 100 milliseconds
const unsigned char buttonPeriod = 200;   // button check period 200 milliseconds
const unsigned long ledPeriod = 500;      // LED update period 500 milliseconds

enum Button_States {BUTTON_SMStart, BUTTON_Init, BUTTON_Waiting, BUTTON_Toggled} Button_State;
enum Heater_States {HEATER_SMStart, HEATER_Init, HEATER_LedOff, HEATER_LedOn} Heater_State;


void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    TimerFlag = 1;
}

void initTimer(void)
{
    Timer_Params params;
    // Init the driver
    Timer_init();
    // Configure the driver
    Timer_Params_init(&params);
    params.period = 100000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }
}



// UART Global Variables
char output[64];
int bytesToSend;
// Driver Handles - Global variables
UART2_Handle uart;

void initUART(void)
{
    UART2_Params uartParams;
    // Init the driver
    //UART2_init();
    // Configure the driver
    UART2_Params_init(&uartParams);
    uartParams.writeMode = UART2_Mode_BLOCKING;
    uartParams.readMode = UART2_Mode_BLOCKING;
    uartParams.readReturnMode = UART2_ReadReturnMode_FULL;
    uartParams.baudRate = 115200;
    // Open the driver
    uart = UART2_open(CONFIG_UART2_0, &uartParams);
    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}

// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
    } sensors[3] = {
                    { 0x48, 0x0000, "11X" },
                    { 0x49, 0x0000, "116" },
                    { 0x41, 0x0001, "006" }
    };

uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;
// Driver Handles - Global variables
I2C_Handle i2c;
// Make sure you call initUART() before calling this function.

void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;
    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - ", NULL, NULL))
    // Init the driver
    I2C_init();
    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL)
    {
        DISPLAY(snprintf(output, 64, "Failed\n\r"))
        while (1);
    }
    DISPLAY(snprintf(output, 32, "Passed\n\r"))
    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses
    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    found = false;
    for (i=0; i<3; ++i)
    {
        i2cTransaction.targetAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))
        if (I2C_transfer(i2c, &i2cTransaction))
        {
            DISPLAY(snprintf(output, 64, "Found\n\r"))
        found = true;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"))
    }
    if(found)
    {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address:%x\n\r", sensors[i].id, i2cTransaction.targetAddress))
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"))
    }
    }
    int16_t readTemp(void)
    {
        int j;
        int16_t temperature = 0;
        i2cTransaction.readCount = 2;
        if (I2C_transfer(i2c, &i2cTransaction))
        {
            /*
             * Extract degrees C from the received data;
             * see TMP sensor datasheet
             */
            temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
            temperature *= 0.0078125;
            /*
             * If the MSB is set '1', then we have a 2's complement
             * negative value which needs to be sign extended
             */
            if (rxBuffer[0] & 0x80)
            {
                temperature |= 0xF000;
            }
        }
        else
        {
            DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r",i2cTransaction.status))
            DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"))
        }
        return temperature;
    }

    void initGPIO(void)
    {

        /* Call driver init functions */
            GPIO_init();

            /* Configure the LED and button pins */
            GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

            GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

            /* Turn on user LED */
           // GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

            /* Configure BUTTON1 pin */
            GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    }



/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{

    // button 1 has been pressed
    ButtonFlag1 = 1;


}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    // button 2 has been pressed
    ButtonFlag2 = 1;

}

int buttonToggled(int state)
{
    switch(Button_State)
    {
    case BUTTON_SMStart:
        Button_State = BUTTON_Init;
        break;

    case BUTTON_Init:
        if(ButtonFlag1 == 1|| ButtonFlag2 == 1)
        {
            Button_State = BUTTON_Toggled;
        }
        else
        {
            Button_State = BUTTON_Waiting;
        }
        break;

    case BUTTON_Toggled:
        Button_State = BUTTON_Waiting;
        break;

    case BUTTON_Waiting:
        if(ButtonFlag1 == 1|| ButtonFlag2 == 1)
        {
            Button_State = BUTTON_Toggled;
        }
        else
        {
            Button_State = BUTTON_Waiting;
        }
        break;

    default:
        Button_State = BUTTON_SMStart; // something went wrong, back to starting state
        break;

    }

    switch(Button_State)
    {
    case BUTTON_Toggled:
        if(ButtonFlag1 == 1)
        {
            if(setpoint != 99)
            {
                ++setpoint;
            }
            else
            {
                setpoint = 99;
            }
            ButtonFlag1 = 0;
        }
       if(ButtonFlag2 == 1)
        {
            if(setpoint != 0)
            {
                --setpoint;
            }
            else
            {
                setpoint = 0;
            }
            ButtonFlag2 = 0;
        }
        break;

    case BUTTON_Waiting:
        break;

    default:
        break;

    }

    return state;
}

int heaterLight(int state)
{

    switch(Heater_State)
    {
    case HEATER_SMStart:
        Heater_State = HEATER_Init;
        break;

    case HEATER_Init:
        if(heat == 1)
        {
            Heater_State = HEATER_LedOn;
        }
        else if(heat == 0)
        {
            Heater_State = HEATER_LedOff;
        }

        break;

    case HEATER_LedOn:
        if(heat == 1)
        {
            Heater_State = HEATER_LedOn;
        }
        else if(heat == 0)
        {
            Heater_State = HEATER_LedOff;
        }

        break;

    case HEATER_LedOff:
        if(heat == 1)
        {
            Heater_State = HEATER_LedOn;
        }
        else if(heat == 0)
        {
            Heater_State = HEATER_LedOff;
        }

        break;

    default:
        Heater_State = HEATER_SMStart;
        break;
    }

    switch(Heater_State)
    {
    case HEATER_LedOn:
        GPIO_write(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_HIGH);
        break;

    case HEATER_LedOff:
        GPIO_write(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_LOW);
        break;

    default:
        break;

    }

    return state;
}

void checkButtonFlag(void)
{

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_1);


}

void checkLedFlag(void)
{


    // get temp
    temperature = readTemp();

    // if temp is greater than set point, turn heater on
    if(temperature > setpoint)
    {
        heat = 1;
    }

    // if temp is less than or equal to set point, turn heater off
    if(temperature <= setpoint)
    {
        heat = 0;
    }

}
/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    unsigned char timer = 0;
    unsigned char seconds = 0;


    initUART();
    initI2C();
    initTimer();
    initGPIO();

    //Initialize tasks
    unsigned char i = 0;
    tasks[i].state = BUTTON_SMStart;
    tasks[i].period = buttonPeriod;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].flag = &checkButtonFlag;
    tasks[i].TickFun = &buttonToggled;
    ++i;
    tasks[i].state = HEATER_SMStart;
    tasks[i].period = ledPeriod;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].flag = &checkLedFlag;
    tasks[i].TickFun = &heaterLight;

    //led off
    GPIO_write(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_LOW);

    while(1)
    {
        unsigned char j = 0;

        //loop through tasks
        for(j = 0; j < tasksNum; ++j)
        {
            if(tasks[j].elapsedTime >= tasks[j].period)
            {
                // check for flag indicating led should turn on or button is pressed
                tasks[j].flag();

                // change state of task
                tasks[j].state = tasks[j].TickFun(tasks[j].state);

                // reset elapsed time
                tasks[j].elapsedTime = 0;
            }
            else
            {
                // elapsed time
                tasks[j].elapsedTime += tasksPeriod;
            }
        }

        // display to uart every second
        if(timer % 10 == 0 && timer != 0)
        {
            DISPLAY( snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", temperature, setpoint, heat, seconds))

        }

        // wait for timer
        while(!TimerFlag)
        {
        }
        TimerFlag = 0;

        // increment timer
        ++timer;

        // if timer is greater than 10, reset
        if(timer > 10)
        {
            timer = 0;
            ++seconds;
        }

    }



    return (NULL);
}
