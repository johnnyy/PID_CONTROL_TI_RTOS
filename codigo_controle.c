/**
 *  \file   main.c
 *
 *  \brief  Example application main file. This application will toggle the led.
 *          The led toggling will be done inside an callback function, which
 *          will be called by Interrupt Service Routine. Interrupts are
 *          triggered manually and no external source is used to trigger
 *          interrupts.
 *
 */

/*
 * Copyright (C) 2014 - 2018 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


#ifdef USE_BIOS
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#if defined(SOC_AM65XX) || defined(SOC_J721E)
#if defined (__aarch64__)
#include <ti/sysbios/family/arm/v8a/Mmu.h>
#endif
#endif
#endif /* #ifdef USE_BIOS */

#include <stdio.h>

/* TI-RTOS Header files */
#include <ti/drv/gpio/GPIO.h>
#include <ti/drv/gpio/soc/GPIO_soc.h>

#include "GPIO_log.h"
#include "GPIO_board.h"

#include <ti/board/board.h>
#include <ti/sysbios/family/arm/a8/intcps/Hwi.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>

#include <ti/drv/i2c/I2C.h>
#include <ti/drv/i2c/soc/I2C_soc.h>

#if defined(SOC_AM65XX) || defined(SOC_J721E)
#include <ti/drv/sciclient/sciclient.h>
#endif

/**********************************************************************
 ************************** Macros ************************************
 **********************************************************************/
#define DELAY_VALUE       (500U)   /* 500 msec */

/**********************************************************************
 ************************** Internal functions ************************
 **********************************************************************/

/* Delay function */
void AppDelay(unsigned int delayVal);

/* Callback function */
void AppGpioCallbackFxn(void);

#if defined(idkAM574x) || defined(idkAM572x) || defined(idkAM571x)
/* GPIO clock and pinmux configurations */
extern void AppGPIOInit(void);
#endif

#if defined(idkAM574x) || defined(idkAM572x)
extern void GPIOApp_UpdateBoardInfo(void);
extern void GPIOAppUpdateConfig(uint32_t *gpioBaseAddr, uint32_t *gpioPin);
#endif

#if defined(SOC_AM65XX) || defined(SOC_J721E)
/* Main domain GPIO interrupt events */
#define MAIN_GPIO_INTRTR_GPIO0_BANK0_INT (0x000000C0) /* GPIO port 0 bank 0 interrupt event #, input to MAIN_GPIO_INTRTR */
#define MAIN_GPIO_INTRTR_GPIO1_BANK0_INT (0x000000C8) /* GPIO port 1 bank 0 interrupt event #, input to MAIN_GPIO_INTRTR */

/* Main domain GPIO interrupt events */
#define WKUP_GPIO_INTRTR_GPIO0_BANK0_INT (0x0000003C) /* GPIO port 0 bank 0 interrupt event #, input to WKUP_GPIO_INTRTR */


/* Main to MCU GPIO interrupt router mux output events */
#define CSL_MAIN2MCU_INTRTR_PLS_GPIOMUX_INT0_DFLT_PLS  (0x00000000)
#define CSL_MAIN2MCU_INTRTR_PLS_GPIOMUX_INT31_DFLT_PLS (0x0000001F)

void GPIO_configIntRouter(uint32_t portNum, uint32_t pinNum, uint32_t gpioIntRtrOutIntNum, GPIO_v0_HwAttrs *cfg)
{
    GPIO_IntCfg       *intCfg;
    uint32_t           bankNum;

    intCfg = cfg->intCfg;

#if defined (am65xx_evm) || defined (am65xx_idk) || defined(j721e_sim)
    
    /* no main domain GPIO pins directly connected to LEDs on GP EVM, 
       use WKUP domain GPIO pins which connected to LEDs on base board */
    cfg->baseAddr = CSL_WKUP_GPIO0_BASE;

    bankNum = pinNum/16; /* Each GPIO bank has 16 pins */

    /* WKUP GPIO int router input interrupt is the GPIO bank interrupt */
#if defined (__aarch64__)
#if defined (SOC_AM65XX)
    intCfg[pinNum].intNum = CSL_GIC0_INTR_WKUP_GPIOMUX_INTRTR0_BUS_OUTP_0 + bankNum;
#endif
#if defined (SOC_J721E)
    intCfg[pinNum].intNum = CSLR_WKUP_GPIOMUX_INTRTR0_IN_WKUP_GPIO0_GPIO_BANK_0 + bankNum;
#endif
#else
#if defined (SOC_AM65XX)
    intCfg[pinNum].intNum = CSL_MCU0_INTR_GPIOMUX_INTR0_OUTP_0 + bankNum;
#endif
#if defined (SOC_J721E)
    intCfg[pinNum].intNum = CSLR_ARMSS0_CPU0_INTR_GPIOMUX_INTRTR0_OUTP_16 + bankNum;
#endif
#endif
    intCfg[pinNum].eventId = 0;
    intCfg[pinNum].intcMuxNum = INVALID_INTC_MUX_NUM;
    intCfg[pinNum].intcMuxInEvent = 0;
    intCfg[pinNum].intcMuxOutEvent = 0;
    
    /* Setup interrupt router configuration for gpio port/pin */
#else
    /* Use main domain GPIO pins directly connected to IDK EVM */

    bankNum = pinNum/16; /* Each GPIO bank has 16 pins */
    if (portNum == 0)
    {
        /* MAIN GPIO int router input interrupt is the GPIO bank interrupt */
#if defined (__aarch64__)
#if defined (SOC_AM65XX)
        intCfg[pinNum].intNum = CSL_GIC0_INTR_MAIN_GPIOMUX_INTROUTER_MAIN_GPIOMUX_INTROUTER_MAIN_0_BUS_OUTP_0 + bankNum;
#endif
#if defined (SOC_J721E)
        intCfg[pinNum].intNum = CSLR_COMPUTE_CLUSTER0_GIC_SPI_GPIOMUX_INTRTR0_OUTP_8 + bankNum;
#endif
#else
#if defined (SOC_AM65XX)
        intCfg[pinNum].intNum = CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_0 + bankNum;
#endif
#if defined (SOC_J721E)
        intCfg[pinNum].intNum = CSLR_MCU_ARMSS0_CPU0_INTR_MAIN2MCU_PLS_INTRTR0_OUTP_0 + bankNum;
#endif
#endif
    }
    else
    {
#if defined (__aarch64__)
#if defined (SOC_AM65XX)
        intCfg[pinNum].intNum = CSL_GIC0_INTR_MAIN_GPIOMUX_INTROUTER_MAIN_GPIOMUX_INTROUTER_MAIN_0_BUS_OUTP_6 + bankNum;
#endif
#if defined (SOC_J721E)
        intCfg[pinNum].intNum = CSLR_COMPUTE_CLUSTER0_GIC_SPI_GPIOMUX_INTRTR0_OUTP_14 + bankNum;
#endif
#else
#if defined (SOC_AM65XX)
        intCfg[pinNum].intNum = CSL_MCU0_INTR_MAIN2MCU_PULSE_INTR0_OUTP_6 + bankNum;
#endif
#if defined (SOC_J721E)
        intCfg[pinNum].intNum = CSLR_MCU_ARMSS0_CPU0_INTR_MAIN2MCU_PLS_INTRTR0_OUTP_6 + bankNum;
#endif
#endif
    }
    intCfg[pinNum].eventId = 0;
    intCfg[pinNum].intcMuxNum = INVALID_INTC_MUX_NUM;
    intCfg[pinNum].intcMuxInEvent = 0;
    intCfg[pinNum].intcMuxOutEvent = 0;
    
#endif
    GPIO_log("\nIntConfig:  portNum[%d],pinNum[%d],bankNum[%d], intNum[%d]",portNum,pinNum,bankNum,intCfg[pinNum].intNum);
}



#ifdef USE_BIOS
#if defined (__aarch64__)
Void InitMmu()
{
    Mmu_MapAttrs attrs;
    Bool         retVal;
    uint32_t     mapIdx = 0;

    Mmu_initMapAttrs(&attrs);

    attrs.attrIndx = 0;
    retVal = Mmu_map(0x00100000, 0x00100000, 0x00900000, &attrs); /* Main MMR0 cfg  */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x00400000, 0x00400000, 0x00001000, &attrs); /* PSC0          */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x01800000, 0x01800000, 0x00200000, &attrs); /* gicv3          */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x02400000, 0x02400000, 0x000c0000, &attrs); /* dmtimer        */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }
    
    mapIdx++;
    retVal = Mmu_map(0x02800000, 0x02800000, 0x00040000, &attrs); /* uart           */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x02000000, 0x02000000, 0x00100000, &attrs); /* main I2C       */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    
    mapIdx++;
    retVal = Mmu_map(0x42120000, 0x42120000, 0x00001000, &attrs); /* Wkup I2C0       */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x02100000, 0x02100000, 0x00080000, &attrs); /* McSPI          */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x00600000, 0x00600000, 0x00002000, &attrs); /* GPIO           */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }
     
    mapIdx++;
    retVal = Mmu_map(0x42110000, 0x42110000, 0x00001000, &attrs); /* WKUP GPIO      */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }
     
    mapIdx++;
    retVal = Mmu_map(0x00a00000, 0x00a00000, 0x00040000, &attrs); /* MAIN INTR_ROUTERs */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }
     
    mapIdx++;
    retVal = Mmu_map(0x42200000, 0x42200000, 0x00001000, &attrs); /* WKUP INTR_ROUTER */
    if (retVal == FALSE)
    {
         goto mmu_exit;
    }
    
    mapIdx++;
    retVal = Mmu_map(0x40f00000, 0x40f00000, 0x00020000, &attrs); /* MCU MMR0 CFG   */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x40d00000, 0x40d00000, 0x00002000, &attrs); /* PLL0 CFG       */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x43000000, 0x43000000, 0x00020000, &attrs); /* WKUP MMR0 cfg  */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x02C40000, 0x02C40000, 0x00100000, &attrs); /* pinmux ctrl    */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x2A430000, 0x2A430000, 0x00001000, &attrs); /* ctrcontrol0    */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x030000000, 0x030000000, 0x10000000, &attrs); /* NAVSS used by sciclient  */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }


    mapIdx++;
    retVal = Mmu_map(0x42000000, 0x42000000, 0x00001000, &attrs); /* PSC WKUP */
    if (retVal == FALSE)
    {
         goto mmu_exit;
    }

    attrs.attrIndx = 7;
    mapIdx++;
    retVal = Mmu_map(0x80000000, 0x80000000, 0x03000000, &attrs); /* ddr            */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    mapIdx++;
    retVal = Mmu_map(0x70000000, 0x70000000, 0x04000000, &attrs); /* msmc           */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

mmu_exit:
    if(retVal == FALSE)
    {
        System_printf("Mmu_map idx %d returned error %d", mapIdx, retVal);
        while(1);
    }

    return;
}
#endif /* #if defined (__aarch64__) */
#endif /* #ifdef USE_BIOS */
#endif /* #if defined(SOC_AM65XX) || defined(SOC_J721E) */


#define PWM                     4095
#define PWM_init                0
#define QTD_IDEAL_INT_100m      4

#define ERRO                    0.05

#define Raio                    3.4
#define Pi                      3.1415

// I2C1

#define I2C_MPU_6065_INSTANCE   1
#define MPU_6050_SLAVE_ADDR     0x68

#define PWR_MGMT_1              0x6b
#define WHOAMI                  0x75
#define ACCEL_CONFIG            0x1c
#define GYRO_CONFIG             0x1b
#define CONFIG                  0x1a
#define SMPLRT_DIV              0x19
#define FIFO_EN                 0x23

#define ACCEL_XOUT_H            0x3b
#define ACCEL_XOUT_L            0x3c
#define ACCEL_YOUT_H            0x3d
#define ACCEL_YOUT_L            0x3e
#define ACCEL_ZOUT_H            0x3f
#define ACCEL_ZOUT_L            0x40

#define GYRO_XOUT_H             0x43
#define GYRO_XOUT_L             0x44
#define GYRO_YOUT_H             0x45
#define GYRO_YOUT_L             0x46
#define GYRO_ZOUT_H             0x47
#define GYRO_ZOUT_L             0x48

#define MPU6050_RANGE_16G       0x03
#define MPU6050_RANGE_8G        0x02
#define MPU6050_RANGE_4G        0x01
#define MPU6050_RANGE_2G        0x00

#define MPU6050_SCALE_250DPS    0x03
#define MPU6050_SCALE_500DPS    0x02
#define MPU6050_SCALE_1000DPS   0x01
#define MPU6050_SCALE_2000DPS   0x00

// I2C2

#define I2C_PCA9685_INSTANCE    2
#define PCA9685_SLAVE_ADDR      0x40

#define MODE1                   0x00
#define MODE2                   0x01

// roda 1 => frente esquerda
// roda 2 => frente direita
// roda 3 => traseira esquerda
// roda 4 => traseira direita


// roda 2

#define LED0_ON_L               0x06
#define LED0_ON_H               0x07
#define LED0_OFF_L              0x08
#define LED0_OFF_H              0x09

// roda 1

#define LED1_ON_L               0x0A
#define LED1_ON_H               0x0B
#define LED1_OFF_L              0x0C
#define LED1_OFF_H              0x0D

// roda 4

#define LED2_ON_L               0x0E
#define LED2_ON_H               0x0F
#define LED2_OFF_L              0x10
#define LED2_OFF_H              0x11

// roda 3

#define LED3_ON_L               0x12
#define LED3_ON_H               0x13
#define LED3_OFF_L              0x14
#define LED3_OFF_H              0x15

#define PRE_SCALE               0xFE
#define PWM_COUNTER_SIZE        4096
#define PWM_DELAY_COUNT         0

// controle

#define Kp                      10
#define Ki                      2
#define Kd                      8

// GPIOs LM393

#define GPIO_REGS_RODA_1        SOC_GPIO_3_REGS
#define GPIO_REGS_RODA_2        SOC_GPIO_1_REGS
#define GPIO_REGS_RODA_3        SOC_GPIO_1_REGS
#define GPIO_REGS_RODA_4        SOC_GPIO_1_REGS

#define GPIO_PIN_RODA_1         21
#define GPIO_PIN_RODA_2         17
#define GPIO_PIN_RODA_3         16
#define GPIO_PIN_RODA_4         28

#define GPIO_INT_LINE_RODA_1    GPIO_INT_LINE_1
#define GPIO_INT_LINE_RODA_2    GPIO_INT_LINE_1
#define GPIO_INT_LINE_RODA_3    GPIO_INT_LINE_1
#define GPIO_INT_LINE_RODA_4    GPIO_INT_LINE_1

// GPIOs Direcao Motores

#define GPIO_REGS_DIR_1         SOC_GPIO_2_REGS
#define GPIO_REGS_DIR_2         SOC_GPIO_2_REGS
#define GPIO_REGS_ESQ_1         SOC_GPIO_2_REGS
#define GPIO_REGS_ESQ_2         SOC_GPIO_2_REGS

#define GPIO_PIN_DIR_1          8
#define GPIO_PIN_DIR_2          8
#define GPIO_PIN_ESQ_1          11
#define GPIO_PIN_ESQ_2          10

// GPIO Controle

#define GPIO_REGS_CONTROLE      SOC_GPIO_0_REGS

#define GPIO_PIN_CONTROLE       27

// GPIO trajetoria

#define GPIO_REGS_TRAJETORIA_1  SOC_GPIO_1_REGS
#define GPIO_REGS_TRAJETORIA_2  SOC_GPIO_1_REGS
#define GPIO_REGS_TRAJETORIA_3  SOC_GPIO_1_REGS
#define GPIO_REGS_TRAJETORIA_4  SOC_GPIO_1_REGS

#define GPIO_PIN_TRAJETORIA_1   21
#define GPIO_PIN_TRAJETORIA_2   22
#define GPIO_PIN_TRAJETORIA_3   23
#define GPIO_PIN_TRAJETORIA_4   24

/*
 *  ======== Board_initI2C ========
 */

static void Board_initGPIO(void)
{
    Board_initCfg boardCfg;

#if defined(SOC_K2H) || defined(SOC_K2K) || defined(SOC_K2E) || defined(SOC_K2L) || defined(SOC_K2G) || defined(SOC_C6678) || defined(SOC_C6657) || defined(SOC_OMAPL137) || defined(SOC_OMAPL138) || defined(SOC_AM65XX) || defined(SOC_J721E)
    GPIO_v0_HwAttrs gpio_cfg;

    /* Get the default SPI init configurations */
    GPIO_socGetInitCfg(GPIO_LED0_PORT_NUM, &gpio_cfg);


#if defined(SOC_K2G)
    /* Setup GPIO interrupt configurations */
    GPIO_socSetIntMux(GPIO_LED0_PORT_NUM, GPIO_LED0_PIN_NUM, NULL, GPIO_MUX_SEL);
#endif
#if defined(SOC_OMAPL137) || defined(SOC_OMAPL138)
    /* Setup GPIO interrupt configurations */
    GPIO_socSetBankInt(GPIO_LED0_PORT_NUM, GPIO_LED0_PIN_NUM, NULL);
#endif
#endif

#if defined(evmK2E) || defined(evmC6678)
    boardCfg = BOARD_INIT_MODULE_CLOCK |
        BOARD_INIT_UART_STDIO;
#else
    boardCfg = BOARD_INIT_PINMUX_CONFIG |
        BOARD_INIT_MODULE_CLOCK |
        BOARD_INIT_UART_STDIO;
#endif
    Board_init(boardCfg);

#if defined(idkAM572x) || defined(idkAM574x)
    GPIOApp_UpdateBoardInfo();
#endif

    /* Modify the default GPIO configurations if necessary */
#if defined (am65xx_evm) || defined (am65xx_idk) || defined (j721e_sim)

    GPIO_configIntRouter(GPIO_LED0_PORT_NUM, GPIO_LED0_PIN_NUM, 0, &gpio_cfg);

    /* Set the default GPIO init configurations */
    GPIO_socSetInitCfg(GPIO_LED0_PORT_NUM, &gpio_cfg);

#endif
}

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/
volatile uint32_t gpio_intr_triggered = 0;
uint32_t gpioBaseAddr;
uint32_t gpioPin;

// variaveis globais

uint32_t qtd_int_roda_1_geral = 0;
uint32_t qtd_int_roda_2_geral = 0;
uint32_t qtd_int_roda_3_geral = 0;
uint32_t qtd_int_roda_4_geral = 0;

uint32_t qtd_int_roda_1 = 0;
uint32_t qtd_int_roda_2 = 0;
uint32_t qtd_int_roda_3 = 0;
uint32_t qtd_int_roda_4 = 0;

uint32_t int_roda_1 = 0;
uint32_t int_roda_2 = 0;
uint32_t int_roda_3 = 0;
uint32_t int_roda_4 = 0;

uint32_t qtd_int_roda_1_anterior = 0;
uint32_t qtd_int_roda_2_anterior = 0;
uint32_t qtd_int_roda_3_anterior = 0;
uint32_t qtd_int_roda_4_anterior = 0;

uint32_t erro_roda_1 = 0;
uint32_t erro_roda_2 = 0;
uint32_t erro_roda_3 = 0;
uint32_t erro_roda_4 = 0;

uint32_t PID_roda_1 = 0;
uint32_t PID_roda_2 = 0;
uint32_t PID_roda_3 = 0;
uint32_t PID_roda_4 = 0;

uint32_t proporcional_1 = 0;
uint32_t proporcional_2 = 0;
uint32_t proporcional_3 = 0;
uint32_t proporcional_4 = 0;

uint32_t integral_1 = 0;
uint32_t integral_2 = 0;
uint32_t integral_3 = 0;
uint32_t integral_4 = 0;

uint32_t derivativo_1 = 0;
uint32_t derivativo_2 = 0;
uint32_t derivativo_3 = 0;
uint32_t derivativo_4 = 0;

uint16_t pwm_roda_1 = 0;
uint16_t pwm_roda_2 = 0;
uint16_t pwm_roda_3 = 0;
uint16_t pwm_roda_4 = 0;

// aceleracao

int value_accel_x = 0;
int value_accel_y = 0;
int value_accel_z = 0;

// velocidade de giro

int value_gyro_x = 0;
int value_gyro_y = 0;
int value_gyro_z = 0;

// posicao

int pos_x = 0;
int pos_y = 0;
int pos_z = 0;

// velocidade

int vel_ant_x = 0;
int vel_ant_y = 0;
int vel_ant_z = 0;

int vel_x = 0;
int vel_y = 0;
int vel_z = 0;

// angulo

int angulo_x = 0;
int angulo_y = 0;
int angulo_z = 0;

float offset_accel_x = 0;
float offset_accel_y = 0;
float offset_accel_z = 0;

float offset_gyro_x = 0;
float offset_gyro_y = 0;
float offset_gyro_z = 0;

int pare = 0;
int trajetoria = 0;
int trecho = 0;

uint32_t int_destino = 0;

Clock_Handle Clock_PID;
Clock_Handle Clock_Espaco;

Semaphore_Handle sem_PID;
Semaphore_Handle sem_Espaco;
Semaphore_Handle sem_ConfigInit;

Task_Handle taskPID;
Task_Handle taskEspaco;
Task_Handle taskConfigInit;

void writeSensor(I2C_Handle h, uint8_t reg, uint8_t val, uint8_t SLAVE_ADDR){
    uint8_t txData[2] = {0,0};
    I2C_Transaction t;
    int16_t transferStatus;
    I2C_transactionInit(&t);
    //memset(&txData, 0x00, sizeof(txData));

    t.slaveAddress = SLAVE_ADDR;
    t.writeBuf = &txData;
    t.writeCount = 2;
    t.readCount = 0;
    t.timeout = 1000U;
    txData[0] = reg;
    txData[1] = val;

    transferStatus = I2C_transfer(h, &t);

    if(I2C_STS_SUCCESS != transferStatus){
       UART_printf("\n Data Transfer failed with transfer status %d \n",transferStatus);
    }
}

uint8_t readSensor(I2C_Handle h, uint8_t reg, uint8_t SLAVE_ADDR){

    uint8_t rxData = 0;
    uint8_t txData = 0;
    I2C_Transaction t;
    int16_t transferStatus;
    I2C_transactionInit(&t);
    memset(&txData, 0x00, sizeof(txData));
    t.slaveAddress = SLAVE_ADDR;
    t.writeBuf = &txData;
    t.writeCount = 1;
    t.readBuf = &rxData;
    t.readCount = 1;
    t.timeout = 1000U;
    txData = reg;
    transferStatus = I2C_transfer(h, &t);
    if(I2C_STS_SUCCESS != transferStatus){
        UART_printf("\n Data Transfer failed with transfer status %d \n",transferStatus);
    }
    return rxData;
}

void IMUSetUp(){

    I2C_Params i2cParams;
    I2C_Handle handle;
    I2C_Params_init(&i2cParams);
    handle = I2C_open(I2C_MPU_6065_INSTANCE, &i2cParams);
    writeSensor(handle, PWR_MGMT_1, 0x00, MPU_6050_SLAVE_ADDR);
    writeSensor(handle, ACCEL_CONFIG, 0x10, MPU_6050_SLAVE_ADDR);
    writeSensor(handle, GYRO_CONFIG, 0x10, MPU_6050_SLAVE_ADDR);
    I2C_close(handle);

}

float absolute(float value){
    if(value < 0){
        return -1 * value;
    }else{
        return value;
    }
}

int round(float value){
    int l = value;
    int u = l + 1;

    if(absolute(value - l) < absolute(value - u)){
        return l;
    }else{
        return u;
    }
}

float getScaleGyro(uint8_t range){

    float result = 0;

    switch (range) {
        case MPU6050_SCALE_250DPS:
            result = 1/131.0;
            break;
        case MPU6050_SCALE_500DPS:
            result = 1/65.5;
            break;
        case MPU6050_SCALE_1000DPS:
            result = 1/32.8;
            break;
        case MPU6050_SCALE_2000DPS:
            result = 1/16.4;
            break;
        default:
            break;
    }

    return result;
}

float getScaleAccel (uint8_t range){

    float result = 0;

    switch (range) {
        case MPU6050_RANGE_2G:
            result = 1/16384.0;
            break;
        case MPU6050_RANGE_4G:
            result = 1/8192.0;
            break;
        case MPU6050_RANGE_8G:
            result = 1/4096.0;
            break;
        case MPU6050_RANGE_16G:
            result = 1/2048.0;
            break;
        default:
            break;
    }

    return result;
}

float ConvertTwosComplementByteToFloatGyro(int rawValue, uint8_t range){
    float result = 0;

    if ((rawValue & 0x00008000) == 0){
        result = ( (float) rawValue) * getScaleGyro(range);
        return result;
    }else{
        rawValue |= 0xffff8000;
        result = ( (float) rawValue) * getScaleGyro(range);
        return result;
    }
}

float ConvertTwosComplementByteToFloatAccel(int rawValue, uint8_t range){
    float result = 0;

    if ((rawValue & 0x00008000) == 0){
        result = ( (float) rawValue) * getScaleAccel(range);
        return result;
    }else{
        rawValue |= 0xffff8000;
        result = ( (float) rawValue) * getScaleAccel(range);
        return result;
    }
}

void calibracao(int qtd_iteracoes){

    float accel_x_acum = 0;
    float accel_y_acum = 0;
    float accel_z_acum = 0;

    float gyro_x_acum = 0;
    float gyro_y_acum = 0;
    float gyro_z_acum = 0;

    I2C_Params i2cParams;
    I2C_Handle handle = NULL;
    I2C_Params_init(&i2cParams);
    handle = I2C_open(I2C_MPU_6065_INSTANCE, &i2cParams);

    for(int i = 0; i < qtd_iteracoes; i++){

        int accel_x_l = readSensor(handle, ACCEL_XOUT_L, MPU_6050_SLAVE_ADDR);
        int accel_x_h = readSensor(handle, ACCEL_XOUT_H, MPU_6050_SLAVE_ADDR);
        int accel_x = (accel_x_h << 8) | accel_x_l;
        float aux_accel_x = ConvertTwosComplementByteToFloatAccel(accel_x, MPU6050_RANGE_8G);
        accel_x_acum += aux_accel_x;

        int accel_y_l = readSensor(handle, ACCEL_YOUT_L, MPU_6050_SLAVE_ADDR);
        int accel_y_h = readSensor(handle, ACCEL_YOUT_H, MPU_6050_SLAVE_ADDR);
        int accel_y = (accel_y_h << 8) | accel_y_l;
        float aux_accel_y = ConvertTwosComplementByteToFloatAccel(accel_y, MPU6050_RANGE_8G);
        accel_y_acum += aux_accel_y;

        int accel_z_l = readSensor(handle, ACCEL_ZOUT_L, MPU_6050_SLAVE_ADDR);
        int accel_z_h = readSensor(handle, ACCEL_ZOUT_H, MPU_6050_SLAVE_ADDR);
        int accel_z = (accel_z_h << 8) | accel_z_l;
        float aux_accel_z = ConvertTwosComplementByteToFloatAccel(accel_z, MPU6050_RANGE_8G);
        accel_z_acum += aux_accel_z;

        int gyro_x_l = readSensor(handle, GYRO_XOUT_L, MPU_6050_SLAVE_ADDR);
        int gyro_x_h = readSensor(handle, GYRO_XOUT_H, MPU_6050_SLAVE_ADDR);
        int gyro_x = (gyro_x_h << 8) | gyro_x_l;
        float aux_gyro_x = ConvertTwosComplementByteToFloatGyro(gyro_x, MPU6050_SCALE_1000DPS);
        gyro_x_acum += aux_gyro_x;

        int gyro_y_l = readSensor(handle, GYRO_YOUT_L, MPU_6050_SLAVE_ADDR);
        int gyro_y_h = readSensor(handle, GYRO_YOUT_H, MPU_6050_SLAVE_ADDR);
        int gyro_y = (gyro_y_h << 8) | gyro_y_l;
        float aux_gyro_y = ConvertTwosComplementByteToFloatGyro(gyro_y, MPU6050_SCALE_1000DPS);
        gyro_y_acum += aux_gyro_y;

        int gyro_z_l = readSensor(handle, GYRO_ZOUT_L, MPU_6050_SLAVE_ADDR);
        int gyro_z_h = readSensor(handle, GYRO_ZOUT_H, MPU_6050_SLAVE_ADDR);
        int gyro_z = (gyro_z_h << 8) | gyro_z_l;
        float aux_gyro_z = ConvertTwosComplementByteToFloatGyro(gyro_z, MPU6050_SCALE_1000DPS);
        gyro_z_acum += aux_gyro_z;
    }

    I2C_close(handle);

    offset_accel_x = round(accel_x_acum / (1.0 * qtd_iteracoes)) - (accel_x_acum / (1.0 * qtd_iteracoes));
    offset_accel_y = round(accel_y_acum / (1.0 * qtd_iteracoes)) - (accel_y_acum / (1.0 * qtd_iteracoes));
    offset_accel_z = round(accel_z_acum / (1.0 * qtd_iteracoes)) - (accel_z_acum / (1.0 * qtd_iteracoes));

    offset_gyro_x = 0 - (gyro_x_acum / (1.0 * qtd_iteracoes));
    offset_gyro_y = 0 - (gyro_y_acum / (1.0 * qtd_iteracoes));
    offset_gyro_z = 0 - (gyro_z_acum / (1.0 * qtd_iteracoes));
}

void readParams(){

    I2C_Params i2cParams;
    I2C_Handle handle = NULL;
    I2C_Params_init(&i2cParams);
    handle = I2C_open(I2C_MPU_6065_INSTANCE, &i2cParams);

    int accel_x_l = readSensor(handle, ACCEL_XOUT_L, MPU_6050_SLAVE_ADDR);
    int accel_x_h = readSensor(handle, ACCEL_XOUT_H, MPU_6050_SLAVE_ADDR);
    int accel_x = (accel_x_h << 8) | accel_x_l;
    float aux_accel_x = ConvertTwosComplementByteToFloatAccel(accel_x, MPU6050_RANGE_8G) + offset_accel_x;
    value_accel_x = (int) 1000 * aux_accel_x;

    int accel_y_l = readSensor(handle, ACCEL_YOUT_L, MPU_6050_SLAVE_ADDR);
    int accel_y_h = readSensor(handle, ACCEL_YOUT_H, MPU_6050_SLAVE_ADDR);
    int accel_y = (accel_y_h << 8) | accel_y_l;
    float aux_accel_y = ConvertTwosComplementByteToFloatAccel(accel_y, MPU6050_RANGE_8G) + offset_accel_y;
    value_accel_y = (int) 1000 * aux_accel_y;

    int accel_z_l = readSensor(handle, ACCEL_ZOUT_L, MPU_6050_SLAVE_ADDR);
    int accel_z_h = readSensor(handle, ACCEL_ZOUT_H, MPU_6050_SLAVE_ADDR);
    int accel_z = (accel_z_h << 8) | accel_z_l;
    float aux_accel_z = ConvertTwosComplementByteToFloatAccel(accel_z, MPU6050_RANGE_8G) + offset_accel_z;
    value_accel_z = (int) 1000 * aux_accel_z;

    int gyro_x_l = readSensor(handle, GYRO_XOUT_L, MPU_6050_SLAVE_ADDR);
    int gyro_x_h = readSensor(handle, GYRO_XOUT_H, MPU_6050_SLAVE_ADDR);
    int gyro_x = (gyro_x_h << 8) | gyro_x_l;
    float aux_gyro_x = ConvertTwosComplementByteToFloatGyro(gyro_x, MPU6050_SCALE_1000DPS) + offset_gyro_x;
    value_gyro_x = (int) 1000 * aux_gyro_x;

    int gyro_y_l = readSensor(handle, GYRO_YOUT_L, MPU_6050_SLAVE_ADDR);
    int gyro_y_h = readSensor(handle, GYRO_YOUT_H, MPU_6050_SLAVE_ADDR);
    int gyro_y = (gyro_y_h << 8) | gyro_y_l;
    float aux_gyro_y = ConvertTwosComplementByteToFloatGyro(gyro_y, MPU6050_SCALE_1000DPS) + offset_gyro_y;
    value_gyro_y = (int) 1000 * aux_gyro_y;

    int gyro_z_l = readSensor(handle, GYRO_ZOUT_L, MPU_6050_SLAVE_ADDR);
    int gyro_z_h = readSensor(handle, GYRO_ZOUT_H, MPU_6050_SLAVE_ADDR);
    int gyro_z = (gyro_z_h << 8) | gyro_z_l;
    float aux_gyro_z = ConvertTwosComplementByteToFloatGyro(gyro_z, MPU6050_SCALE_1000DPS) + offset_gyro_z;
    value_gyro_z = (int) 1000 * aux_gyro_z;

    I2C_close(handle);

    if((absolute(value_gyro_x) < ERRO) && (absolute(value_gyro_y) < ERRO) && (absolute(value_gyro_z) < ERRO)){
        angulo_x += 0;
        angulo_y += 0;
        angulo_z += 0;
    }else{
        angulo_x += value_gyro_x * 0.1;
        angulo_y += value_gyro_y * 0.1;
        angulo_z += value_gyro_z * 0.1;
    }

    if(((aux_accel_x * aux_accel_x + aux_accel_y * aux_accel_y + aux_accel_z * aux_accel_z) >= ((1 - ERRO) * (1 - ERRO))) && (aux_accel_x * aux_accel_x + aux_accel_y * aux_accel_y + aux_accel_z * aux_accel_z) <= ((1 + ERRO) * (1 + ERRO))){
        vel_x = 0;
        vel_y = 0;
        vel_z = 0;
    }else{
        vel_x += value_accel_x * 0.1 * 9.8;
        vel_y += value_accel_y * 0.1 * 9.8;
        vel_z += value_accel_z * 0.1 * 9.8;

        pos_x += (vel_x - vel_ant_x) * 0.1;
        pos_y += (vel_y - vel_ant_y) * 0.1;
        pos_z += (vel_z - vel_ant_z) * 0.1;
    }

    vel_ant_x = vel_x;
    vel_ant_y = vel_y;
    vel_ant_z = vel_z;

}

void SetupPCA(uint16_t pwm_config_roda_1, uint16_t pwm_config_roda_2, uint16_t pwm_config_roda_3, uint16_t pwm_config_roda_4){

    if(pwm_config_roda_1 > PWM){
        pwm_config_roda_1 = PWM;
    }

    if(pwm_config_roda_2 > PWM){
        pwm_config_roda_2 = PWM;
    }

    if(pwm_config_roda_3 > PWM){
        pwm_config_roda_3 = PWM;
    }

    if(pwm_config_roda_4 > PWM){
        pwm_config_roda_4 = PWM;
    }

    if(pwm_config_roda_1 < 0){
        pwm_config_roda_1 = 0;
    }

    if(pwm_config_roda_2 < 0){
        pwm_config_roda_2 = 0;
    }

    if(pwm_config_roda_3 < 0){
        pwm_config_roda_3 = 0;
    }

    if(pwm_config_roda_4 < 0){
        pwm_config_roda_4 = 0;
    }

    uint8_t pwm_l_roda_1 = pwm_config_roda_1 & 0x00FF;
    uint8_t pwm_h_roda_1 = (pwm_config_roda_1 >> 8);

    uint8_t pwm_l_roda_2 = pwm_config_roda_2 & 0x00FF;
    uint8_t pwm_h_roda_2 = (pwm_config_roda_2 >> 8);

    uint8_t pwm_l_roda_3 = pwm_config_roda_3 & 0x00FF;
    uint8_t pwm_h_roda_3 = (pwm_config_roda_3 >> 8);

    uint8_t pwm_l_roda_4 = pwm_config_roda_4 & 0x00FF;
    uint8_t pwm_h_roda_4 = (pwm_config_roda_4 >> 8);

    I2C_Params i2cParams;
    I2C_Handle handle = NULL;

    I2C_Params_init(&i2cParams);
    handle = I2C_open(I2C_PCA9685_INSTANCE, &i2cParams);
    writeSensor(handle, MODE1, 0x10, PCA9685_SLAVE_ADDR);
    Task_sleep(1);

    writeSensor(handle, PRE_SCALE, 0x78, PCA9685_SLAVE_ADDR);

    writeSensor(handle, LED0_ON_H, 0x00, PCA9685_SLAVE_ADDR);
    writeSensor(handle, LED0_ON_L, 0x00, PCA9685_SLAVE_ADDR);
    writeSensor(handle, LED0_OFF_H, pwm_h_roda_2, PCA9685_SLAVE_ADDR);
    writeSensor(handle, LED0_OFF_L, pwm_l_roda_2, PCA9685_SLAVE_ADDR);

    writeSensor(handle, LED1_ON_H, 0x00, PCA9685_SLAVE_ADDR);
    writeSensor(handle, LED1_ON_L, 0x00, PCA9685_SLAVE_ADDR);
    writeSensor(handle, LED1_OFF_H, pwm_h_roda_1, PCA9685_SLAVE_ADDR);
    writeSensor(handle, LED1_OFF_L, pwm_l_roda_1, PCA9685_SLAVE_ADDR);

    writeSensor(handle, LED2_ON_H, 0x00, PCA9685_SLAVE_ADDR);
    writeSensor(handle, LED2_ON_L, 0x00, PCA9685_SLAVE_ADDR);
    writeSensor(handle, LED2_OFF_H, pwm_h_roda_4, PCA9685_SLAVE_ADDR);
    writeSensor(handle, LED2_OFF_L, pwm_l_roda_4, PCA9685_SLAVE_ADDR);

    writeSensor(handle, LED3_ON_H, 0x00, PCA9685_SLAVE_ADDR);
    writeSensor(handle, LED3_ON_L, 0x00, PCA9685_SLAVE_ADDR);
    writeSensor(handle, LED3_OFF_H, pwm_h_roda_3, PCA9685_SLAVE_ADDR);
    writeSensor(handle, LED3_OFF_L, pwm_l_roda_3, PCA9685_SLAVE_ADDR);

    writeSensor(handle, MODE1, 0x00, PCA9685_SLAVE_ADDR);

    I2C_close(handle);
}

uint32_t min(uint32_t x, uint32_t y){
    if(x < y){
        return x;
    }else{
        return y;
    }
}

void Espaco_CONTROLE(){

    switch (trajetoria) {
        case 1:
            if(min(qtd_int_roda_1_geral, qtd_int_roda_2_geral) >= int_destino){
                pare = 1;
            }
            break;
        case 2:
            if(min(qtd_int_roda_1_geral, qtd_int_roda_2_geral) >= int_destino){
                pare = 1;
            }
            break;
        case 3:
            if(trecho == 0){
                if(qtd_int_roda_1_geral >= int_destino){
                    trecho++;
                    GPIOPinWrite(SOC_GPIO_2_REGS, 7, 1);
                    GPIOPinWrite(SOC_GPIO_2_REGS, 9, 1);
                    GPIOPinWrite(SOC_GPIO_2_REGS, 11, 1);
                    GPIOPinWrite(SOC_GPIO_2_REGS, 13, 1);

                    pwm_roda_1 = 0;
                    pwm_roda_2 = 0;
                    pwm_roda_3 = 0;
                    pwm_roda_4 = 0;

                    SetupPCA(pwm_roda_1, pwm_roda_2, pwm_roda_3, pwm_roda_4);

                    qtd_int_roda_1_geral = 0;
                    qtd_int_roda_2_geral = 0;
                    qtd_int_roda_3_geral = 0;
                    qtd_int_roda_4_geral = 0;

                    qtd_int_roda_1 = 0;
                    qtd_int_roda_2 = 0;
                    qtd_int_roda_3 = 0;
                    qtd_int_roda_4 = 0;

                    GPIOPinWrite(SOC_GPIO_2_REGS, 7, 0);
                    GPIOPinWrite(SOC_GPIO_2_REGS, 9, 0);
                    GPIOPinWrite(SOC_GPIO_2_REGS, 11, 0);
                    GPIOPinWrite(SOC_GPIO_2_REGS, 13, 0);
                }
            }else{
                if(qtd_int_roda_2_geral >= int_destino){
                    pare = 1;
                }
            }
            break;
        case 4:
            break;

        default:
            break;
    }

    if(pare == 1){

        GPIOPinWrite(SOC_GPIO_2_REGS, 7, 1);
        GPIOPinWrite(SOC_GPIO_2_REGS, 9, 1);
        GPIOPinWrite(SOC_GPIO_2_REGS, 11, 1);
        GPIOPinWrite(SOC_GPIO_2_REGS, 13, 1);

        pwm_roda_1 = 0;
        pwm_roda_2 = 0;
        pwm_roda_3 = 0;
        pwm_roda_4 = 0;

        SetupPCA(pwm_roda_1, pwm_roda_2, pwm_roda_3, pwm_roda_4);

        Semaphore_post(sem_ConfigInit);
    }
}

void PID_CONTROLE(){


    int_roda_4 = qtd_int_roda_4;
    int_roda_1 = qtd_int_roda_1;
    int_roda_3 = qtd_int_roda_3;
    int_roda_2 = qtd_int_roda_2;

    UART_printf("-------------------------\n");
    UART_printf("QTD_INT_RODA_1: %d\n", int_roda_1);
    UART_printf("QTD_INT_RODA_2: %d\n", int_roda_2);
    UART_printf("QTD_INT_RODA_3: %d\n", int_roda_3);
    UART_printf("QTD_INT_RODA_4: %d\n", int_roda_4);

    qtd_int_roda_1 = 0;
    qtd_int_roda_2 = 0;
    qtd_int_roda_3 = 0;
    qtd_int_roda_4 = 0;


    switch (trajetoria) {
        case 1:
            erro_roda_1 = QTD_IDEAL_INT_100m - int_roda_1;
            erro_roda_2 = QTD_IDEAL_INT_100m - int_roda_2;
            erro_roda_3 = QTD_IDEAL_INT_100m - int_roda_3;
            erro_roda_4 = QTD_IDEAL_INT_100m - int_roda_4;
            break;
        case 2:
            erro_roda_1 = QTD_IDEAL_INT_100m - int_roda_1;
            erro_roda_2 = QTD_IDEAL_INT_100m - int_roda_2;
            erro_roda_3 = QTD_IDEAL_INT_100m - int_roda_3;
            erro_roda_4 = QTD_IDEAL_INT_100m - int_roda_4;
            break;

        case 3:
            erro_roda_1 = 17 - int_roda_1;
            erro_roda_2 = 20 - int_roda_2;
            erro_roda_3 = 17 - int_roda_3;
            erro_roda_4 = 20 - int_roda_4;
            break;
        default:
            break;
    }

    proporcional_1 = erro_roda_1 * Kp;
    proporcional_2 = erro_roda_2 * Kp;
    proporcional_3 = erro_roda_3 * Kp;
    proporcional_4 = erro_roda_4 * Kp;

    integral_1 += erro_roda_1 * Ki;
    integral_2 += erro_roda_2 * Ki;
    integral_3 += erro_roda_3 * Ki;
    integral_4 += erro_roda_4 * Ki;

    derivativo_1 = (qtd_int_roda_1_anterior - int_roda_1) * Kd;
    derivativo_2 = (qtd_int_roda_2_anterior - int_roda_2) * Kd;
    derivativo_3 = (qtd_int_roda_3_anterior - int_roda_3) * Kd;
    derivativo_4 = (qtd_int_roda_4_anterior - int_roda_4) * Kd;
 
    PID_roda_1 = proporcional_1 + integral_1 + derivativo_1;
    PID_roda_2 = proporcional_2 + integral_2 + derivativo_2;
    PID_roda_3 = proporcional_3 + integral_3 + derivativo_3;
    PID_roda_4 = proporcional_4 + integral_4 + derivativo_4;

    pwm_roda_1 += PID_roda_1;
    pwm_roda_2 += PID_roda_2;
    pwm_roda_3 += PID_roda_3;
    pwm_roda_4 += PID_roda_4;

    SetupPCA(pwm_roda_1, pwm_roda_2, pwm_roda_3, pwm_roda_4);

    qtd_int_roda_1_anterior = int_roda_1;
    qtd_int_roda_2_anterior = int_roda_2;
    qtd_int_roda_3_anterior = int_roda_3;
    qtd_int_roda_4_anterior = int_roda_4;

}

void isrFunc(){

    if(GPIOPinIntStatus(GPIO_REGS_RODA_1, GPIO_INT_LINE_RODA_1, GPIO_PIN_RODA_1)){
        qtd_int_roda_1_geral++;
        qtd_int_roda_1++;
        GPIOPinIntClear(GPIO_REGS_RODA_1, GPIO_INT_LINE_RODA_1, GPIO_PIN_RODA_1);
    }

    if(GPIOPinIntStatus(GPIO_REGS_RODA_2, GPIO_INT_LINE_RODA_2, GPIO_PIN_RODA_2)){
        qtd_int_roda_2_geral++;
        qtd_int_roda_2++;
        GPIOPinIntClear(GPIO_REGS_RODA_2, GPIO_INT_LINE_RODA_2, GPIO_PIN_RODA_2);
    }

    if(GPIOPinIntStatus(GPIO_REGS_RODA_3, GPIO_INT_LINE_RODA_3, GPIO_PIN_RODA_3)){
        qtd_int_roda_3_geral++;
        qtd_int_roda_3++;
        GPIOPinIntClear(GPIO_REGS_RODA_3, GPIO_INT_LINE_RODA_3, GPIO_PIN_RODA_3);
    }

    if(GPIOPinIntStatus(GPIO_REGS_RODA_4, GPIO_INT_LINE_RODA_4, GPIO_PIN_RODA_4)){
        qtd_int_roda_4_geral++;
        qtd_int_roda_4++;
        GPIOPinIntClear(GPIO_REGS_RODA_4, GPIO_INT_LINE_RODA_4, GPIO_PIN_RODA_4);
    }
}

void swiFuncPID(){
    if(pare == 0){
        Semaphore_post(sem_PID);
    }
}

void swiFuncEspaco(){
    if(pare == 0){
        Semaphore_post(sem_Espaco);
    }
}

void PID(){

    Semaphore_pend(sem_PID, BIOS_WAIT_FOREVER);

    while(1){
        PID_CONTROLE();
        readParams();
        Semaphore_pend(sem_PID, BIOS_WAIT_FOREVER);
    }

}

uint32_t calc_int_destino(float distancia_cm){
    float aux = distancia_cm / (2 * Pi * Raio);
    aux *= 20;
    uint32_t qtd_int_ = aux;
    return qtd_int_;
}

void ConfigInit(){

    pare = 1;

    UART_printf("Config MPU...\n");

    IMUSetUp();

    UART_printf("Config MPU\n");

    UART_printf("Calibration...\n");

    calibracao(1000);

    UART_printf("Calibration\n");

    GPIOPinWrite(SOC_GPIO_2_REGS, 6, 1);
    GPIOPinWrite(SOC_GPIO_2_REGS, 7, 1);
    GPIOPinWrite(SOC_GPIO_2_REGS, 8, 1);
    GPIOPinWrite(SOC_GPIO_2_REGS, 9, 1);
    GPIOPinWrite(SOC_GPIO_2_REGS, 10, 1);
    GPIOPinWrite(SOC_GPIO_2_REGS, 11, 1);
    GPIOPinWrite(SOC_GPIO_2_REGS, 12, 1);
    GPIOPinWrite(SOC_GPIO_2_REGS, 13, 1);

    pwm_roda_1 = 0;
    pwm_roda_2 = 0;
    pwm_roda_3 = 0;
    pwm_roda_4 = 0;

    UART_printf("Config PCA...\n");

    SetupPCA(pwm_roda_1, pwm_roda_2, pwm_roda_3, pwm_roda_4);

    UART_printf("Config PCA\n");

    trajetoria = 0;

    while(1){

        GPIOPinWrite(SOC_GPIO_1_REGS, 21, GPIO_PIN_LOW);
        GPIOPinWrite(SOC_GPIO_1_REGS, 22, GPIO_PIN_LOW);
        GPIOPinWrite(SOC_GPIO_1_REGS, 23, GPIO_PIN_LOW);
        GPIOPinWrite(SOC_GPIO_1_REGS, 24, GPIO_PIN_LOW);

        //UART_scanFmt("%d", &trajetoria);

        //trajetoria = (trajetoria % 4) + 1;

        trajetoria++;

        UART_printf("Select trajetoria\n");

        while((trajetoria < 1) || (trajetoria > 1)){
            UART_printf("Trajetoria inválida\n");
            UART_scanFmt("%d", &trajetoria);
        }

        UART_printf("Config trajetoria\n");

        switch (trajetoria) {
            case 1:
                GPIOPinWrite(SOC_GPIO_1_REGS, 21, GPIO_PIN_HIGH);
                int_destino = calc_int_destino(100);
                break;
            case 2:
                GPIOPinWrite(SOC_GPIO_1_REGS, 22, GPIO_PIN_HIGH);
                int_destino = calc_int_destino(200);
                break;
            case 3:
                GPIOPinWrite(SOC_GPIO_1_REGS, 23, GPIO_PIN_HIGH);
                int_destino = calc_int_destino(Pi * 92);
                break;
            case 4:
                GPIOPinWrite(SOC_GPIO_1_REGS, 24, GPIO_PIN_HIGH);
                trecho = 0;
                int_destino = calc_int_destino(Pi * 92);
                break;
            default:
                break;
        }

        pwm_roda_1 = PWM_init;
        pwm_roda_2 = PWM_init;
        pwm_roda_3 = PWM_init;
        pwm_roda_4 = PWM_init;

        qtd_int_roda_1 = 0;
        qtd_int_roda_2 = 0;
        qtd_int_roda_3 = 0;
        qtd_int_roda_4 = 0;

        qtd_int_roda_1_geral = 0;
        qtd_int_roda_2_geral = 0;
        qtd_int_roda_3_geral = 0;
        qtd_int_roda_4_geral = 0;

        SetupPCA(pwm_roda_1, pwm_roda_2, pwm_roda_3, pwm_roda_4);

        GPIOPinWrite(SOC_GPIO_2_REGS, 7, 0);
        GPIOPinWrite(SOC_GPIO_2_REGS, 9, 0);
        GPIOPinWrite(SOC_GPIO_2_REGS, 11, 0);
        GPIOPinWrite(SOC_GPIO_2_REGS, 13, 0);

        pare = 0;

        Semaphore_pend(sem_ConfigInit, BIOS_WAIT_FOREVER);
    }
}

void Espaco(){

    Semaphore_pend(sem_Espaco, BIOS_WAIT_FOREVER);

    while(1){
        /*
        UART_printf("-------------------------\n");

        UART_printf("PWM_RODA_1: %d\n", pwm_roda_1);
        UART_printf("PWM_RODA_2: %d\n", pwm_roda_2);
        UART_printf("PWM_RODA_3: %d\n", pwm_roda_3);
        UART_printf("PWM_RODA_4: %d\n", pwm_roda_4);

        UART_printf("PID_RODA_1: %d\n", PID_roda_1);
        UART_printf("PID_RODA_2: %d\n", PID_roda_2);
        UART_printf("PID_RODA_3: %d\n", PID_roda_3);
        UART_printf("PID_RODA_4: %d\n", PID_roda_4);
         */
        Espaco_CONTROLE();
        Semaphore_pend(sem_Espaco, BIOS_WAIT_FOREVER);
    }

}

#ifdef USE_BIOS
/*
 *  ======== main ========
 */
int main(void)
{
 #if defined(SOC_AM65XX) || defined(SOC_J721E)
    Sciclient_ConfigPrms_t  sciClientCfg;
#endif

#if defined(SOC_AM65XX) || defined(SOC_J721E)
    Sciclient_configPrmsInit(&sciClientCfg);
    Sciclient_init(&sciClientCfg);
#endif
    /* Call board init functions */
    Board_initGPIO();

#if defined(idkAM574x) || defined(idkAM572x) || defined(idkAM571x)
    AppGPIOInit();
#endif

    UART_printf("Config...\n");

    I2C_HwAttrs   i2c_cfg;

    I2C_socGetInitCfg(I2C_MPU_6065_INSTANCE, &i2c_cfg);
    I2C_socSetInitCfg(I2C_MPU_6065_INSTANCE, &i2c_cfg);

    I2C_socGetInitCfg(I2C_PCA9685_INSTANCE, &i2c_cfg);
    I2C_socSetInitCfg(I2C_PCA9685_INSTANCE, &i2c_cfg);

    I2C_init();


    /* Create Clock */

    Clock_Params clkParamsClockPID;
    Clock_Params_init(&clkParamsClockPID);
    clkParamsClockPID.startFlag = 1;
    clkParamsClockPID.period = 100;

    Clock_PID = Clock_create(swiFuncPID, 100, &clkParamsClockPID, NULL);

    Clock_Params clkParamsEspaco;
    Clock_Params_init(&clkParamsEspaco);
    clkParamsEspaco.startFlag = 1;
    clkParamsEspaco.period = 500;

    Clock_Espaco = Clock_create(swiFuncEspaco, 500, &clkParamsEspaco, NULL);


    /* Create Semaphore */

    Semaphore_Params semParamsPID;
    Semaphore_Params_init(&semParamsPID);
    semParamsPID.mode = Semaphore_Mode_BINARY;

    sem_PID = Semaphore_create(0, &semParamsPID, NULL);

    Semaphore_Params semParamsEspaco;
    Semaphore_Params_init(&semParamsEspaco);
    semParamsEspaco.mode = Semaphore_Mode_BINARY;

    sem_Espaco = Semaphore_create(0, &semParamsEspaco, NULL);

    Semaphore_Params semParamsConfigInit;
    Semaphore_Params_init(&semParamsConfigInit);
    semParamsConfigInit.mode = Semaphore_Mode_BINARY;

    sem_ConfigInit = Semaphore_create(0, &semParamsConfigInit, NULL);

    /* Config GPIO */

    GPIODirModeSet(SOC_GPIO_2_REGS, 6, GPIO_CFG_OUTPUT);
    GPIODirModeSet(SOC_GPIO_2_REGS, 7, GPIO_CFG_OUTPUT);
    GPIODirModeSet(SOC_GPIO_2_REGS, 8, GPIO_CFG_OUTPUT);
    GPIODirModeSet(SOC_GPIO_2_REGS, 9, GPIO_CFG_OUTPUT);
    GPIODirModeSet(SOC_GPIO_2_REGS, 10, GPIO_CFG_OUTPUT);
    GPIODirModeSet(SOC_GPIO_2_REGS, 11, GPIO_CFG_OUTPUT);
    GPIODirModeSet(SOC_GPIO_2_REGS, 12, GPIO_CFG_OUTPUT);
    GPIODirModeSet(SOC_GPIO_2_REGS, 13, GPIO_CFG_OUTPUT);

    GPIODirModeSet(SOC_GPIO_1_REGS, 21, GPIO_CFG_OUTPUT);
    GPIODirModeSet(SOC_GPIO_1_REGS, 22, GPIO_CFG_OUTPUT);
    GPIODirModeSet(SOC_GPIO_1_REGS, 23, GPIO_CFG_OUTPUT);
    GPIODirModeSet(SOC_GPIO_1_REGS, 24, GPIO_CFG_OUTPUT);

    /* Create Tasks */

    Task_Params taskParamsPID;
    Task_Params taskParamsEspaco;
    Task_Params taskParamsConfigInit;

    Task_Params_init(&taskParamsPID);
    Task_Params_init(&taskParamsEspaco);
    Task_Params_init(&taskParamsConfigInit);

    taskParamsPID.stackSize = 0x1400;
    taskParamsEspaco.stackSize = 0x1400;
    taskParamsConfigInit.stackSize = 0x1400;

    taskParamsPID.priority = 12;
    taskParamsEspaco.priority = 5;
    taskParamsConfigInit.priority = 14;

    taskPID = Task_create(PID, &taskParamsPID, NULL);
    taskEspaco = Task_create(Espaco, &taskParamsEspaco, NULL);
    taskConfigInit = Task_create(ConfigInit, &taskParamsConfigInit, NULL);

    /* Create Hwi*/

    Hwi_Params hwiParams;
    Hwi_Params_init(&hwiParams);

    /* Create parameters */

    hwiParams.enableInt = FALSE;
    Hwi_create(62, isrFunc, &hwiParams, NULL);
    Hwi_create(63, isrFunc, &hwiParams, NULL);
    Hwi_create(98, isrFunc, &hwiParams, NULL);
    Hwi_create(99, isrFunc, &hwiParams, NULL);

    /* enable interrupts */

    Hwi_enableInterrupt(62);
    Hwi_enableInterrupt(63);
    Hwi_enableInterrupt(98);
    Hwi_enableInterrupt(99);

    GPIODirModeSet(GPIO_REGS_RODA_1, GPIO_PIN_RODA_1, GPIO_CFG_INPUT);
    GPIOIntTypeSet(GPIO_REGS_RODA_1, GPIO_PIN_RODA_1, GPIO_INT_TYPE_RISE_EDGE);
    GPIOPinIntEnable(GPIO_REGS_RODA_1, GPIO_INT_LINE_RODA_1, GPIO_PIN_RODA_1);

    GPIODirModeSet(GPIO_REGS_RODA_2, GPIO_PIN_RODA_2, GPIO_CFG_INPUT);
    GPIOIntTypeSet(GPIO_REGS_RODA_2, GPIO_PIN_RODA_2, GPIO_INT_TYPE_RISE_EDGE);
    GPIOPinIntEnable(GPIO_REGS_RODA_2, GPIO_INT_LINE_RODA_2, GPIO_PIN_RODA_2);

    GPIODirModeSet(GPIO_REGS_RODA_3, GPIO_PIN_RODA_3, GPIO_CFG_INPUT);
    GPIOIntTypeSet(GPIO_REGS_RODA_3, GPIO_PIN_RODA_3, GPIO_INT_TYPE_RISE_EDGE);
    GPIOPinIntEnable(GPIO_REGS_RODA_3, GPIO_INT_LINE_RODA_3, GPIO_PIN_RODA_3);

    GPIODirModeSet(GPIO_REGS_RODA_4, GPIO_PIN_RODA_4, GPIO_CFG_INPUT);
    GPIOIntTypeSet(GPIO_REGS_RODA_4, GPIO_PIN_RODA_4, GPIO_INT_TYPE_RISE_EDGE);
    GPIOPinIntEnable(GPIO_REGS_RODA_4, GPIO_INT_LINE_RODA_4, GPIO_PIN_RODA_4);

    UART_printf("Config\n");

    /* Start BIOS */
    BIOS_start();
    return (0);
}
#endif

/*
 *  ======== AppDelay ========
 */
void AppDelay(unsigned int delayVal)
{
    Osal_delay(delayVal);
}

/*
 *  ======== AppLoopDelay ========
 */
void AppLoopDelay(uint32_t delayVal)
{
    volatile uint32_t i;

    for (i = 0; i < (delayVal * 1000); i++)
        ;
}

/*
 *  ======== Callback function ========
 */
void AppGpioCallbackFxn(void)
{
    /* Toggle LED1 */
    GPIO_toggle(USER_LED1);
    AppLoopDelay(DELAY_VALUE);
    gpio_intr_triggered = 1;
}



