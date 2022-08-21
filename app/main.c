// Copyright 2022 PurplePopo <PurplePopo(at)outlook.com>
#include "fw_hal.h"
#include "mpu6050.h"
#include "ESP01S_AT_CmdTable.h"
#include "ANO_TC_ProtocolDef.h"

#define TMCLK_HZ(x)             (1000 / (x))
#define UART_RX_IDLE_TIMEOUT_MS (20)

#define LED1_ON()    P11 = RESET
#define LED1_OFF()   P11 = SET
#define LED2_ON()    P11 = RESET
#define LED2_OFF()   P11 = SET
#define LED3_ON()    P11 = RESET
#define LED3_OFF()   P11 = SET
#define LED4_ON()    P11 = RESET
#define LED4_OFF()   P11 = SET

static volatile uint16_t g_timestamp = 0;
static volatile uint16_t g_pwmduty_motor1 = 0;
static volatile uint16_t g_pwmduty_motor2 = 0;
static volatile uint16_t g_pwmduty_motor3 = 0;
static volatile uint16_t g_pwmduty_motor4 = 0;

extern char UART1_RxBuffer[UART_RX_BUFF_SIZE];
extern __BIT busy;
__BIT RxTimeout;
uint8_t RxTimeoutTicks = 0;

INTERRUPT(UART1_Routine, EXTI_VectUART1) {
    if (TI) {
        UART1_ClearTxInterrupt();
        busy = 0;
    }
    if (RI) {
        UART1_ClearRxInterrupt();
        UART1_RxBuffer[rptr++] = SBUF;
        rptr = rptr % UART_RX_BUFF_SIZE;
        RxTimeoutTicks = 0;
    }
}

INTERRUPT(Timer0_Routine, EXTI_VectTimer0) {
    g_timestamp++;
    if (g_timestamp >= 1000) {
        g_timestamp = 0;
    }
    if (RxTimeoutTicks <= UART_RX_IDLE_TIMEOUT_MS) {
        RxTimeoutTicks++;
    } else {
        RxTimeout = 1;
    }
    WDT_ResetCounter();
}

static void GPIO_Init(void) {
    GPIO_P3_SetMode(GPIO_Pin_All, GPIO_Mode_Input_HIP);
    // SDA
    GPIO_P3_SetMode(GPIO_Pin_3, GPIO_Mode_InOut_QBD);
    // SCL
    GPIO_P3_SetMode(GPIO_Pin_2, GPIO_Mode_Output_PP);
    // LED
    GPIO_P1_SetMode(GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_7, GPIO_Mode_Output_PP);
    // PWM
    GPIO_P1_SetMode(GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_6, GPIO_Mode_Output_PP);
    // charge
    GPIO_P5_SetMode(GPIO_Pin_4, GPIO_Mode_InOut_QBD);
}

// MPU6050 low power mode, freq 40Hz, default param: gyro 500 dps / acc 4g
static void Hal_MPU6050_Init(void) {
    // Master mode
    I2C_SetWorkMode(I2C_WorkMode_Master);
    /**
     * I2C clock = FOSC / 2 / (__prescaler__ * 2 + 4)
     * MPU6050 works with i2c clock up to 400KHz
     * 
     * 44.2368 / 2 / (26 * 2 + 4) = 0.39 MHz
    */
    I2C_SetClockPrescaler(0x1A);
    // Switch alternative port
    I2C_SetPort(I2C_AlterPort_P32_P33);
    // Start I2C
    I2C_SetEnabled(HAL_State_ON);
    MPU6050_Init();
    MPU6050_EnableLowPowerMode(MPU6050_Wakeup_Freq_40Hz);
}

static void Hal_ESP01S_Init(void) {
    UART1_Config8bitUart(UART1_BaudSource_Timer1, HAL_State_ON, 115200);
    UART1_SwitchPort(UART1_AlterPort_P36_P37);
    EXTI_Global_SetIntState(HAL_State_ON);
    EXTI_UART1_SetIntState(HAL_State_ON);
}

static void Hal_MotorPWM_Init(void) {
    PWMA_PWM1_SetPortState(HAL_State_OFF);
    PWMA_PWM2_SetPortState(HAL_State_OFF);
    PWMA_PWM3_SetPortState(HAL_State_OFF);
    PWMA_PWM4_SetPortState(HAL_State_OFF);
    PWMA_PWM1_SetPortDirection(PWMB_PortDirOut);
    PWMA_PWM2_SetPortDirection(PWMB_PortDirOut);
    PWMA_PWM3_SetPortDirection(PWMB_PortDirOut);
    PWMA_PWM4_SetPortDirection(PWMB_PortDirOut);
    PWMA_PWM1_ConfigOutputMode(PWM_OutputMode_PWM_LowIfLess);
    PWMA_PWM2_ConfigOutputMode(PWM_OutputMode_PWM_LowIfLess);
    PWMA_PWM3_ConfigOutputMode(PWM_OutputMode_PWM_LowIfLess);
    PWMA_PWM4_ConfigOutputMode(PWM_OutputMode_PWM_LowIfLess);
    PWMA_PWM1_SetComparePreload(HAL_State_ON);
    PWMA_PWM2_SetComparePreload(HAL_State_ON);
    PWMA_PWM3_SetComparePreload(HAL_State_ON);
    PWMA_PWM4_SetComparePreload(HAL_State_ON);
    PWMA_PWM1_SetPortState(HAL_State_ON);
    PWMA_PWM2_SetPortState(HAL_State_ON);
    PWMA_PWM3_SetPortState(HAL_State_ON);
    PWMA_PWM4_SetPortState(HAL_State_ON);
    PWMA_SetPrescaler(0);
    PWMA_SetPeriod(0xFF);
    PWMA_SetCounterDirection(PWM_CounterDirection_Down);
    PWMA_SetAutoReloadPreload(HAL_State_ON);
    PWMA_SetPinOutputState(PWM_Pin_1 | PWM_Pin_2 | PWM_Pin_3 | PWM_Pin_4, HAL_State_ON);
    PWMA_PWM1_SetPort(PWMA_PWM1_AlterPort_P10_P11);
    PWMA_PWM2_SetPort(PWMA_PWM2_AlterPort_P12P54_P13);
    PWMA_PWM3_SetPort(PWMA_PWM3_AlterPort_P14_P15);
    PWMA_PWM4_SetPort(PWMA_PWM4_AlterPort_P16_P17);
    PWMA_SetOverallState(HAL_State_ON);
    PWMA_SetCounterState(HAL_State_ON);
}

static void Hal_Timestamp_Init(void) {
    // Timer0: 12T mode, frequency 1000Hz
    TIM_Timer0_Config(HAL_State_OFF, TIM_TimerMode_16BitAuto, 1000);
    EXTI_Timer0_SetIntState(HAL_State_ON);
    EXTI_Timer0_SetIntPriority(EXTI_IntPriority_High);
    EXTI_Global_SetIntState(HAL_State_ON);
    TIM_Timer0_SetRunState(HAL_State_ON);
    // watch dog
    WDT_SetCounterPrescaler(0x07);
    WDT_StartWatchDog();
}

void main(void) {
    uint16_t buf[7] = { 0 };

    SYS_SetClock();
    GPIO_Init();

    Hal_MPU6050_Init();
    Hal_ESP01S_Init();
    Hal_MotorPWM_Init();
    Hal_Timestamp_Init();

    while (1) {
        switch (g_timestamp) {
            case TMCLK_HZ(20) :     // wifi transmission
                UART1_IntTxString(" string\r\n");
                break;
            case TMCLK_HZ(40) :     // sampling and calculation
                MPU6050_ReadAll(buf);
                break;
            case TMCLK_HZ(100) :    // motor ctrl
                PWMA_PWM1_SetCaptureCompareValue(g_pwmduty_motor1);
                PWMA_PWM1_SetCaptureCompareValue(g_pwmduty_motor2);
                PWMA_PWM1_SetCaptureCompareValue(g_pwmduty_motor3);
                PWMA_PWM1_SetCaptureCompareValue(g_pwmduty_motor4);
                break;
            default: break;
        }
    }
}
