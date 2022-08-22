// Copyright 2022 PurplePopo <PurplePopo(at)outlook.com>
#include <math.h>
#include <string.h>
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

typedef struct _tAttitude {
    float yaw;
    float roll;
    float pitch;
} tAttitude;

enum {
    eWIFISTATE_DISCONNECTED = 0,
    eWIFISTATE_CONN_AND_READ,
    eWIFISTATE_SEND,
    eWIFISTATE_WAIT_RESPONE,
} eWIFISTATE;

static uint8_t g_linkstate           = eWIFISTATE_DISCONNECTED;
static volatile uint16_t g_timestamp = 0;
extern char rptr, UART1_RxBuffer[UART_RX_BUFF_SIZE];
extern __BIT busy;
__BIT TxBusyFlag;
__BIT RxTimeout;
uint8_t RxTimeoutTicks = 0;

static volatile uint16_t g_pwmduty_motor1 = 0;
static volatile uint16_t g_pwmduty_motor2 = 0;
static volatile uint16_t g_pwmduty_motor3 = 0;
static volatile uint16_t g_pwmduty_motor4 = 0;

static int16_t MPU6050Tmp[7];
static tAttitude g_Attitude = { 0 };

static __XDATA float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
static __XDATA float exInt = 0, eyInt = 0, ezInt = 0;

static void     GPIO_Init(void);
static void     Hal_MPU6050_Init(void);
static void     Hal_ESP01S_Init(void);
static void     Hal_MotorPWM_Init(void);
static void     Hal_Timestamp_Init(void);
static uint8_t  ESP01S_Send(char *cmd);
static void     ESP01S_ClearBuff(void);
static uint8_t  ESP01S_Check(char *response);

static void IMU_Update(float ax, float ay, float az,
                        float gx, float gy, float gz);

void main(void) {
    SYS_SetClock();
    GPIO_Init();

    Hal_MPU6050_Init();
    Hal_ESP01S_Init();
    Hal_MotorPWM_Init();
    Hal_Timestamp_Init();

    while (1) {
        switch (g_timestamp) {
            case TMCLK_HZ(20) :     // wifi transmission
                if (g_linkstate == eWIFISTATE_DISCONNECTED) {
                    ESP01S_Send(ATCMD_CHK_CIPSTATE);
                    if (0 == ESP01S_Check(AT_RESPONE_CIPSTATE)) {
                        g_linkstate == eWIFISTATE_CONN_AND_READ;
                        ESP01S_ClearBuff();
                    }
                } else if (g_linkstate == eWIFISTATE_CONN_AND_READ) {
                    if (0 == ESP01S_Check(AT_RECV_DATA)) {
                        // ANO_TC_Protocol
                        ESP01S_ClearBuff();
                    } else {
                        g_linkstate == eWIFISTATE_SEND;
                    }
                } else if (g_linkstate == eWIFISTATE_SEND) {
                    // ANO_TC_Protocol
                    ESP01S_Send(ATCMD_SET_CIPSEND(10));
                    if (0 == ESP01S_Check(AT_RESPONE_OK)) {
                        ESP01S_ClearBuff();
                        ESP01S_Send("data");
                        g_linkstate == eWIFISTATE_WAIT_RESPONE;
                    } else {
                        g_linkstate == eWIFISTATE_CONN_AND_READ;
                    }
                } else if (g_linkstate == eWIFISTATE_WAIT_RESPONE) {
                    if (0 == ESP01S_Check(AT_RESPONE_OK)) {
                        ESP01S_ClearBuff();
                        g_linkstate == eWIFISTATE_CONN_AND_READ;
                    } else {
                        g_linkstate == eWIFISTATE_DISCONNECTED;
                    }
                }
                UART1_IntTxString(" string\r\n");
                break;
            case TMCLK_HZ(40) :     // sampling and calculation
                MPU6050_ReadAll(MPU6050Tmp);
                IMU_Update(2 * 9.8f * MPU6050Tmp[0] / 32768,
                            2 * 9.8f * MPU6050Tmp[1] / 32768,
                            2 * 9.8f * MPU6050Tmp[2] / 32768,
                            2000.0f * MPU6050Tmp[4] / 32768,
                            2000.0f * MPU6050Tmp[5] / 32768,
                            2000.0f * MPU6050Tmp[6] / 32768);
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


static float InvSqrt(float x) {
    float xhalf = 0.5f*x;
    int i = *(int*)&x;  // get bits for floating VALUE  --NOLINT
    i = 0x5f375a86 - (i>>1);  // gives initial guess y0
    x = *(float*)&i;  // convert bits BACK to float  --NOLINT
    x = x*(1.5f-xhalf*x*x);  // Newton step, repeating increases accuracy
    return x;
}

#define Kp      2.0f
#define Ki      0.005f
#define halfT   ((1 / TMCLK_HZ(40)) / 2)

#define q0q0 (q0 * q0)
#define q0q1 (q0 * q1)
#define q0q2 (q0 * q2)
#define q0q3 (q0 * q3)
#define q1q1 (q1 * q1)
#define q1q2 (q1 * q2)
#define q1q3 (q1 * q3)
#define q2q2 (q2 * q2)
#define q2q3 (q2 * q3)
#define q3q3 (q3 * q3)
#define vx (2 * (q1 * q3 - q0 * q2))
#define vy (2 * (q0 * q1 + q2 * q3))
#define vz (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3)
#define Rot_matrix02 (2.0f * (q1q3 + q0q2))
#define Rot_matrix10 (2.0f * (q1q2 + q0q3))
#define Rot_matrix11 (q0q0 - q1q1 + q2q2 - q3q3)
#define Rot_matrix12 (2.0f * (q2q3 - q0q1))
#define Rot_matrix22 (q0q0 - q1q1 - q2q2 + q3q3)

static void IMU_Update(float ax, float ay, float az,
                        float gx, float gy, float gz) {
    float norm;
    float ex, ey, ez;

    norm = InvSqrt(ax * ax + ay * ay + az * az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    // vx = 2 * (q1 * q3 - q0 * q2);
    // vy = 2 * (q0 * q1 + q2 * q3);
    // vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    exInt = exInt + ex * Ki;
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;

    gx = gx + Kp * ex + exInt;
    gy = gy + Kp * ey + eyInt;
    gz = gz + Kp * ez + ezInt;

    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

    norm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;

    // Rot_matrix[0][0] = q0q0 + q1q1 - q2q2 - q3q3;
    // Rot_matrix[0][1] = 2.0f * (q1q2 - q0q3);
    // Rot_matrix[0][2] = 2.0f * (q1q3 + q0q2);
    // Rot_matrix[1][0] = 2.0f * (q1q2 + q0q3);
    // Rot_matrix[1][1] = q0q0 - q1q1 + q2q2 - q3q3;
    // Rot_matrix[1][2] = 2.0f * (q2q3 - q0q1);
    // Rot_matrix[2][0] = 2.0f * (q1q3 - q0q2);
    // Rot_matrix[2][1] = 2.0f * (q2q3 + q0q1);
    // Rot_matrix[2][2] = q0q0 - q1q1 - q2q2 + q3q3;

    g_Attitude.pitch = asinf(Rot_matrix12);
    g_Attitude.roll  = atan2f(-Rot_matrix02, Rot_matrix22);
    g_Attitude.yaw   = atan2f(-Rot_matrix10, Rot_matrix11);
}

static void GPIO_Init(void) {
    GPIO_P3_SetMode(GPIO_Pin_All, GPIO_Mode_Input_HIP);
    // SDA
    GPIO_P3_SetMode(GPIO_Pin_3, GPIO_Mode_InOut_QBD);
    // SCL
    GPIO_P3_SetMode(GPIO_Pin_2, GPIO_Mode_Output_PP);
    // LED
    GPIO_P1_SetMode(GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_7, GPIO_Mode_Output_PP);    // --NOLINT
    // PWM
    GPIO_P1_SetMode(GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_6, GPIO_Mode_Output_PP);    // --NOLINT
    // charge
    GPIO_P5_SetMode(GPIO_Pin_4, GPIO_Mode_InOut_QBD);
}

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
    MPU6050_SetSampleRateDiv(50);
    MPU6050_EnableLowPowerMode(MPU6050_Wakeup_Freq_40Hz);
    MPU6050_SetDLPF(MPU6050_DLPF_Delay2ms);
    MPU6050_SetGyroFullScaleRange(MPU6050_Gyro_FullScaleRange_2000dps);
    MPU6050_SetAccFullScaleRange(MPU6050_Acc_FullScaleRange_2g);
}

static void Hal_ESP01S_Init(void) {
    UART1_Config8bitUart(UART1_BaudSource_Timer1, HAL_State_ON, 115200);
    UART1_SwitchPort(UART1_AlterPort_P36_P37);
    EXTI_Global_SetIntState(HAL_State_ON);
    EXTI_UART1_SetIntState(HAL_State_ON);
    ESP01S_Send(ATCMD_AT);
    while (!RxTimeout) {}
    ESP01S_Check(AT_RESPONE_OK);
    ESP01S_Send(ATCMD_SET_CWMODE_AP);
    while (!RxTimeout) {}
    ESP01S_Check(AT_RESPONE_OK);
    ESP01S_ClearBuff();
    ESP01S_Send(ATCMD_SET_CWSAP(popofly, 12345678));
    while (!RxTimeout) {}
    ESP01S_Check(AT_RESPONE_OK);
    ESP01S_ClearBuff();
    ESP01S_Send(ATCMD_SET_CWDHCP);
    while (!RxTimeout) {}
    ESP01S_Check(AT_RESPONE_OK);
    ESP01S_Send(ATCMD_SET_MDNS(popofly, ano_udp, 8080));
    while (!RxTimeout) {}
    ESP01S_Check(AT_RESPONE_OK);
    ESP01S_ClearBuff();
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
    PWMA_SetPinOutputState(PWM_Pin_1 | PWM_Pin_2 | PWM_Pin_3 | PWM_Pin_4, HAL_State_ON);    // --NOLINT
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

static uint8_t ESP01S_Send(char *cmd) {
    if (!TxBusyFlag) {
        UART1_IntTxString(cmd);
        return 0;
    } else {
        return 1;
    }
}

static void     ESP01S_ClearBuff(void) {
    memset(UART1_RxBuffer, 0, sizeof(UART1_RxBuffer));
    rptr = 0;
}

static uint8_t ESP01S_Check(char *response) {
    uint8_t ret = 1;
    if (RxTimeout) {
        TxBusyFlag = 0;
        RxTimeout  = 0;
        if (NULL != strstr(UART1_RxBuffer, response)) {
            ret  = 0;
        }
    }
    return ret;
}


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
