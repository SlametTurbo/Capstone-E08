#ifndef __MPU6050_H__
#define __MPU6050_H__

#include <stdint.h>

/* ----- Alamat dan register (singkat) ----- */
#define MPU6050_ADDR            (0x68 << 1)
#define MPU6050_WHO_AM_I        0x75
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_SMPRT_DIV       0x19
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_INT_PIN_CFG     0x37
#define MPU6050_INT_ENABLE      0x38
#define MPU6050_INT_STATUS      0x3A
#define MPU6050_ACCEL_XOUT_H    0x3B

/* ----- Port INT sesuaikan project kamu ----- */
#define MPU6050_INT_PORT        GPIOC
#define MPU6050_INT_PIN         GPIO_PIN_13

#ifndef RAD2DEG
#define RAD2DEG 57.29577951308232f
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* raw */
    int16_t acc_x_raw, acc_y_raw, acc_z_raw;
    int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
    int16_t temperature_raw;

    /* satuan fisik (g, dps, °C) — sebelum filter */
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;
    float temperature;

    /* offset kalibrasi */
    float acc_off_x, acc_off_y, acc_off_z;
    float gyro_off_x, gyro_off_y, gyro_off_z;

    /* nilai TERFILTER (hasil Kalman skalar per-sumbu) */
    float acc_x_f, acc_y_f, acc_z_f;
    float gyro_x_f, gyro_y_f, gyro_z_f;

    /* sudut hasil Kalman sudut (roll, pitch), yaw integrasi gyro */
    float roll_deg, pitch_deg, yaw_deg;

    /* sensitivitas LSB -> fisik */
    float lsb_acc, lsb_gyro;

    /* internal timing */
    uint32_t last_ms;

    /* tuning filter skalar (acc/gyro) */
    float kf_acc_Q,  kf_acc_R;   // sama untuk semua sumbu acc
    float kf_gyro_Q, kf_gyro_R;  // sama untuk semua sumbu gyro

    /* tuning Kalman sudut */
    float kf_ang_Qangle;
    float kf_ang_Qbias;
    float kf_ang_Rmeas;

    /* internal state Kalman (disembunyikan di .c) */
    void* _kf_acc_xyz;   // pointer internal
    void* _kf_gyro_xyz;  // pointer internal
    void* _kf_roll;
    void* _kf_pitch;
} Struct_MPU6050;

/* variabel global opsional (seperti yang sudah kamu pakai) */
extern Struct_MPU6050 MPU6050;

/* API utama — simpel di main.c */
void MPU6050_Initialization(void);
void MPU6050_Calibrate(Struct_MPU6050* m, uint16_t samples, uint16_t delay_ms);
void MPU6050_InitAnglesFromAccel(Struct_MPU6050* m);

/* konfigurasi default dan/atau kustom untuk filter */
void MPU6050_KF_SetDefaults(Struct_MPU6050* m);
void MPU6050_KF_Config(Struct_MPU6050* m,
                       float acc_Q,  float acc_R,
                       float gyro_Q, float gyro_R,
                       float ang_Qangle, float ang_Qbias, float ang_Rmeas);

/* loop pemrosesan tunggal — semua sudah difilter di sini */
void MPU6050_Update(Struct_MPU6050* m);

/* helper opsional */
static inline void MPU6050_GetRPY(const Struct_MPU6050* m, float* r, float* p, float* y) {
    if (r) *r = m->roll_deg;
    if (p) *p = m->pitch_deg;
    if (y) *y = m->yaw_deg;
}

int  MPU6050_DataReady(void); // pakai pin INT (opsional)
void MPU6050_ProcessData(Struct_MPU6050* m); // masih tersedia, tapi MPU6050_Update sudah cukup

/* low-level I2C helpers */
void MPU6050_Writebyte(uint8_t reg_addr, uint8_t val);
void MPU6050_Writebytes(uint8_t reg_addr, uint8_t len, uint8_t* data);
void MPU6050_Readbyte(uint8_t reg_addr, uint8_t* data);
void MPU6050_Readbytes(uint8_t reg_addr, uint8_t len, uint8_t* data);

#ifdef __cplusplus
}
#endif
#endif
