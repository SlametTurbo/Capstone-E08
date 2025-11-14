#include "MPU6050.h"
#include "main.h"
#include "i2c.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* ------------------ 1) KALMAN SEDERHANA UNTUK SUDUT (Lauszus-style 1D) ------------------ */
typedef struct {
    float angle; // derajat
    float bias;  // dps
    float rate;  // dps
    float P[2][2];
    float Q_angle, Q_bias, R_measure;
} KalmanAngle;

static void KA_Init(KalmanAngle* k, float Q_angle, float Q_bias, float R_measure) {
    memset(k, 0, sizeof(*k));
    k->Q_angle = Q_angle;
    k->Q_bias  = Q_bias;
    k->R_measure = R_measure;
    k->P[0][0] = 0.0f; k->P[0][1] = 0.0f;
    k->P[1][0] = 0.0f; k->P[1][1] = 0.0f;
}
static void KA_setAngle(KalmanAngle* k, float angle) { k->angle = angle; }
static float KA_getAngle(KalmanAngle* k, float newAngle_meas, float newRate_dps, float dt) {
    // predict
    k->rate  = newRate_dps - k->bias;
    k->angle += dt * k->rate;

    // P = A P A' + Q  (A=[[1,-dt],[0,1]])
    k->P[0][0] += dt * (dt*k->P[1][1] - k->P[0][1] - k->P[1][0]) + k->Q_angle;
    k->P[0][1] -= dt * k->P[1][1];
    k->P[1][0] -= dt * k->P[1][1];
    k->P[1][1] += k->Q_bias;

    // update
    float S  = k->P[0][0] + k->R_measure;
    float K0 = k->P[0][0] / S;
    float K1 = k->P[1][0] / S;

    float y  = newAngle_meas - k->angle;

    k->angle += K0 * y;
    k->bias  += K1 * y;

    float P00 = k->P[0][0];
    float P01 = k->P[0][1];

    k->P[0][0] -= K0 * P00;
    k->P[0][1] -= K0 * P01;
    k->P[1][0] -= K1 * P00;
    k->P[1][1] -= K1 * P01;

    return k->angle;
}

/* ------------------ 2) KALMAN SKALAR SANGAT RINGAN (untuk smoothing acc/gyro per-sumbu) ------------------ */
typedef struct {
    float x;   // state (estimasi)
    float P;   // kovarians
    float Q;   // process noise
    float R;   // measurement noise
    uint8_t init;
} KFScalar;

static void KFS_Init(KFScalar* k, float Q, float R, float x0) {
    k->x = x0; k->P = 1.0f; k->Q = Q; k->R = R; k->init = 1;
}
static float KFS_Update(KFScalar* k, float z) {
    if (!k->init) KFS_Init(k, k->Q>0?k->Q:1e-3f, k->R>0?k->R:1e-2f, z);
    // predict
    k->P += k->Q;
    // update
    float S = k->P + k->R;
    float K = k->P / S;
    k->x += K * (z - k->x);
    k->P *= (1.0f - K);
    return k->x;
}

/* ------------------ 3) INSTANCE GLOBAL ------------------ */
Struct_MPU6050 MPU6050;

/* pointer internal ke filter (disimpan sebagai void* di struct publik) */
typedef struct { KFScalar a[3]; } _KF3;
static _KF3 _kf_acc_store, _kf_gyro_store;
static KalmanAngle _kf_roll_store, _kf_pitch_store;

/* ------------------ 4) LOW-LEVEL I2C ------------------ */
void MPU6050_Writebyte(uint8_t reg_addr, uint8_t val) {
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &val, 1, HAL_MAX_DELAY);
}
void MPU6050_Writebytes(uint8_t reg_addr, uint8_t len, uint8_t* data) {
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}
void MPU6050_Readbyte(uint8_t reg_addr, uint8_t* data) {
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}
void MPU6050_Readbytes(uint8_t reg_addr, uint8_t len, uint8_t* data) {
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}

/* ------------------ 5) INISIALISASI PERANGKAT ------------------ */
static void _get_lsb_sens(uint8_t FS_SCALE_GYRO, uint8_t FS_SCALE_ACC, float* lsb_gyro, float* lsb_acc) {
    switch (FS_SCALE_GYRO) {
        case 0: *lsb_gyro = 131.f;  break;   // ±250 dps
        case 1: *lsb_gyro = 65.5f;  break;   // ±500
        case 2: *lsb_gyro = 32.8f;  break;   // ±1000
        default:*lsb_gyro = 16.4f;  break;   // ±2000
    }
    switch (FS_SCALE_ACC) {
        case 0: *lsb_acc = 16384.f; break;   // ±2g
        case 1: *lsb_acc = 8192.f;  break;   // ±4g
        case 2: *lsb_acc = 4096.f;  break;   // ±8g
        default:*lsb_acc = 2048.f;  break;   // ±16g
    }
}

void MPU6050_Initialization(void)
{
    HAL_Delay(50);
    uint8_t who = 0;
    MPU6050_Readbyte(MPU6050_WHO_AM_I, &who);
    if (who != 0x68) {
        printf("MPU6050 WHO_AM_I=0x%02X (expected 0x68)\r\n", who);
    }

    /* reset & wake */
    MPU6050_Writebyte(MPU6050_PWR_MGMT_1, 1<<7);
    HAL_Delay(100);
    MPU6050_Writebyte(MPU6050_PWR_MGMT_1, 0x00);
    HAL_Delay(50);

    /* sample rate & DLPF simpel */
    MPU6050_Writebyte(MPU6050_SMPRT_DIV, 39); // 8k/ (1+39) ≈ 200 Hz gyro; acc 1k internal -> 200Hz out
    MPU6050_Writebyte(MPU6050_CONFIG, 0x00);
    HAL_Delay(10);

    /* FS gyro & acc */
    uint8_t fs_g=0, fs_a=0; // ±250 dps, ±2g
    MPU6050_Writebyte(MPU6050_GYRO_CONFIG,  (fs_g<<3));
    MPU6050_Writebyte(MPU6050_ACCEL_CONFIG, (fs_a<<3));
    HAL_Delay(10);

    _get_lsb_sens(fs_g, fs_a, &MPU6050.lsb_gyro, &MPU6050.lsb_acc);

    /* interrupt config (opsional) */
    MPU6050_Writebyte(MPU6050_INT_PIN_CFG,  (0<<7)|(0<<5)|(1<<4));
    MPU6050_Writebyte(MPU6050_INT_ENABLE,   0x01);
    HAL_Delay(10);

    /* default offset & sudut */
    MPU6050.acc_off_x = MPU6050.acc_off_y = MPU6050.acc_off_z = 0.0f;
    MPU6050.gyro_off_x= MPU6050.gyro_off_y= MPU6050.gyro_off_z= 0.0f;
    MPU6050.roll_deg = MPU6050.pitch_deg = MPU6050.yaw_deg = 0.0f;

    /* default tuning KF */
    MPU6050_KF_SetDefaults(&MPU6050);

    /* hook internal pointers */
    MPU6050._kf_acc_xyz  = &_kf_acc_store;
    MPU6050._kf_gyro_xyz = &_kf_gyro_store;
    MPU6050._kf_roll     = &_kf_roll_store;
    MPU6050._kf_pitch    = &_kf_pitch_store;

    /* init kalman sudut */
    KA_Init((KalmanAngle*)MPU6050._kf_roll,  MPU6050.kf_ang_Qangle, MPU6050.kf_ang_Qbias, MPU6050.kf_ang_Rmeas);
    KA_Init((KalmanAngle*)MPU6050._kf_pitch, MPU6050.kf_ang_Qangle, MPU6050.kf_ang_Qbias, MPU6050.kf_ang_Rmeas);

    MPU6050.last_ms = HAL_GetTick();

    printf("MPU6050 init OK (LSB acc=%.0f, gyro=%.1f)\r\n", MPU6050.lsb_acc, MPU6050.lsb_gyro);
}

/* ------------------ 6) BACA RAW & KONVERSI ------------------ */
static void _get6raw(Struct_MPU6050* m) {
    uint8_t d[14];
    MPU6050_Readbytes(MPU6050_ACCEL_XOUT_H, 14, d);
    m->acc_x_raw  = (int16_t)((d[0]<<8)|d[1]);
    m->acc_y_raw  = (int16_t)((d[2]<<8)|d[3]);
    m->acc_z_raw  = (int16_t)((d[4]<<8)|d[5]);
    m->temperature_raw = (int16_t)((d[6]<<8)|d[7]);
    m->gyro_x_raw = (int16_t)((d[8]<<8)|d[9]);
    m->gyro_y_raw = (int16_t)((d[10]<<8)|d[11]);
    m->gyro_z_raw = (int16_t)((d[12]<<8)|d[13]);
}

static void _convert(Struct_MPU6050* m) {
    m->acc_x = (float)m->acc_x_raw / m->lsb_acc;
    m->acc_y = (float)m->acc_y_raw / m->lsb_acc;
    m->acc_z = (float)m->acc_z_raw / m->lsb_acc;
    m->temperature = (float)m->temperature_raw / 340.0f + 36.53f;
    m->gyro_x = (float)m->gyro_x_raw / m->lsb_gyro;
    m->gyro_y = (float)m->gyro_y_raw / m->lsb_gyro;
    m->gyro_z = (float)m->gyro_z_raw / m->lsb_gyro;
}

void MPU6050_ProcessData(Struct_MPU6050* m) {
    _get6raw(m);
    _convert(m);
}

/* ------------------ 7) KALIBRASI & SEED ANGLE ------------------ */
void MPU6050_Calibrate(Struct_MPU6050* m, uint16_t samples, uint16_t delay_ms) {
    float ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
    for (uint16_t i=0;i<samples;i++){
        MPU6050_ProcessData(m);
        ax+=m->acc_x; ay+=m->acc_y; az+=m->acc_z;
        gx+=m->gyro_x; gy+=m->gyro_y; gz+=m->gyro_z;
        HAL_Delay(delay_ms);
    }
    ax/=samples; ay/=samples; az/=samples;
    gx/=samples; gy/=samples; gz/=samples;

    m->acc_off_x = ax;
    m->acc_off_y = ay;
    m->acc_off_z = (az - 1.0f);
    m->gyro_off_x = gx;
    m->gyro_off_y = gy;
    m->gyro_off_z = gz;
}

void MPU6050_InitAnglesFromAccel(Struct_MPU6050* m) {
    MPU6050_ProcessData(m);
    float Ax=m->acc_x-m->acc_off_x, Ay=m->acc_y-m->acc_off_y, Az=m->acc_z-m->acc_off_z;
    float rollAcc  = atan2f(Ay, Az)*RAD2DEG;
    float pitchAcc = atan2f(-Ax, sqrtf(Ay*Ay+Az*Az))*RAD2DEG;

    KA_setAngle((KalmanAngle*)m->_kf_roll,  rollAcc);
    KA_setAngle((KalmanAngle*)m->_kf_pitch, pitchAcc);

    m->roll_deg  = rollAcc;
    m->pitch_deg = pitchAcc;
    m->yaw_deg   = 0.f;
    m->last_ms   = HAL_GetTick();

    /* inisialisasi KF skalar acc/gyro agar output awal tidak NaN */
    _KF3* kfa = (_KF3*)m->_kf_acc_xyz;
    _KF3* kfg = (_KF3*)m->_kf_gyro_xyz;
    for (int i=0;i<3;i++){
        KFS_Init(&kfa->a[i], m->kf_acc_Q,  m->kf_acc_R,  (i==0?Ax:(i==1?Ay:Az)));
        float G0 = (i==0?m->gyro_x:(i==1?m->gyro_y:m->gyro_z)) - (i==0?m->gyro_off_x:(i==1?m->gyro_off_y:m->gyro_off_z));
        KFS_Init(&kfg->a[i], m->kf_gyro_Q, m->kf_gyro_R, G0);
    }
}

/* ------------------ 8) KONFIGURASI FILTER ------------------ */
void MPU6050_KF_SetDefaults(Struct_MPU6050* m) {
    m->kf_acc_Q  = 1e-3f; m->kf_acc_R  = 1e-2f;  // smoothing acc
    m->kf_gyro_Q = 1e-3f; m->kf_gyro_R = 1e-1f;  // smoothing gyro
    m->kf_ang_Qangle = 1e-3f;   // Kalman sudut
    m->kf_ang_Qbias  = 3e-3f;
    m->kf_ang_Rmeas  = 3e-2f;
}
void MPU6050_KF_Config(Struct_MPU6050* m,
                       float acc_Q,  float acc_R,
                       float gyro_Q, float gyro_R,
                       float ang_Qangle, float ang_Qbias, float ang_Rmeas)
{
    m->kf_acc_Q = acc_Q;   m->kf_acc_R = acc_R;
    m->kf_gyro_Q= gyro_Q;  m->kf_gyro_R= gyro_R;
    m->kf_ang_Qangle = ang_Qangle;
    m->kf_ang_Qbias  = ang_Qbias;
    m->kf_ang_Rmeas  = ang_Rmeas;

    /* re-init sudut filter dengan param baru */
    KA_Init((KalmanAngle*)m->_kf_roll,  m->kf_ang_Qangle, m->kf_ang_Qbias, m->kf_ang_Rmeas);
    KA_Init((KalmanAngle*)m->_kf_pitch, m->kf_ang_Qangle, m->kf_ang_Qbias, m->kf_ang_Rmeas);
    /* seed ulang nanti lewat MPU6050_InitAnglesFromAccel() */
}

/* ------------------ 9) UPDATE TUNGGAL — SEMUA FILTER DI SINI ------------------ */
void MPU6050_Update(Struct_MPU6050* m)
{
    /* 1) ambil & konversi */
    MPU6050_ProcessData(m);

    /* 2) offset-corrected */
    float Ax = m->acc_x - m->acc_off_x;
    float Ay = m->acc_y - m->acc_off_y;
    float Az = m->acc_z - m->acc_off_z;

    float Gx = m->gyro_x - m->gyro_off_x;   // dps
    float Gy = m->gyro_y - m->gyro_off_y;
    float Gz = m->gyro_z - m->gyro_off_z;

    /* 3) dt */
    uint32_t now = HAL_GetTick();
    float dt = (now - m->last_ms) * 0.001f;
    if (dt <= 0.f) dt = 0.001f;
    m->last_ms = now;

    /* 4) KF skalar untuk acc/gyro per-sumbu (nilai halus siap dipakai di main) */
    _KF3* kfa = (_KF3*)m->_kf_acc_xyz;
    _KF3* kfg = (_KF3*)m->_kf_gyro_xyz;

    /* pastikan Q/R sudah terisi */
    for (int i=0;i<3;i++){
        kfa->a[i].Q = m->kf_acc_Q;  kfa->a[i].R = m->kf_acc_R;
        kfg->a[i].Q = m->kf_gyro_Q; kfg->a[i].R = m->kf_gyro_R;
    }

    m->acc_x_f  = KFS_Update(&kfa->a[0], Ax);
    m->acc_y_f  = KFS_Update(&kfa->a[1], Ay);
    m->acc_z_f  = KFS_Update(&kfa->a[2], Az);

    m->gyro_x_f = KFS_Update(&kfg->a[0], Gx);
    m->gyro_y_f = KFS_Update(&kfg->a[1], Gy);
    m->gyro_z_f = KFS_Update(&kfg->a[2], Gz);

    /* 5) sudut dari accel (meas) */
    float rollAcc  = atan2f(m->acc_y_f, m->acc_z_f) * RAD2DEG;
    float pitchAcc = atan2f(-m->acc_x_f, sqrtf(m->acc_y_f*m->acc_y_f + m->acc_z_f*m->acc_z_f)) * RAD2DEG;

    /* 6) Kalman sudut (gabung meas accel + rate gyro) */
    KalmanAngle* kr = (KalmanAngle*)m->_kf_roll;
    KalmanAngle* kp = (KalmanAngle*)m->_kf_pitch;

    /* jika belum di-seed, seed dulu */
    if (kr->P[0][0]==0.f && kr->P[1][1]==0.f && kr->angle==0.f && m->roll_deg==0.f) {
        KA_setAngle(kr, rollAcc);
        KA_setAngle(kp, pitchAcc);
    }

    m->roll_deg  = KA_getAngle(kr, rollAcc,  m->gyro_x_f, dt);
    m->pitch_deg = KA_getAngle(kp, pitchAcc, m->gyro_y_f, dt);

    /* 7) yaw sederhana (integrasi gyro Z yang sudah difilter) */
    m->yaw_deg  += m->gyro_z_f * dt;
}

/* ------------------ 10) OPSIONAL: INT pin ------------------ */
int MPU6050_DataReady(void) {
    return HAL_GPIO_ReadPin(MPU6050_INT_PORT, MPU6050_INT_PIN);
}
