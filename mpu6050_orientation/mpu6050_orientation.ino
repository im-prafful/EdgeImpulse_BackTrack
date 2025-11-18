#include <Wire.h>
#include <math.h>

// ====== MPU6050 Registers ======
#define MPU6050_ADDR   0x68
#define WHO_AM_I       0x75
#define PWR_MGMT_1     0x6B
#define SMPLRT_DIV     0x19
#define CONFIG_REG     0x1A
#define GYRO_CONFIG    0x1B
#define ACCEL_CONFIG   0x1C
#define ACCEL_XOUT_H   0x3B

// ====== Config ======
const uint8_t ACCEL_FS_4G = (1 << 3);   // ±4g
const uint8_t GYRO_FS_500 = (1 << 3);   // ±500 dps
const uint8_t DLPF_CFG    = 3;          // ~44 Hz accel / 42 Hz gyro
const float   ACC_LSB_PER_G    = 8192.0f; // ±4g
const float   GYRO_LSB_PER_DPS = 65.5f;   // ±500 dps

// Calibration parameters
const int CAL_DISCARD = 500;
const int CAL_SAMPLES = 8000;

// Sampling and fusion parameters
const int   SAMPLE_PERIOD_MS = 5;       // ~200 Hz
const float ALPHA_RP         = 0.98f;   // complementary filter blend for roll/pitch
// yaw has no accel correction (no magnetometer); you can slowly bleed bias if desired.

// Biases
float ax_bias=0, ay_bias=0, az_bias=0;
float gx_bias=0, gy_bias=0, gz_bias=0;

// Orientation state (Euler degrees)
float roll_deg  = 0.0f;
float pitch_deg = 0.0f;
float yaw_deg   = 0.0f;

// --- I2C helpers ---
void i2cWrite8(uint8_t reg, uint8_t val){
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission(true);
}
bool i2cReadN(uint8_t reg, uint8_t n, uint8_t* buf){
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU6050_ADDR, (int)n, (int)true) != n) return false;
  for (int i=0;i<n;i++) buf[i]=Wire.read();
  return true;
}
bool readRaw(int16_t& ax,int16_t& ay,int16_t& az,int16_t& gx,int16_t& gy,int16_t& gz){
  uint8_t data[14];
  if (!i2cReadN(ACCEL_XOUT_H,14,data)) return false;
  ax = (int16_t)((data[0]<<8)|data[1]);
  ay = (int16_t)((data[2]<<8)|data[3]);
  az = (int16_t)((data[4]<<8)|data[5]);
  // temp bytes data[6], data[7] ignored
  gx = (int16_t)((data[8]<<8)|data[9]);
  gy = (int16_t)((data[10]<<8)|data[11]);
  gz = (int16_t)((data[12]<<8)|data[13]);
  return true;
}

// --- Device init ---
bool mpu_init(){
  i2cWrite8(PWR_MGMT_1, 0x00);  // wake
  delay(100);
  uint8_t who=0;
  if (!i2cReadN(WHO_AM_I,1,&who)) return false;
  if (who != 0x68) return false;
  i2cWrite8(CONFIG_REG, DLPF_CFG & 0x07);
  i2cWrite8(SMPLRT_DIV, 4);                // 200 Hz nominal
  i2cWrite8(GYRO_CONFIG,  GYRO_FS_500);
  i2cWrite8(ACCEL_CONFIG, ACCEL_FS_4G);
  return true;
}

// --- Orientation-agnostic calibration (same logic you approved) ---
bool calibrate() {
  const int DISCARD = CAL_DISCARD;
  const int N = CAL_SAMPLES;
  const int READ_DELAY_MS = 2;

  long long ax_sum=0, ay_sum=0, az_sum=0;
  long long gx_sum=0, gy_sum=0, gz_sum=0;
  int good = 0;

  for (int i = 0; i < DISCARD; i++) {
    int16_t ax,ay,az,gx,gy,gz;
    if (!readRaw(ax,ay,az,gx,gy,gz)) return false;
    delay(READ_DELAY_MS);
  }
  for (int i = 0; i < N; i++) {
    int16_t ax,ay,az,gx,gy,gz;
    if (!readRaw(ax,ay,az,gx,gy,gz)) return false;
    ax_sum += ax; ay_sum += ay; az_sum += az;
    gx_sum += gx; gy_sum += gy; gz_sum += gz;
    good++;
    delay(READ_DELAY_MS);
  }
  if (good < N) return false;

  float invN = 1.0f / (float)good;
  float ax_mean = (float)ax_sum * invN;
  float ay_mean = (float)ay_sum * invN;
  float az_mean = (float)az_sum * invN;
  float gx_mean = (float)gx_sum * invN;
  float gy_mean = (float)gy_sum * invN;
  float gz_mean = (float)gz_sum * invN;

  // Gyro biases
  gx_bias = gx_mean;
  gy_bias = gy_mean;
  gz_bias = gz_mean;

  // Detect gravity axis and correct that axis by ±1g
  float ax_abs = fabsf(ax_mean), ay_abs = fabsf(ay_mean), az_abs = fabsf(az_mean);
  int up_idx = 0; float up_val = ax_abs;
  if (ay_abs > up_val) { up_idx = 1; up_val = ay_abs; }
  if (az_abs > up_val) { up_idx = 2; up_val = az_abs; }

  float up_g = up_val / ACC_LSB_PER_G;
  if (up_g < 0.7f || up_g > 1.3f) return false; // not still/flat enough

  ax_bias = ax_mean;
  ay_bias = ay_mean;
  az_bias = az_mean;

  float g_target_raw = ACC_LSB_PER_G;
  if ((up_idx==0 && ax_mean<0) || (up_idx==1 && ay_mean<0) || (up_idx==2 && az_mean<0)) {
    g_target_raw = -ACC_LSB_PER_G;
  }
  if (up_idx==0) ax_bias = ax_mean - g_target_raw;
  else if (up_idx==1) ay_bias = ay_mean - g_target_raw;
  else az_bias = az_mean - g_target_raw;

  return true;
}

void print_offsets(){
  Serial.println("CALIBRATION_OFFSETS_BEGIN");
  Serial.printf("GX_BIAS_LSB,%.3f\n", gx_bias);
  Serial.printf("GY_BIAS_LSB,%.3f\n", gy_bias);
  Serial.printf("GZ_BIAS_LSB,%.3f\n", gz_bias);
  Serial.printf("AX_BIAS_LSB,%.3f\n", ax_bias);
  Serial.printf("AY_BIAS_LSB,%.3f\n", ay_bias);
  Serial.printf("AZ_BIAS_LSB,%.3f\n", az_bias);
  Serial.printf("ACC_LSB_PER_G,%.1f\n", ACC_LSB_PER_G);
  Serial.printf("GYRO_LSB_PER_DPS,%.1f\n", GYRO_LSB_PER_DPS);
  Serial.printf("ACCEL_FS,±4g\n");
  Serial.printf("GYRO_FS,±500dps\n");
  Serial.printf("DLPF_CFG,%u\n", (unsigned)DLPF_CFG);
  Serial.println("CALIBRATION_OFFSETS_END");
}

// --- Complementary filter helpers ---
static inline float rad2deg(float r){ return r * (180.0f / (float)M_PI); }
static inline float deg2rad(float d){ return d * ((float)M_PI / 180.0f); }

void setup(){
  Serial.begin(115200);
  delay(200);
  Wire.begin(); // or Wire.begin(SDA,SCL,400000)

  Serial.println("INIT: MPU6050 over I2C...");
  if(!mpu_init()){ Serial.println("INIT_FAIL"); return; }
  Serial.println("INIT_OK");
  Serial.println("CALIBRATION_START: keep the sensor flat and still...");
  delay(2000);

  if(!calibrate()){
    Serial.println("CALIBRATION_FAIL");
    print_offsets();
    return;
  }
  Serial.println("CALIBRATION_OK");
  print_offsets();

  // Initialize roll/pitch from accelerometer (yaw starts at 0)
  int16_t ax,ay,az,gx,gy,gz;
  if (readRaw(ax,ay,az,gx,gy,gz)) {
    float ax_g = (ax - ax_bias) / ACC_LSB_PER_G;
    float ay_g = (ay - ay_bias) / ACC_LSB_PER_G;
    float az_g = (az - az_bias) / ACC_LSB_PER_G;
    // atan2(ay, az) for roll; atan2(-ax, sqrt(ay^2+az^2)) for pitch
    roll_deg  = rad2deg(atan2f(ay_g, az_g));
    pitch_deg = rad2deg(atan2f(-ax_g, sqrtf(ay_g*ay_g + az_g*az_g)));
    yaw_deg   = 0.0f; // relative yaw
  }
  Serial.println("ORIENTATION_READY");
  Serial.println("Header: roll_deg,pitch_deg,yaw_deg");
}

void loop(){
  static uint32_t last_ms = millis();
  uint32_t now = millis();
  uint32_t dt_ms = now - last_ms;
  if (dt_ms < (uint32_t)SAMPLE_PERIOD_MS) return;
  last_ms = now;
  float dt = dt_ms * 1e-3f;

  int16_t ax,ay,az,gx,gy,gz;
  if (!readRaw(ax,ay,az,gx,gy,gz)) return;

  // Bias-correct and scale
  float ax_g = (ax - ax_bias) / ACC_LSB_PER_G;
  float ay_g = (ay - ay_bias) / ACC_LSB_PER_G;
  float az_g = (az - az_bias) / ACC_LSB_PER_G;
  float gx_dps = (gx - gx_bias) / GYRO_LSB_PER_DPS;
  float gy_dps = (gy - gy_bias) / GYRO_LSB_PER_DPS;
  float gz_dps = (gz - gz_bias) / GYRO_LSB_PER_DPS;

  // 1) Integrate gyro
  roll_deg  += gx_dps * dt;
  pitch_deg += gy_dps * dt;
  yaw_deg   += gz_dps * dt;   // relative yaw (will drift)

  // 2) Compute accel-only tilt (gravity-based)
  // Guard against transient free-fall (very small norm) to avoid NaNs
  float norm = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
  if (norm > 0.5f && norm < 2.0f) {
    float roll_acc  = rad2deg(atan2f(ay_g, az_g));
    float pitch_acc = rad2deg(atan2f(-ax_g, sqrtf(ay_g*ay_g + az_g*az_g)));

    // 3) Complementary blend (heavier gyro at short term, accel corrects long-term drift)
    roll_deg  = ALPHA_RP * roll_deg  + (1.0f - ALPHA_RP) * roll_acc;
    pitch_deg = ALPHA_RP * pitch_deg + (1.0f - ALPHA_RP) * pitch_acc;
  }

  // Optional: wrap yaw to [-180,180] to prevent growth
  if (yaw_deg > 180.f) yaw_deg -= 360.f;
  else if (yaw_deg < -180.f) yaw_deg += 360.f;

  // Output persistent orientation (CSV-like or human-readable)
  Serial.printf("%.2f,%.2f,%.2f\n", roll_deg, pitch_deg, yaw_deg);
}
