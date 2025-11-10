#include <Wire.h>

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
const uint8_t DLPF_CFG     = 3;         // ~44 Hz accel/42 Hz gyro
const int SAMPLES_TO_COLLECT = 2000;

const float ACC_LSB_PER_G     = 8192.0f;  // ±4g
const float GYRO_LSB_PER_DPS  = 65.5f;    // ±500 dps

const int CAL_DISCARD = 500;
const int CAL_SAMPLES = 8000;

float ax_bias=0, ay_bias=0, az_bias=0;
float gx_bias=0, gy_bias=0, gz_bias=0;

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
  gx = (int16_t)((data[8]<<8)|data[9]);
  gy = (int16_t)((data[10]<<8)|data[11]);
  gz = (int16_t)((data[12]<<8)|data[13]);
  return true;
}

bool mpu_init(){
  i2cWrite8(PWR_MGMT_1, 0x00);  // wake
  delay(100);
  uint8_t who=0;
  if (!i2cReadN(WHO_AM_I,1,&who)) return false;
  if (who != 0x68) return false;
  i2cWrite8(CONFIG_REG, DLPF_CFG & 0x07);
  i2cWrite8(SMPLRT_DIV, 4);                // 200 Hz
  i2cWrite8(GYRO_CONFIG,  GYRO_FS_500);
  i2cWrite8(ACCEL_CONFIG, ACCEL_FS_4G);
  return true;
}

// Orientation-agnostic calibration
bool calibrate() {
  // Parameters
  const int DISCARD = CAL_DISCARD;   // e.g., 500
  const int N = CAL_SAMPLES;         // e.g., 8000
  const int READ_DELAY_MS = 2;

  // Accumulators in 64-bit to avoid overflow
  long long ax_sum=0, ay_sum=0, az_sum=0;
  long long gx_sum=0, gy_sum=0, gz_sum=0;
  int good = 0;

  // 1) Discard warmup samples
  for (int i = 0; i < DISCARD; i++) {
    int16_t ax,ay,az,gx,gy,gz;
    if (!readRaw(ax,ay,az,gx,gy,gz)) return false;
    delay(READ_DELAY_MS);
  }

  // 2) Average still samples with read checks
  for (int i = 0; i < N; i++) {
    int16_t ax,ay,az,gx,gy,gz;
    if (!readRaw(ax,ay,az,gx,gy,gz)) {
      // If any read fails, abort to avoid corrupt averages
      return false;
    }
    ax_sum += ax; ay_sum += ay; az_sum += az;
    gx_sum += gx; gy_sum += gy; gz_sum += gz;
    good++;
    delay(READ_DELAY_MS);
  }
  if (good < N) return false;

  // 3) Compute means
  const float invN = 1.0f / (float)good;
  float ax_mean = (float)ax_sum * invN;
  float ay_mean = (float)ay_sum * invN;
  float az_mean = (float)az_sum * invN;
  float gx_mean = (float)gx_sum * invN;
  float gy_mean = (float)gy_sum * invN;
  float gz_mean = (float)gz_sum * invN;

  // 4) Gyro biases (expect near 0 dps at rest)
  gx_bias = gx_mean;
  gy_bias = gy_mean;
  gz_bias = gz_mean;

  // 5) Detect which accel axis is “up” (largest |mean|)
  float ax_abs = fabsf(ax_mean);
  float ay_abs = fabsf(ay_mean);
  float az_abs = fabsf(az_mean);

  // Select index of max magnitude: 0->X, 1->Y, 2->Z
  int up_idx = 0;
  float up_val = ax_abs;
  if (ay_abs > up_val) { up_idx = 1; up_val = ay_abs; }
  if (az_abs > up_val) { up_idx = 2; up_val = az_abs; }

  // Convert to g using selected FSR constant
  float up_g = up_val / ACC_LSB_PER_G;

  // 6) Sanity check gravity magnitude
  if (up_g < 0.7f || up_g > 1.3f) {
    // Likely moving or unstable orientation; ask to re-run
    return false;
  }

  // 7) Accel biases:
  // Raw biases are the means. For the up-axis, subtract 1g (with correct sign)
  ax_bias = ax_mean;
  ay_bias = ay_mean;
  az_bias = az_mean;

  // Determine the sign of gravity on the up-axis (+1g or -1g)
  float g_target_raw = ACC_LSB_PER_G;  // +1g in raw units
  if ((up_idx == 0 && ax_mean < 0) ||
      (up_idx == 1 && ay_mean < 0) ||
      (up_idx == 2 && az_mean < 0)) {
    g_target_raw = -ACC_LSB_PER_G;    // -1g if axis mean is negative
  }

  // Apply 1g expectation correction only to the up-axis
  if (up_idx == 0) {
    ax_bias = ax_mean - g_target_raw;
  } else if (up_idx == 1) {
    ay_bias = ay_mean - g_target_raw;
  } else {
    az_bias = az_mean - g_target_raw;
  }

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

void print_csv_header(){
  Serial.println("sample,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,"
                 "GX_BIAS_LSB,GY_BIAS_LSB,GZ_BIAS_LSB,AX_BIAS_LSB,AY_BIAS_LSB,AZ_BIAS_LSB,"
                 "ACC_LSB_PER_G,GYRO_LSB_PER_DPS,ACCEL_FS,GYRO_FS,DLPF_CFG");
}

void setup(){
  Serial.begin(115200);
  delay(200);
  Wire.begin(); // Wire.begin(SDA,SCL,400000) if needed

  Serial.println("INIT: MPU6050 over I2C...");
  if(!mpu_init()){ Serial.println("INIT_FAIL"); return; }
  Serial.println("INIT_OK");
  Serial.println("CALIBRATION_START: keep the sensor flat and still...");
  delay(2000);

  bool ok = calibrate();
  if(!ok){ Serial.println("CALIBRATION_FAIL"); print_offsets(); return; }
  Serial.println("CALIBRATION_OK");
  print_offsets();

  // Signal the PC logger to start file
  Serial.println("START_CSV");
  print_csv_header();

  // Stream 2000 samples as CSV
  for (int n=0; n<SAMPLES_TO_COLLECT; n++){
    int16_t ax,ay,az,gx,gy,gz;
    if (!readRaw(ax,ay,az,gx,gy,gz)) {
      Serial.printf("%d,NaN,NaN,NaN,NaN,NaN,NaN,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,±4g,±500dps,%u\n",
                    n, gx_bias, gy_bias, gz_bias, ax_bias, ay_bias, az_bias,
                    ACC_LSB_PER_G, GYRO_LSB_PER_DPS, (unsigned)DLPF_CFG);
    } else {
      float ax_g = (ax - ax_bias) / ACC_LSB_PER_G;
      float ay_g = (ay - ay_bias) / ACC_LSB_PER_G;
      float az_g = (az - az_bias) / ACC_LSB_PER_G;
      float gx_dps = (gx - gx_bias) / GYRO_LSB_PER_DPS;
      float gy_dps = (gy - gy_bias) / GYRO_LSB_PER_DPS;
      float gz_dps = (gz - gz_bias) / GYRO_LSB_PER_DPS;

      Serial.printf("%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,±4g,±500dps,%u\n",
                    n, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps,
                    gx_bias, gy_bias, gz_bias, ax_bias, ay_bias, az_bias,
                    ACC_LSB_PER_G, GYRO_LSB_PER_DPS, (unsigned)DLPF_CFG);
    }
    delay(5); // ≈200 Hz output
  }
  Serial.println("END_CSV");
}

void loop(){}
