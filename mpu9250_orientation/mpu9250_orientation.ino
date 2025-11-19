#include <Wire.h>
#include <math.h>

// ====== MPU9250 + AK8963 I2C Addresses ======
#define MPU9250_ADDR   0x68
#define AK8963_ADDR    0x0C

// ====== MPU9250 Registers (MPU block) ======
#define MPU_WHO_AM_I   0x75
#define MPU_PWR_MGMT_1 0x6B
#define MPU_SMPLRT_DIV 0x19
#define MPU_CONFIG     0x1A
#define MPU_GYRO_CONFIG  0x1B
#define MPU_ACCEL_CONFIG 0x1C
#define MPU_ACCEL_XOUT_H 0x3B
#define MPU_INT_PIN_CFG  0x37

// ====== AK8963 Registers (mag block) ======
#define AK8963_WHO_AM_I   0x00
#define AK8963_ST1        0x02
#define AK8963_HXL        0x03
#define AK8963_ST2        0x09
#define AK8963_CNTL1      0x0A
#define AK8963_ASAX       0x10

// ====== Config (same accel/gyro as your 6050 code) ======
const uint8_t ACCEL_FS_4G = (1 << 3);   // ±4g
const uint8_t GYRO_FS_500 = (1 << 3);   // ±500 dps
const uint8_t DLPF_CFG    = 3;          // ~44 Hz accel / 42 Hz gyro

const float   ACC_LSB_PER_G    = 8192.0f; // ±4g
const float   GYRO_LSB_PER_DPS = 65.5f;   // ±500 dps

// Magnetometer: 16-bit, 4912 µT full scale -> 0.15 µT/LSB (exact: 4912/32760)
const float   MAG_LSB_PER_UT   = 0.15f;

// Calibration parameters (unchanged for accel/gyro)
const int CAL_DISCARD = 500;
const int CAL_SAMPLES = 8000;

// Sampling and fusion parameters
const int   SAMPLE_PERIOD_MS = 5;        // ~200 Hz
const float ALPHA_RP         = 0.98f;    // complementary filter roll/pitch
const float ALPHA_YAW        = 0.98f;    // complementary filter for yaw (gyro vs mag)

// Biases
float ax_bias=0, ay_bias=0, az_bias=0;
float gx_bias=0, gy_bias=0, gz_bias=0;

// Magnetometer scale/bias (basic placeholders; tune these by calibration)
float mx_soft_bias=0.0f, my_soft_bias=0.0f, mz_soft_bias=0.0f;   // µT offsets
float mx_soft_scale=1.0f, my_soft_scale=1.0f, mz_soft_scale=1.0f;

float mag_declination_deg = 0.0f; // set your local declination if desired

// Orientation state (Euler degrees)
float roll_deg  = 0.0f;
float pitch_deg = 0.0f;
float yaw_deg   = 0.0f;

// --- Small helpers ---
static inline float rad2deg(float r){ return r * (180.0f / (float)M_PI); }
static inline float deg2rad(float d){ return d * ((float)M_PI / 180.0f); }

// --- I2C helpers ---
void i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val){
  Wire.beginTransmission(addr);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission(true);
}

bool i2cReadN(uint8_t addr, uint8_t reg, uint8_t n, uint8_t* buf){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, (int)n, (int)true) != n) return false;
  for (int i=0;i<n;i++) buf[i]=Wire.read();
  return true;
}

// --- MPU9250 accel+gyro raw read ---
bool readRawAG(int16_t& ax,int16_t& ay,int16_t& az,
               int16_t& gx,int16_t& gy,int16_t& gz){
  uint8_t data[14];
  if (!i2cReadN(MPU9250_ADDR, MPU_ACCEL_XOUT_H, 14, data)) return false;
  ax = (int16_t)((data[0]<<8)|data[1]);
  ay = (int16_t)((data[2]<<8)|data[3]);
  az = (int16_t)((data[4]<<8)|data[5]);
  gx = (int16_t)((data[8]<<8)|data[9]);
  gy = (int16_t)((data[10]<<8)|data[11]);
  gz = (int16_t)((data[12]<<8)|data[13]);
  return true;
}

// --- AK8963 mag raw read (blocking single sample) ---
bool readRawMag(int16_t& mx, int16_t& my, int16_t& mz){
  uint8_t st1;
  if (!i2cReadN(AK8963_ADDR, AK8963_ST1, 1, &st1)) return false;
  if (!(st1 & 0x01)) return false; // no new data
  uint8_t data[7];
  if (!i2cReadN(AK8963_ADDR, AK8963_HXL, 7, data)) return false;
  uint8_t st2 = data[6];
  if (st2 & 0x08) return false; // overflow
  // Note: AK8963 is little-endian: L then H
  mx = (int16_t)((data[1]<<8) | data[0]);
  my = (int16_t)((data[3]<<8) | data[2]);
  mz = (int16_t)((data[5]<<8) | data[4]);
  return true;
}

// --- MPU9250 block init (accel+gyro + bypass enable for mag) ---
bool mpu9250_init(){
  // Wake MPU
  i2cWrite8(MPU9250_ADDR, MPU_PWR_MGMT_1, 0x00);
  delay(100);

  // WHO_AM_I check (0x71 for MPU9250, 0x73 for some variants)
  uint8_t who=0;
  if (!i2cReadN(MPU9250_ADDR, MPU_WHO_AM_I, 1, &who)) return false;
  if (who != 0x71 && who != 0x73) return false;

  // DLPF and sample rate
  i2cWrite8(MPU9250_ADDR, MPU_CONFIG, DLPF_CFG & 0x07);
  i2cWrite8(MPU9250_ADDR, MPU_SMPLRT_DIV, 4); // ~200 Hz from 1 kHz

  // FS ranges
  i2cWrite8(MPU9250_ADDR, MPU_GYRO_CONFIG,  GYRO_FS_500);
  i2cWrite8(MPU9250_ADDR, MPU_ACCEL_CONFIG, ACCEL_FS_4G);

  // Enable I2C bypass so host talks directly to AK8963
  i2cWrite8(MPU9250_ADDR, MPU_INT_PIN_CFG, 0x02); // BYPASS_EN=1

  return true;
}

// --- AK8963 init (magnetometer) ---
bool ak8963_init(){
  // Power-down
  i2cWrite8(AK8963_ADDR, AK8963_CNTL1, 0x00);
  delay(50);

  uint8_t who=0;
  if (!i2cReadN(AK8963_ADDR, AK8963_WHO_AM_I, 1, &who)) return false;
  if (who != 0x48) return false;

  // Enter Fuse ROM access mode to read ASA (factory sensitivity)
  i2cWrite8(AK8963_ADDR, AK8963_CNTL1, 0x0F);
  delay(50);

  uint8_t asa[3];
  if (!i2cReadN(AK8963_ADDR, AK8963_ASAX, 3, asa)) return false;

  // Optionally use ASA to refine scale; here we fold them into scale multipliers
  float x_adj = ((asa[0] - 128) * 0.5f / 128.0f) + 1.0f;
  float y_adj = ((asa[1] - 128) * 0.5f / 128.0f) + 1.0f;
  float z_adj = ((asa[2] - 128) * 0.5f / 128.0f) + 1.0f;
  mx_soft_scale = x_adj;
  my_soft_scale = y_adj;
  mz_soft_scale = z_adj;

  // Back to power-down
  i2cWrite8(AK8963_ADDR, AK8963_CNTL1, 0x00);
  delay(50);

  // Continuous measurement mode 2 (100 Hz), 16-bit output
  i2cWrite8(AK8963_ADDR, AK8963_CNTL1, 0x16);
  delay(50);

  return true;
}

// --- Orientation-agnostic accel/gyro calibration (same as your logic) ---
bool calibrate_ag() {
  const int DISCARD = CAL_DISCARD;
  const int N = CAL_SAMPLES;
  const int READ_DELAY_MS = 2;

  long long ax_sum=0, ay_sum=0, az_sum=0;
  long long gx_sum=0, gy_sum=0, gz_sum=0;
  int good = 0;

  for (int i = 0; i < DISCARD; i++) {
    int16_t ax,ay,az,gx,gy,gz;
    if (!readRawAG(ax,ay,az,gx,gy,gz)) return false;
    delay(READ_DELAY_MS);
  }
  for (int i = 0; i < N; i++) {
    int16_t ax,ay,az,gx,gy,gz;
    if (!readRawAG(ax,ay,az,gx,gy,gz)) return false;
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

  gx_bias = gx_mean;
  gy_bias = gy_mean;
  gz_bias = gz_mean;

  float ax_abs = fabsf(ax_mean), ay_abs = fabsf(ay_mean), az_abs = fabsf(az_mean);
  int up_idx = 0; float up_val = ax_abs;
  if (ay_abs > up_val) { up_idx = 1; up_val = ay_abs; }
  if (az_abs > up_val) { up_idx = 2; up_val = az_abs; }

  float up_g = up_val / ACC_LSB_PER_G;
  if (up_g < 0.7f || up_g > 1.3f) return false;

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
  Serial.printf("MAG_LSB_PER_UT,%.3f\n", MAG_LSB_PER_UT);
  Serial.println("CALIBRATION_OFFSETS_END");
}

// --- Tilt-compensated yaw from mag+accel (inputs: mag in µT, angles in rad) ---
float compute_mag_yaw_deg(float mx_uT, float my_uT, float mz_uT,
                          float roll_rad, float pitch_rad){
  // Tilt compensation
  float cr = cosf(roll_rad),  sr = sinf(roll_rad);
  float cp = cosf(pitch_rad), sp = sinf(pitch_rad);

  float mx_comp = mx_uT * cp + mz_uT * sp;
  float my_comp = mx_uT * sr * sp + my_uT * cr - mz_uT * sr * cp;

  float yaw_rad = atan2f(-my_comp, mx_comp); // ENU convention
  float yaw_deg = rad2deg(yaw_rad);

  yaw_deg += mag_declination_deg;
  if (yaw_deg > 180.f)  yaw_deg -= 360.f;
  if (yaw_deg < -180.f) yaw_deg += 360.f;

  return yaw_deg;
}

// --- Setup ---
void setup(){
  Serial.begin(115200);
  delay(200);
  Wire.begin(); // or Wire.begin(SDA,SCL,400000);

  Serial.println("INIT: MPU9250 (AG) over I2C...");
  if(!mpu9250_init()){ Serial.println("INIT_FAIL_MPU"); return; }
  Serial.println("INIT_OK_MPU");

  Serial.println("INIT: AK8963 (mag)...");
  if(!ak8963_init()){ Serial.println("INIT_FAIL_MAG"); return; }
  Serial.println("INIT_OK_MAG");

  Serial.println("CALIBRATION_START: keep the sensor flat and still...");
  delay(2000);

  if(!calibrate_ag()){
    Serial.println("CALIBRATION_FAIL");
    print_offsets();
    return;
  }
  Serial.println("CALIBRATION_OK");
  print_offsets();

  // Initialize roll/pitch from accelerometer (yaw from mag)
  int16_t ax,ay,az,gx,gy,gz;
  if (readRawAG(ax,ay,az,gx,gy,gz)) {
    float ax_g = (ax - ax_bias) / ACC_LSB_PER_G;
    float ay_g = (ay - ay_bias) / ACC_LSB_PER_G;
    float az_g = (az - az_bias) / ACC_LSB_PER_G;

    roll_deg  = rad2deg(atan2f(ay_g, az_g));
    pitch_deg = rad2deg(atan2f(-ax_g, sqrtf(ay_g*ay_g + az_g*az_g)));
  }

  // Initial yaw from mag (if available)
  int16_t mx,my,mz;
  if (readRawMag(mx,my,mz)){
    float mx_uT = ((float)mx * MAG_LSB_PER_UT * mx_soft_scale) - mx_soft_bias;
    float my_uT = ((float)my * MAG_LSB_PER_UT * my_soft_scale) - my_soft_bias;
    float mz_uT = ((float)mz * MAG_LSB_PER_UT * mz_soft_scale) - mz_soft_bias;
    yaw_deg = compute_mag_yaw_deg(mx_uT, my_uT, mz_uT,
                                  deg2rad(roll_deg), deg2rad(pitch_deg));
  } else {
    yaw_deg = 0.0f;
  }

  Serial.println("ORIENTATION_READY");
  Serial.println("Header: roll_deg,pitch_deg,yaw_deg");
}

// --- Main loop ---
void loop(){
  static uint32_t last_ms = millis();
  uint32_t now = millis();
  uint32_t dt_ms = now - last_ms;
  if (dt_ms < (uint32_t)SAMPLE_PERIOD_MS) return;
  last_ms = now;
  float dt = dt_ms * 1e-3f;

  int16_t ax,ay,az,gx,gy,gz;
  if (!readRawAG(ax,ay,az,gx,gy,gz)) return;

  // Bias-correct and scale
  float ax_g = (ax - ax_bias) / ACC_LSB_PER_G;
  float ay_g = (ay - ay_bias) / ACC_LSB_PER_G;
  float az_g = (az - az_bias) / ACC_LSB_PER_G;
  float gx_dps = (gx - gx_bias) / GYRO_LSB_PER_DPS;
  float gy_dps = (gy - gy_bias) / GYRO_LSB_PER_DPS;
  float gz_dps = (gz - gz_bias) / GYRO_LSB_PER_DPS;

  // 1) Integrate gyro (deg)
  roll_deg  += gx_dps * dt;
  pitch_deg += gy_dps * dt;
  yaw_deg   += gz_dps * dt;

  // 2) Compute accel-only tilt
  float norm = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
  if (norm > 0.5f && norm < 2.0f) {
    float roll_acc  = rad2deg(atan2f(ay_g, az_g));
    float pitch_acc = rad2deg(atan2f(-ax_g, sqrtf(ay_g*ay_g + az_g*az_g)));
    roll_deg  = ALPHA_RP * roll_deg  + (1.0f - ALPHA_RP) * roll_acc;
    pitch_deg = ALPHA_RP * pitch_deg + (1.0f - ALPHA_RP) * pitch_acc;
  }

  // 3) Magnetometer-assisted yaw
  int16_t mx,my,mz;
  if (readRawMag(mx,my,mz)){
    float mx_uT = ((float)mx * MAG_LSB_PER_UT * mx_soft_scale) - mx_soft_bias;
    float my_uT = ((float)my * MAG_LSB_PER_UT * my_soft_scale) - my_soft_bias;
    float mz_uT = ((float)mz * MAG_LSB_PER_UT * mz_soft_scale) - mz_soft_bias;
    float yaw_mag_deg = compute_mag_yaw_deg(mx_uT, my_uT, mz_uT,
                                            deg2rad(roll_deg), deg2rad(pitch_deg));
    yaw_deg = ALPHA_YAW * yaw_deg + (1.0f - ALPHA_YAW) * yaw_mag_deg;
  }

  // Wrap yaw
  if (yaw_deg > 180.f) yaw_deg -= 360.f;
  else if (yaw_deg < -180.f) yaw_deg += 360.f;

  Serial.printf("%.2f,%.2f,%.2f\n", roll_deg, pitch_deg, yaw_deg);
}
