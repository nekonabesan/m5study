// (参)M5 Stack が公式に提供しているソースコード
// https://github.com/m5stack/M5-ProductExampleCodes/blob/master/App/BalaC/Arduino/Balac/Balac.ino
#include "header.h"

#define LED 10
#define N_CAL1 100
#define N_CAL2 100
#define LCDV_MID 60
#define MOTOR_I2C_ADDRESS 0x38
#define MAX_PWM_VALUE 127
#define MIN_PWM_VALUE -127

boolean serial_monitor = true;
boolean standing = false;
int16_t counter = 0;
uint32_t time0 = 0;
uint32_t time1 = 0;
int16_t counter_over_power = 0, max_over_power = 20;
float power, power_r, power_l, yaw_power;
float var_angle, var_omg, var_spd, var_dst, var_i_angle;
float gyro_x_offset, gyro_y_offset, gyro_z_offset, acc_x_offset;
float gyro_x_data, gyro_y_data, gyro_z_data, acc_x_data, acc_z_data;
float average_acc_x = 0.0, average_acc_z = 0.0, average_abs_omg = 0.0;
float cutoff = 0.1; //~=2 * pi * f [Hz]
const float clk = 0.01;
const uint32_t interval = (uint32_t)(clk * 1000);// in msec
float K_angle, K_omg, KI_angle, K_yaw, K_dst, K_speed;
int16_t max_power;
float yaw_angle = 0.0;
float move_destination, move_target;
float move_rate;
const float move_step = 0.2 * clk;
int16_t fb_balance = 0;
int16_t motor_dependand = 0;
float mach_fact_r, mach_fact_l;
//int8_t motor_r_dir = 0, motor_l_dir = 0;
bool spin_continuous = false;
float spin_dest, spin_target, spin_fact = 1.0;
float spin_step = 0.0;
int16_t ipower_l = 0, ipower_r = 0;
int16_t motor_l_dir = 0, motor_r_dir = 0;
float v_batt, volt_average = 3.7;
int16_t punch_power, punch_power_2, punch_dir, punch_control_l, punch_control_r;
byte demo_mode = 0;


/**
 * @brief 初期化処理
 * 
 * @param void
 * @return void
 * 
 * 処理概要：
 * - 
 * - 
 * -
 */
void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  M5.begin();
  Wire.begin(0, 26);
  imuInit();
  M5.Axp.ScreenBreath(11);
  M5.Lcd.setRotation(2);
  M5.Lcd.setTextFont(4);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(1);
  resetMotor();
  resetPara();
  resetVar();
  caliblation1();
  #ifdef DEBUG
  debugSetup();
  #else
  setMode(false);
  #endif
}

/**
 * @brief 制御周期毎にポーリングされる処理
 * 
 * @param void
 * @return void
 * 
 * 処理概要：
 * - ボタンの状態をチェックし、必要に応じてモードを切り替える
 * - IMUからデータを取得するための遅延を導出する
 * - モータの制御を行う
 */

void loop(void) {
    checkButtonP();
    #ifdef DEBUG
    if (debugLoop1()) return;
    #endif
    getGyro();
    if (standing == false) {
        displayBatteryVoltage();
        average_abs_omg = average_abs_omg * 0.9 + abs(var_omg) * 0.1;
        average_acc_z = average_acc_z * 0.9 + acc_z_data * 0.1;
        M5.Lcd.setCursor(10, 130);
        M5.Lcd.printf("%5.2f  ", -average_acc_z);
        if (abs(average_acc_z) > 0.9 && average_abs_omg < 1.5) {
            caliblation2();
            if (demo_mode == 1) startDemo();
            standing = true;
        }
    } else {
        if (abs(var_angle) > 30.0 || counter_over_power > max_over_power) {
            resetMotor();
            resetVar();
            standing = false;
            setMode(false);        
        } else {
            driver();
        }
    }
    counter += 1;
    if (counter >= 100) {
        counter = 0;
        displayBatteryVoltage();
        if (serial_monitor == true) sendStatus();
    }
    do time1 = millis();
    while (time1 - time0 < interval);
    time0 = time1;
}

/**
 * @brief IMUをキャリブレーションする
 * 
 * @param void
 * @return void
 * 
 * 処理概要：
 * - 変数を初期化
 * - モータを初期化
 * - 各種オフセットを初期化
 * - ジャイロセンサーと加速度センサーからデータを取得し、オフセットを計算する
 * - キャリブレーション後に画面をクリアし、LEDを点灯する
 */
void caliblation1(void)
{
    calDelay(30);
    digitalWrite(LED, LOW);
    calDelay(80);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, LCDV_MID);
    M5.Lcd.print(" Calibration 1 ");
    gyro_y_offset = 0.0;
    for (int i = 0; i < N_CAL1; i++) {
        readGyro();
        gyro_y_offset += gyro_y_data;
        delay(9);
    }
    gyro_y_offset /= (float)N_CAL1;
    M5.Lcd.fillScreen(BLACK);
    digitalWrite(LED, HIGH);
}

/**
 * @brief IMUをキャリブレーションする
 * 
 * @param void
 * @return void
 * 
 * 処理概要：
 * - 変数を初期化
 * - モータを初期化
 * - 各種オフセットを初期化
 * - ジャイロセンサーと加速度センサーからデータを取得し、オフセットを計算する
 * - キャリブレーション後に画面をクリアし、LEDを点灯する
 */
void caliblation2(void)
{
    resetVar();
    resetMotor();
    digitalWrite(LED, LOW);
    calDelay(80);
    M5.Lcd.setCursor(0, LCDV_MID);
    M5.Lcd.print(" Calibration 2 ");
    acc_x_offset = 0.0;
    gyro_z_offset = 0.0;
    for (int i = 0; i < N_CAL2; i++) {
        readGyro();
        acc_x_offset += acc_x_data;
        gyro_z_offset += gyro_z_data;
        delay(9);
    }
    acc_x_offset /= (float)N_CAL2;
    gyro_z_offset /= (float)N_CAL2;
    M5.Lcd.fillScreen(BLACK);
    digitalWrite(LED, HIGH);
}

/**
 * @brief ボタンの状態をチェックし、必要に応じてモードを切り替える
 * 
 * @param void
 * @return void
 * 
 * 処理概要：
 * - ボタンの状態をチェックし、必要に応じてモードを切り替える
 */
void checkButtonP(void)
{
    byte pbtn = M5.Axp.GetBtnPress();
    if (pbtn == 2) caliblation1();
    else if (pbtn == 1) setMode(true);
}

/**
 * @brief IMUからデータを取得するための遅延を計算する
 * 
 * @param int n
 * @return void
 * 
 * 処理概要：
 * - IMUからデータを取得するための遅延を導出する
 * - 
 */
void calDelay(int n)
{
    for (int i = 0; i < n; i++) {
        getGyro();
        delay(9);
    }
}

/**
 * @brief モードを切り替える
 * 
 * @param void
 * @return void
 * 
 * 処理概要：
 * - デモモードとスタンバイモードを切り替える
 */

void setMode(bool inc)
{
    if (inc) demo_mode = ++demo_mode % 2;
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(5, 5);
    if (demo_mode == 0) M5.Lcd.print("Stand ");  
    else if (demo_mode == 1) M5.Lcd.print("Demo ");
}

/**
 * @brief TEXT MESSAGE
 * 
 * @param void
 * @return void
 * 
 * 処理概要：
 * - 
 * - 
 * -
 */
void startDemo(void)
{
    move_rate = 1.0;
    spin_continuous = true;
    spin_step = - 40.0 * clk;
}

/**
 * @brief 角度と速度の変数をリセット
 * 
 * @param void
 * @return void
 * 
 * 処理概要：
 * - 角度と速度の変数をリセットする
 * - 各種オフセットを初期化する
 */
void resetPara(void)
{
    K_angle = 37.0;
    K_omg = 0.84;
    KI_angle = 800.0;
    K_yaw = 4.0;
    K_dst = 85.0;
    K_speed = 2.7;
    mach_fact_r = 0.45;
    mach_fact_l = 0.45;
    punch_power = 20;
    punch_dir = 1;
    fb_balance = -3;
    motor_dependand = 10;
    max_power = 120;
    punch_power_2 = max(punch_power, motor_dependand);
}

/**
 * @brief IMUから取得したデータからオフセットを除去する
 * 
 * @param void
 * @return void
 * 
 * 処理概要：
 * - ジャイロセンサーからのデータを取得し、各種角度を更新する
 */
void getGyro(void)
{
    readGyro();
    var_omg = (gyro_y_data - gyro_y_offset);
    yaw_angle += (gyro_z_data - gyro_z_offset) * clk;
    var_angle += (var_omg + ((acc_x_data - acc_x_offset) * 57.3 - var_angle) * cutoff) * clk;
}

/**
 * @brief IMUからデータを取得
 * 
 * @param void
 * @return void
 * 
 * 処理概要：
 * - ジャイロセンサーからのデータを取得し、各種角度を更新する
 */

void readGyro(void)
{
    float gx, gy, gz, ax, ay, az;
    M5.Imu.getGyroData(&gx, &gy, &gz);
    M5.Imu.getAccelData(&ax, &ay, &az);
    gyro_y_data = gx;
    gyro_z_data = gy;
    gyro_x_data = gz;
    acc_x_data = az;
    acc_z_data = ay;
}

/**
 * @brief モータ駆動
 * 
 * @param void
 * @return void
 * 
 * 処理概要：
 * - モータの駆動を制御する
 */
void driver(void)
{
    #ifdef DEBUG
    debugDriver();
    #endif
    // Calculate the power for the motors
    if (abs(move_rate) > 0.1) spin_fact = constrain(-(power_r + power_l) / 10.0, -1.0, 1.0);
    else spin_fact = 1.0;
    // If spin_continuous is true, spin_target will be updated continuously
    if (spin_continuous == true) {
        spin_target += spin_step * spin_fact;
    } else {
        if (spin_target < spin_dest) spin_target += spin_step;
        if (spin_target > spin_dest) spin_target -= spin_step;
    }
    // Calculate the target move destination
    move_target += move_step * (move_rate + (float)fb_balance / 100.0);
    var_spd += power * clk;
    var_dst += K_dst * (var_spd * clk - move_target);
    var_i_angle += KI_angle * var_angle * clk;
    power = var_i_angle + var_dst + (K_speed * var_spd) + (K_angle * var_angle) + (K_omg * var_omg);
    // Limit the power to prevent overloading
    if (abs(power) > 1000.0) counter_over_power += 1;
    else counter_over_power = 0;
    // If the counter exceeds the maximum allowed, return without driving the motors
    if (counter_over_power > max_over_power) return;
    // Constrain the power to the maximum allowed value
    power = constrain(power, -max_power, max_power);
    yaw_power = (yaw_angle - spin_target) * K_yaw;
    power_r = power + yaw_power;
    power_l = power - yaw_power;
    // Calculate the power for the motors
    int16_t mdbn = - motor_dependand;
    int16_t pp2n =  - punch_power_2;
    // Left motor control
    ipower_l = (int16_t) constrain(power_l * mach_fact_l, -max_power, max_power);
    if (power_l > 0) {
        // If the left motor is moving forward
        if (motor_l_dir == 1) punch_control_l = constrain(++punch_control_l, 0, 100);
        else punch_control_l = 0; 
        motor_l_dir = 1;
        if (punch_control_l < punch_dir) driveMotorL(max(ipower_l, mdbn));
        else driveMotorL(max(ipower_l, motor_dependand));
    } else if (ipower_l < 0) {
        // If the left motor is moving backward
        if (motor_l_dir ==  -1) punch_control_l = constrain(++punch_control_l, 0, 100);
        else punch_control_l = 0;
        motor_l_dir = -1;
        if (punch_control_l < punch_dir) driveMotorL(min(ipower_l, pp2n));
        else driveMotorL(min(ipower_l, mdbn));
    } else {
        // If the left motor is not moving
        driveMotorL(0);
        motor_l_dir = 0;
    }
    // Right motor control
    ipower_r = (int16_t) constrain(power_r * mach_fact_r, -max_power, max_power);
    if (power_r > 0) {
        // If the right motor is moving forward
        if (motor_r_dir == 1) punch_control_r = constrain(++punch_control_r, 0, 100);
        else punch_control_r = 0; 
        motor_r_dir = 1;
        if (punch_control_r < punch_dir) driveMotorR(max(ipower_r, mdbn));
        else driveMotorR(max(ipower_r, motor_dependand));
    } else if (ipower_r < 0) {
        // If the right motor is moving backward
        if (motor_r_dir == -1) punch_control_r = constrain(++punch_control_r, 0, 100);
        else punch_control_r = 0;
        motor_r_dir = -1;
        if (punch_control_r < punch_dir) driveMotorR(min(ipower_r, pp2n));
        else driveMotorR(min(ipower_r, mdbn));
    } else {
        // If the right motor is not moving
        driveMotorR(0);
        motor_r_dir = 0;
    }
}

/**
 * @brief MOVE LEFT MOTOR
 * 
 * @param int16_t pwm
 * @return void
 * 
 * 処理概要：
 * - int16 pwm: PWM value for the left motor
 */
void driveMotorL(int16_t pwm)
{
    driveMotor(0, (int8_t)constrain(pwm, MIN_PWM_VALUE, MAX_PWM_VALUE));
}

/**
 * @brief MOVE RIGHT MOTOR
 * 
 * @param int16_t pwm
 * @return void
 * 
 * 処理概要：
 * - int16 pwm: PWM value for the right motor
 */
void driveMotorR(int16_t pwm)
{
    driveMotor(1, (int8_t)constrain(-pwm, MIN_PWM_VALUE, MAX_PWM_VALUE));
}

/**
 * @brief モータドライバにパラメータを与える
 * 
 * @param byte ch
 * @param int8_t speed
 * @return void
 * 
 * 処理概要：
 * - モータのチャネルを指定する
 * - 指定したチャネルにPWM値を送信する
 */
void driveMotor(byte ch, int8_t speed)
{
    Wire.beginTransmission(MOTOR_I2C_ADDRESS);
    Wire.write(ch);
    Wire.write(speed);
    Wire.endTransmission();
}

/**
 * @brief PWM値を0にしてモータを停止する
 * 
 * @param void
 * @return void
 * 
 * 処理概要：
 * - Left and right motors are stopped by sending 0 PWM values
 * - 出力超過回数をリセットする
 */
void resetMotor(void)
{
    driveMotorL(0);
    driveMotorR(0);
    counter_over_power = 0;
}

/**
 * @brief 変数を初期化する
 * 
 * @param void
 * @return void
 * 
 * 処理概要：
 * - 動的に更新される変数を初期化する
 */
void resetVar(void)
{
    power = 0.0;
    move_target = 0.0;
    move_rate = 0.0;
    spin_continuous = false;
    spin_dest = 0.0;
    spin_target = 0.0;
    spin_step = 0.0;
    yaw_angle = 0.0;
    var_angle = 0.0;
    var_omg = 0.0;
    var_spd = 0.0;
    var_dst = 0.0;
    var_i_angle = 0.0;
}

/**
 * @brief LCDにステータスを表示する
 * 
 * @param void
 * @return void
 * 
 * 処理概要：
 * - LCDに現在のステータスを表示する
 * - 各種センサーデータを表示する
 * - モータの状態を表示する
 */
void sendStatus(void)
{
    Serial.print(millis() - time0);
    Serial.print("  stand="); Serial.print(standing);
    Serial.print("  acc_x="); Serial.print(acc_x_data);
    Serial.print("  power="); Serial.print(power);
    Serial.print("  angle="); Serial.print(var_angle);
    Serial.print(", ");
    Serial.print(millis() - time0);
    Serial.println();
}

/**
 * @brief IMUを初期化する
 * 
 * @param void
 * @return void
 * 
 * 処理概要：
 * - M5StickC内蔵のMPU6886を初期化する
 */
void imuInit(void)
{
    M5.Imu.Init();
    if (M5.Imu.imuType == M5.Imu.IMU_MPU6886) {
        M5.Mpu6886.SetGyroFsr(M5.Mpu6886.GFS_250DPS);
        M5.Mpu6886.SetAccelFsr(M5.Mpu6886.AFS_4G);
        if (serial_monitor == true) Serial.println("MPU6886 found");
    } else if (serial_monitor == true) {
        Serial.println("MPU6886 not found");
    }
}

/**
 * @brief LCDにバッテリ電圧を表示する
 * 
 * @param void
 * @return void
 * 
 * 処理概要：
 * - LCDにバッテリ電圧を表示する
 * - バッテリ電圧はVバッテリ変数に格納される
 * - LCDのカーソル位置は(5, LCDV_MID)に設定される
 */
void displayBatteryVoltage(void)
{
    M5.Lcd.setCursor(5, LCDV_MID);
    v_batt = M5.Axp.GetBatVoltage();
    M5.Lcd.printf("%4.2fv ", v_batt);
}
