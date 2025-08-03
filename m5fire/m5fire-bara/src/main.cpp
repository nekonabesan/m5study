#include <M5Stack.h>
#include <Wire.h>
#include "BaseX.h"
BASE_X baseX;

// 関数プロトタイプ宣言
void calculatePID();
void controlMotor();
void sendMotorCommand(int power);
void sendDualMotorCommand(int power3, int power4);
void updateDisplay();
void adjustParameters();

// PID制御パラメータ
float kp = 30.0;  // 比例ゲイン
float ki = 0.5;   // 積分ゲイン
float kd = 1.5;   // 微分ゲイン

// IMU関連変数
float pitch = 0.0;          // ピッチ角（傾斜角）
float target_angle = 0.0;   // 目標角度
float error = 0.0;          // 誤差
float last_error = 0.0;     // 前回の誤差
float integral = 0.0;       // 積分項
float derivative = 0.0;     // 微分項
float pid_output = 0.0;     // PID制御出力

// モータ制御用変数
int motor_power = 0;        // 基本モータ出力パワー
int motor3_power = 0;       // MOTOR3出力パワー
int motor4_power = 0;       // MOTOR4出力パワー
unsigned long last_time = 0;
float dt = 0.01;            // 制御周期（10ms）

// BaseXのI2Cアドレス（通常は0x26）
#define BASEX_ADDR 0x26

// BaseXモータアドレス（公式仕様）
#define MOTOR1_ADDR 0x50
#define MOTOR2_ADDR 0x60
#define MOTOR3_ADDR 0x70
#define MOTOR4_ADDR 0x80

// 倒立振子で使用するモータ（MOTOR3とMOTOR4を使用）
#define MOTOR_LEFT MOTOR3_ADDR   // 左モータ
#define MOTOR_RIGHT MOTOR4_ADDR  // 右モータ

void setup() {
    M5.begin();
    M5.Power.begin();
    
    // シリアル通信初期化
    Serial.begin(115200);
    
    // I2C初期化（BaseX通信用）
    Wire.begin();

    // BaseX初期化
    delay(100); // BaseX起動待ち
    
    // BaseX モーター初期設定
    baseX.SetMode(3, NORMAL_MODE); // MOTOR3をNORMAL_MODEに設定
    baseX.SetMode(4, NORMAL_MODE); // MOTOR4をNORMAL_MODEに設定
    
    // モータを停止状態で初期化
    baseX.SetMotorSpeed(3, 0);
    baseX.SetMotorSpeed(4, 0);
    
    Serial.println("BaseX Motors initialized");
    
    // IMU初期化
    M5.IMU.Init();
    
    // LCD表示設定
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.println("Inverted Pendulum");
    M5.Lcd.println("Press A to start");
    
    // 初期化完了メッセージ
    Serial.println("M5Fire Inverted Pendulum Controller");
    Serial.println("IMU and Motor initialized");
    
    delay(1000);
}

void loop() {
    M5.update();
    
    // IMUデータ読み取り
    float gyroX, gyroY, gyroZ;
    float accX, accY, accZ;
    
    M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
    M5.IMU.getAccelData(&accX, &accY, &accZ);
    
    // 傾斜角計算（加速度センサーから）
    pitch = atan2(accY, accZ) * 180.0 / PI;
    
    // ボタン処理
    if (M5.BtnA.wasPressed()) {
        // PID制御開始/停止
        integral = 0.0;  // 積分項リセット
        target_angle = pitch;  // 現在の角度を目標角度に設定
        Serial.println("Control started");
    }
    
    if (M5.BtnB.wasPressed()) {
        // 緊急停止
        motor_power = 0;
        sendDualMotorCommand(0, 0);
        Serial.println("Emergency stop");
    }
    
    if (M5.BtnC.wasPressed()) {
        // パラメータ調整モード
        adjustParameters();
    }
    
    // PID制御計算
    unsigned long current_time = millis();
    if (current_time - last_time >= 10) {  // 10ms周期
        calculatePID();
        
        // モータ制御
        controlMotor();
        
        last_time = current_time;
    }
    
    // LCD表示更新
    updateDisplay();
    
    // シリアル出力（デバッグ用）
    if (current_time % 100 == 0) {  // 100ms毎
        Serial.printf("Pitch: %.2f, Error: %.2f, PID: %.2f, M3: %d, M4: %d\n", 
                     pitch, error, pid_output, motor3_power, motor4_power);
    }
    
    delay(5);
}

void calculatePID() {
    // 誤差計算
    error = target_angle - pitch;
    
    // 積分項計算（ワインドアップ対策付き）
    integral += error * dt;
    if (integral > 100) integral = 100;
    if (integral < -100) integral = -100;
    
    // 微分項計算
    derivative = (error - last_error) / dt;
    
    // PID出力計算
    pid_output = kp * error + ki * integral + kd * derivative;
    
    // 出力制限
    if (pid_output > 255) pid_output = 255;
    if (pid_output < -255) pid_output = -255;
    
    last_error = error;
}

void controlMotor() {
    // PID出力をモータパワーに変換
    motor_power = (int)pid_output;
    
    // モータが動作範囲内の傾斜角の場合のみ制御
    if (abs(pitch) < 45.0) {  // 45度以内
        // 両輪同じ方向に回転（前進・後退による姿勢制御）
        motor3_power = motor_power;  // 左モータ
        motor4_power = motor_power;  // 右モータ
        sendDualMotorCommand(motor3_power, motor4_power);
    } else {
        // 倒れすぎた場合は停止
        motor3_power = 0;
        motor4_power = 0;
        sendDualMotorCommand(0, 0);
    }
}

void sendMotorCommand(int power) {
    // 単一モータ制御（MOTOR3を使用）
    Wire.beginTransmission(BASEX_ADDR);
    Wire.write(MOTOR_LEFT);                     // モータアドレス（0x70 = MOTOR3）
    Wire.write((uint8_t)(power & 0xFF));        // 下位バイト
    Wire.write((uint8_t)((power >> 8) & 0xFF)); // 上位バイト
    Wire.endTransmission();
}

void sendDualMotorCommand(int power3, int power4) {
    // BaseXライブラリを使用してモータ制御
    // 範囲を-127〜127に制限（BaseXの仕様に合わせて調整）
    int motor3_speed = constrain(power3, -127, 127);
    int motor4_speed = constrain(power4, -127, 127);
    
    // BaseXライブラリでモータ制御（正しいメソッド名を使用）
    baseX.SetMotorSpeed(3, motor3_speed); // MOTOR3
    baseX.SetMotorSpeed(4, motor4_speed); // MOTOR4
    
    // デバッグ出力
    Serial.printf("Motor commands: M3=%d, M4=%d\n", motor3_speed, motor4_speed);
}


void updateDisplay() {
    // LCD表示更新
    M5.Lcd.fillRect(0, 60, 320, 180, BLACK);
    M5.Lcd.setCursor(0, 60);
    M5.Lcd.printf("Pitch: %.2f deg\n", pitch);
    M5.Lcd.printf("Target: %.2f deg\n", target_angle);
    M5.Lcd.printf("Error: %.2f\n", error);
    M5.Lcd.printf("PID Out: %.2f\n", pid_output);
    M5.Lcd.printf("M3: %d  M4: %d\n", motor3_power, motor4_power);
    
    // 制御状態表示
    if (abs(pitch) >= 45.0) {
        M5.Lcd.setTextColor(RED);
        M5.Lcd.printf("TILT STOP!\n");
        M5.Lcd.setTextColor(WHITE);
    } else {
        M5.Lcd.setTextColor(GREEN);
        M5.Lcd.printf("ACTIVE\n");
        M5.Lcd.setTextColor(WHITE);
    }
    
    M5.Lcd.printf("Kp:%.1f Ki:%.1f Kd:%.1f", kp, ki, kd);
    
    // 傾斜角の視覚表示
    int center_x = 160;
    int center_y = 200;
    int line_length = 50;
    int end_x = center_x + line_length * sin(pitch * PI / 180.0);
    int end_y = center_y - line_length * cos(pitch * PI / 180.0);
    
    M5.Lcd.drawLine(center_x, center_y, end_x, end_y, WHITE);
    M5.Lcd.drawCircle(center_x, center_y, 3, RED);
}

void adjustParameters() {
    // パラメータ調整モード（簡易版）
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.println("Parameter Adjust Mode");
    M5.Lcd.println("A: Kp+  B: Ki+  C: Kd+");
    
    bool adjust_mode = true;
    while (adjust_mode) {
        M5.update();
        
        if (M5.BtnA.wasPressed()) {
            kp += 1.0;
            if (kp > 100) kp = 100;
        }
        if (M5.BtnB.wasPressed()) {
            ki += 0.1;
            if (ki > 10) ki = 10;
        }
        if (M5.BtnC.wasPressed()) {
            kd += 0.1;
            if (kd > 10) kd = 10;
        }
        
        // 長押しで調整モード終了
        if (M5.BtnA.pressedFor(2000) || M5.BtnB.pressedFor(2000) || M5.BtnC.pressedFor(2000)) {
            adjust_mode = false;
        }
        
        // パラメータ表示
        M5.Lcd.fillRect(0, 100, 320, 140, BLACK);
        M5.Lcd.setCursor(0, 100);
        M5.Lcd.printf("Kp: %.1f\n", kp);
        M5.Lcd.printf("Ki: %.1f\n", ki);
        M5.Lcd.printf("Kd: %.1f\n", kd);
        M5.Lcd.println("\nHold button to exit");
        
        delay(100);
    }
    
    // メイン画面に戻る
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.println("Inverted Pendulum");
}