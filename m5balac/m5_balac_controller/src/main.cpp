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
 * @brief システム初期化処理
 * 
 * @param void
 * @return void
 * 
 * 処理概要：
 * - LED、M5StickC、I2C通信の初期化
 * - IMUセンサー（MPU6886）の初期化
 * - LCD画面の設定（回転、フォント、サイズ）
 * - モータ制御の初期化とリセット
 * - 制御パラメータの設定
 * - 制御変数の初期化
 * - 第1回キャリブレーション（ジャイロオフセット計算）実行
 * - デバッグモード/通常モードの分岐処理
 */
void setup() {
  pinMode(LED, OUTPUT);        // LED用ピンを出力モードに設定
  digitalWrite(LED, HIGH);     // LED点灯（初期化開始を表示）
  M5.begin();                  // M5StickCライブラリの初期化
  Wire.begin(0, 26);           // I2C通信初期化（SDA=0, SCL=26）
  imuInit();                   // IMUセンサーの初期化
  M5.Axp.ScreenBreath(11);     // LCD画面の輝度設定
  M5.Lcd.setRotation(2);       // 画面回転（180度）
  M5.Lcd.setTextFont(4);       // フォントサイズ設定
  M5.Lcd.fillScreen(BLACK);    // 画面を黒色でクリア
  M5.Lcd.setTextSize(1);       // テキストサイズを1に設定
  resetMotor();                // モータを停止状態に初期化
  resetPara();                 // 制御パラメータを初期値に設定
  resetVar();                  // 制御変数を初期化
  caliblation1();              // 第1回キャリブレーション実行
  #ifdef DEBUG
  debugSetup();                // デバッグモード初期化
  #else
  setMode(false);              // 通常モード（現在モード表示のみ）
  #endif
}

/**
 * @brief メイン制御ループ（制御周期10ms）
 * 
 * @param void
 * @return void
 * 
 * 処理概要：
 * - 電源ボタンの状態チェック（モード切替・再キャリブレーション）
 * - IMUセンサーからの傾斜角・角速度データ取得
 * - 倒立状態の判定と制御の分岐
 *   [非倒立時] バッテリー電圧表示、倒立開始条件の監視
 *   [倒立時] 転倒検出、PID制御によるモータ駆動
 * - 定期的なバッテリー電圧表示とシリアル通信
 * - 制御周期の調整（10ms間隔の維持）
 */

void loop(void) {
    checkButtonP();              // 電源ボタンの状態をチェック
    #ifdef DEBUG
    if (debugLoop1()) return;    // デバッグモード処理（有効時は早期リターン）
    #endif
    getGyro();                   // IMUデータ取得と角度計算
    if (standing == false) {
        displayBatteryVoltage(); // バッテリー電圧をLCDに表示
        // 角速度の移動平均を計算（ノイズ除去）
        average_abs_omg = average_abs_omg * 0.9 + abs(var_omg) * 0.1;
        // Z軸加速度の移動平均を計算（起立判定用）
        average_acc_z = average_acc_z * 0.9 + acc_z_data * 0.1;
        M5.Lcd.setCursor(10, 130);
        M5.Lcd.printf("%5.2f  ", -average_acc_z);  // Z軸加速度を表示
        // 倒立開始条件：Z軸加速度が0.9G以上かつ角速度が1.5以下
        if (abs(average_acc_z) > 0.9 && average_abs_omg < 1.5) {
            caliblation2();      // 第2回キャリブレーション実行
            if (demo_mode == 1) startDemo();  // デモモード時は回転動作開始
            standing = true;     // 倒立状態フラグをセット
        }
    } else {
        // 転倒検出：傾斜角30度超過または過負荷カウンター超過
        if (abs(var_angle) > 30.0 || counter_over_power > max_over_power) {
            resetMotor();        // モータ緊急停止
            resetVar();          // 制御変数リセット
            standing = false;    // 倒立状態フラグをクリア
            setMode(false);      // モード表示更新
        } else {
            driver();            // PID制御によるモータ駆動
        }
    }
    counter += 1;                // ループカウンターを増加
    if (counter >= 100) {        // 100ループ毎（約1秒毎）の処理
        counter = 0;             // カウンターリセット
        displayBatteryVoltage(); // バッテリー電圧表示更新
        if (serial_monitor == true) sendStatus();  // シリアル出力
    }
    // 制御周期の調整（10ms間隔を維持）
    do time1 = millis();
    while (time1 - time0 < interval);
    time0 = time1;
}

/**
 * @brief 第1回IMUキャリブレーション（起動時）
 * 
 * @param void
 * @return void
 * 
 * 処理概要：
 * - システム起動時に実行されるジャイロセンサーのオフセット計算
 * - LED点滅による処理状態の表示
 * - LCD画面にキャリブレーション状態を表示
 * - N_CAL1回数（100回）のジャイロY軸データを取得
 * - ジャイロY軸のオフセット値を平均値として計算・保存
 * - 処理完了後、画面クリアとLED点灯
 */
void caliblation1(void)
{
    calDelay(30);                // センサー安定化待機（30回×9ms）
    digitalWrite(LED, LOW);      // LED消灯（キャリブレーション開始表示）
    calDelay(80);                // さらに安定化待機（80回×9ms）
    M5.Lcd.fillScreen(BLACK);    // 画面クリア
    M5.Lcd.setCursor(0, LCDV_MID);
    M5.Lcd.print(" Calibration 1 ");  // キャリブレーション状態表示
    gyro_y_offset = 0.0;         // ジャイロY軸オフセット初期化
    for (int i = 0; i < N_CAL1; i++) {
        readGyro();              // IMU生データ読み取り
        gyro_y_offset += gyro_y_data;  // Y軸ジャイロデータを累積
        delay(9);                // 9ms待機（安定したサンプリング）
    }
    gyro_y_offset /= (float)N_CAL1;  // 平均値を計算してオフセット値とする
    M5.Lcd.fillScreen(BLACK);    // 画面クリア
    digitalWrite(LED, HIGH);     // LED点灯（キャリブレーション完了表示）
}

/**
 * @brief 第2回IMUキャリブレーション（倒立開始前）
 * 
 * @param void
 * @return void
 * 
 * 処理概要：
 * - 倒立制御開始前に実行される精密キャリブレーション
 * - 制御変数とモータ状態のリセット
 * - LED点滅による処理状態の表示
 * - LCD画面にキャリブレーション状態を表示
 * - N_CAL2回数（100回）の加速度X軸とジャイロZ軸データを同時取得
 * - 加速度X軸とジャイロZ軸のオフセット値を平均値として計算・保存
 * - 処理完了後、画面クリアとLED点灯
 */
void caliblation2(void)
{
    resetVar();                  // 制御変数をリセット
    resetMotor();                // モータを停止状態にリセット
    digitalWrite(LED, LOW);      // LED消灯（キャリブレーション開始表示）
    calDelay(80);                // センサー安定化待機
    M5.Lcd.setCursor(0, LCDV_MID);
    M5.Lcd.print(" Calibration 2 ");  // キャリブレーション状態表示
    acc_x_offset = 0.0;          // 加速度X軸オフセット初期化
    gyro_z_offset = 0.0;         // ジャイロZ軸オフセット初期化
    for (int i = 0; i < N_CAL2; i++) {
        readGyro();              // IMU生データ読み取り
        acc_x_offset += acc_x_data;    // X軸加速度データを累積
        gyro_z_offset += gyro_z_data;  // Z軸ジャイロデータを累積
        delay(9);                // 9ms待機
    }
    acc_x_offset /= (float)N_CAL2;   // 加速度X軸の平均値を計算
    gyro_z_offset /= (float)N_CAL2;  // ジャイロZ軸の平均値を計算
    M5.Lcd.fillScreen(BLACK);    // 画面クリア
    digitalWrite(LED, HIGH);     // LED点灯（キャリブレーション完了表示）
}

/**
 * @brief 電源ボタン状態チェックと機能実行
 * 
 * @param void
 * @return void
 * 
 * 処理概要：
 * - M5StickCの電源ボタン押下状態を取得
 * - 長押し（pbtn=2）: 第1回キャリブレーションを再実行
 * - 短押し（pbtn=1）: 動作モード切替（Stand ⇔ Demo）
 * - 押下なし: 何も実行しない
 */
void checkButtonP(void)
{
    byte pbtn = M5.Axp.GetBtnPress(); // 電源ボタンの押下状態を取得
    if (pbtn == 2) caliblation1();    // 長押し時：再キャリブレーション実行
    else if (pbtn == 1) setMode(true); // 短押し時：モード切替実行
}

/**
 * @brief キャリブレーション用遅延処理
 * 
 * @param int n 繰り返し回数
 * @return void
 * 
 * 処理概要：
 * - キャリブレーション時の安定した測定のための遅延処理
 * - 指定回数分、IMUデータ取得と9ms遅延を繰り返す
 * - センサーの安定化とノイズ除去を目的とした処理
 */
void calDelay(int n)
{
    for (int i = 0; i < n; i++) {
        getGyro();               // IMUデータ取得（センサー安定化）
        delay(9);                // 9ms待機
    }
}

/**
 * @brief 動作モード切替とLCD表示
 * 
 * @param bool inc モード切替フラグ（true:切替実行、false:現在モード表示のみ）
 * @return void
 * 
 * 処理概要：
 * - 倒立振子の動作モードを制御
 * - inc=true時: demo_modeを0→1→0...とトグル切替
 * - LCD画面をクリアし、現在のモードを表示
 *   - demo_mode=0: "Stand"（通常の倒立制御モード）
 *   - demo_mode=1: "Demo"（回転デモンストレーションモード）
 */

void setMode(bool inc)
{
    if (inc) demo_mode = ++demo_mode % 2;  // モード切替（0→1→0...のトグル）
    M5.Lcd.fillScreen(BLACK);      // 画面クリア
    M5.Lcd.setCursor(5, 5);        // カーソル位置設定
    if (demo_mode == 0) M5.Lcd.print("Stand ");      // 通常倒立モード表示
    else if (demo_mode == 1) M5.Lcd.print("Demo ");  // デモモード表示
}

/**
 * @brief デモモード開始時の動作設定
 * 
 * @param void
 * @return void
 * 
 * 処理概要：
 * - デモンストレーションモード専用の動作パラメータ設定
 * - move_rate=1.0: 前進動作を有効化
 * - spin_continuous=true: 連続回転モードを有効化
 * - spin_step=-40.0*clk: 反時計回りの回転速度設定
 * - 倒立しながら回転するデモ動作を実現
 */
void startDemo(void)
{
    move_rate = 1.0;             // 前進速度を1.0に設定
    spin_continuous = true;      // 連続回転モードを有効化
    spin_step = - 40.0 * clk;    // 反時計回り回転速度設定（-0.4度/制御周期）
}

/**
 * @brief 制御パラメータの初期設定
 * 
 * @param void
 * @return void
 * 
 * 処理概要：
 * - 倒立振子制御に必要な全パラメータを設定
 * - PID制御ゲイン: K_angle=37.0, K_omg=0.84, KI_angle=800.0
 * - ヨー制御ゲイン: K_yaw=4.0
 * - 移動制御ゲイン: K_dst=85.0, K_speed=2.7
 * - モータ出力補正: mach_fact_r/l=0.45（左右モータの特性差吸収）
 * - パンチ力設定: punch_power=20（静止摩擦克服用）
 * - 最大出力制限: max_power=120
 * - バランス調整: fb_balance=-3, motor_dependand=10
 */
void resetPara(void)
{
    K_angle = 37.0;              // 角度制御ゲイン（傾斜角に対する応答性）
    K_omg = 0.84;                // 角速度制御ゲイン（安定化効果）
    KI_angle = 800.0;            // 積分制御ゲイン（定常偏差除去）
    K_yaw = 4.0;                 // ヨー制御ゲイン（回転制御の応答性）
    K_dst = 85.0;                // 距離制御ゲイン（位置制御）
    K_speed = 2.7;               // 速度制御ゲイン（移動制御）
    mach_fact_r = 0.45;          // 右モータ出力補正係数
    mach_fact_l = 0.45;          // 左モータ出力補正係数
    punch_power = 20;            // パンチ力（静止摩擦克服用の初期出力）
    punch_dir = 1;               // パンチ力適用期間（制御周期数）
    fb_balance = -3;             // 前後バランス調整（機体重心補正）
    motor_dependand = 10;        // モータ最小動作出力（不感帯補償）
    max_power = 120;             // 最大出力制限値
    punch_power_2 = max(punch_power, motor_dependand);  // パンチ力の最大値
}

/**
 * @brief IMUデータ取得とオフセット補正・角度計算
 * 
 * @param void
 * @return void
 * 
 * 処理概要：
 * - 生のIMUデータを取得（readGyro()呼び出し）
 * - ジャイロY軸データからオフセット除去: var_omg = gyro_y_data - gyro_y_offset
 * - ヨー角積分計算: yaw_angle += (gyro_z_data - gyro_z_offset) * clk
 * - 相補フィルタによる傾斜角計算: 
 *   - ジャイロ積分 + 加速度センサー補正（cutoff=0.1でローパスフィルタ）
 *   - var_angle += (var_omg + ((acc_x_data - acc_x_offset) * 57.3 - var_angle) * cutoff) * clk
 */
void getGyro(void)
{
    readGyro();                  // 生IMUデータを取得
    var_omg = (gyro_y_data - gyro_y_offset);  // ジャイロY軸からオフセットを除去
    yaw_angle += (gyro_z_data - gyro_z_offset) * clk;  // ヨー角を積分計算
    // 相補フィルタによる傾斜角計算：ジャイロ積分＋加速度センサー補正
    var_angle += (var_omg + ((acc_x_data - acc_x_offset) * 57.3 - var_angle) * cutoff) * clk;
}

/**
 * @brief 生IMUデータ取得と軸変換
 * 
 * @param void
 * @return void
 * 
 * 処理概要：
 * - M5StickC内蔵IMU（MPU6886）から生データを取得
 * - ジャイロスコープデータ: getGyroData(&gx, &gy, &gz)
 * - 加速度センサーデータ: getAccelData(&ax, &ay, &az)
 * - M5StickCの取り付け方向に合わせた軸変換:
 *   - gyro_y_data = gx (ピッチ軸角速度)
 *   - gyro_z_data = gy (ヨー軸角速度)
 *   - acc_x_data = az (前後傾斜検出用)
 *   - acc_z_data = ay (起立判定用)
 */

void readGyro(void)
{
    float gx, gy, gz, ax, ay, az;
    M5.Imu.getGyroData(&gx, &gy, &gz);     // ジャイロスコープデータ取得
    M5.Imu.getAccelData(&ax, &ay, &az);    // 加速度センサーデータ取得
    // M5StickCの取り付け方向に合わせた軸変換
    gyro_y_data = gx;            // ピッチ軸角速度（前後傾斜）
    gyro_z_data = gy;            // ヨー軸角速度（左右回転）
    gyro_x_data = gz;            // ロール軸角速度（使用しない）
    acc_x_data = az;             // 前後方向加速度（傾斜角検出用）
    acc_z_data = ay;             // 上下方向加速度（起立判定用）
}

/**
 * @brief メイン制御アルゴリズム（PID制御+モータ駆動）
 * 
 * @param void
 * @return void
 * 
 * 処理概要：
 * - 倒立振子の核心となる制御処理を実行
 * - 回転制御: spin_target更新（連続回転またはターゲット追従）
 * - 移動制御: move_target更新（前後移動指令）
 * - 多重ループPID制御:
 *   - 角度制御: K_angle * var_angle
 *   - 角速度制御: K_omg * var_omg  
 *   - 積分制御: KI_angle * var_i_angle
 *   - 速度制御: K_speed * var_spd
 *   - 位置制御: K_dst * var_dst
 * - ヨー制御: yaw_power = (yaw_angle - spin_target) * K_yaw
 * - 左右モータ出力計算: power_r/l = power ± yaw_power
 * - モータ方向制御とパンチ力制御（静止摩擦克服）
 * - 過負荷保護機能（counter_over_power監視）
 */
void driver(void)
{
    #ifdef DEBUG
    debugDriver();
    #endif
    // 回転制御の補正係数を計算（移動時は回転速度を抑制）
    if (abs(move_rate) > 0.1) spin_fact = constrain(-(power_r + power_l) / 10.0, -1.0, 1.0);
    else spin_fact = 1.0;
    // 連続回転モードが有効な場合、spin_targetを継続的に更新
    if (spin_continuous == true) {
        spin_target += spin_step * spin_fact;
    } else {
        if (spin_target < spin_dest) spin_target += spin_step;
        if (spin_target > spin_dest) spin_target -= spin_step;
    }
    // 移動目標位置を更新（前後バランス調整を含む）
    move_target += move_step * (move_rate + (float)fb_balance / 100.0);
    var_spd += power * clk;      // 速度積分（位置制御用）
    var_dst += K_dst * (var_spd * clk - move_target);  // 距離偏差積分
    var_i_angle += KI_angle * var_angle * clk;         // 角度積分（定常偏差除去）
    // 多重ループPID制御：全制御項目の合成
    power = var_i_angle + var_dst + (K_speed * var_spd) + (K_angle * var_angle) + (K_omg * var_omg);
    // 過負荷防止: 出力が閾値を超えた場合の保護機能
    if (abs(power) > 1000.0) counter_over_power += 1;
    else counter_over_power = 0;
    // 過負荷カウンターが最大値を超えた場合、モータ駆動を停止
    if (counter_over_power > max_over_power) return;
    // 出力を最大許容値内に制限
    power = constrain(power, -max_power, max_power);
    yaw_power = (yaw_angle - spin_target) * K_yaw;
    power_r = power + yaw_power;
    power_l = power - yaw_power;
    // モータ制御用の負値定数を計算
    int16_t mdbn = - motor_dependand;
    int16_t pp2n =  - punch_power_2;
    // 左モータ制御
    ipower_l = (int16_t) constrain(power_l * mach_fact_l, -max_power, max_power);
    if (power_l > 0) {
        // 左モータが前進する場合
        if (motor_l_dir == 1) punch_control_l = constrain(++punch_control_l, 0, 100);
        else punch_control_l = 0; 
        motor_l_dir = 1;
        if (punch_control_l < punch_dir) driveMotorL(max(ipower_l, mdbn));
        else driveMotorL(max(ipower_l, motor_dependand));
    } else if (ipower_l < 0) {
        // 左モータが後進する場合
        if (motor_l_dir ==  -1) punch_control_l = constrain(++punch_control_l, 0, 100);
        else punch_control_l = 0;
        motor_l_dir = -1;
        if (punch_control_l < punch_dir) driveMotorL(min(ipower_l, pp2n));
        else driveMotorL(min(ipower_l, mdbn));
    } else {
        // 左モータが停止している場合
        driveMotorL(0);
        motor_l_dir = 0;
    }
    // 右モータ制御
    ipower_r = (int16_t) constrain(power_r * mach_fact_r, -max_power, max_power);
    if (power_r > 0) {
        // 右モータが前進する場合
        if (motor_r_dir == 1) punch_control_r = constrain(++punch_control_r, 0, 100);
        else punch_control_r = 0; 
        motor_r_dir = 1;
        if (punch_control_r < punch_dir) driveMotorR(max(ipower_r, mdbn));
        else driveMotorR(max(ipower_r, motor_dependand));
    } else if (ipower_r < 0) {
        // 右モータが後進する場合
        if (motor_r_dir == -1) punch_control_r = constrain(++punch_control_r, 0, 100);
        else punch_control_r = 0;
        motor_r_dir = -1;
        if (punch_control_r < punch_dir) driveMotorR(min(ipower_r, pp2n));
        else driveMotorR(min(ipower_r, mdbn));
    } else {
        // 右モータが停止している場合
        driveMotorR(0);
        motor_r_dir = 0;
    }
}

/**
 * @brief 左モータPWM出力制御
 * 
 * @param int16_t pwm PWM値（-127～127）
 * @return void
 * 
 * 処理概要：
 * - 左モータ専用の出力制御関数
 * - PWM値を安全範囲（MIN_PWM_VALUE～MAX_PWM_VALUE）に制限
 * - driveMotor(0, pwm)を呼び出してI2C経由でモータドライバーに送信
 * - チャネル0が左モータに対応
 */
void driveMotorL(int16_t pwm)
{
    driveMotor(0, (int8_t)constrain(pwm, MIN_PWM_VALUE, MAX_PWM_VALUE));
}

/**
 * @brief 右モータPWM出力制御
 * 
 * @param int16_t pwm PWM値（-127～127）
 * @return void
 * 
 * 処理概要：
 * - 右モータ専用の出力制御関数
 * - PWM値の符号を反転（-pwm）して左右モータの回転方向を統一
 * - PWM値を安全範囲（MIN_PWM_VALUE～MAX_PWM_VALUE）に制限
 * - driveMotor(1, -pwm)を呼び出してI2C経由でモータドライバーに送信
 * - チャネル1が右モータに対応
 */
void driveMotorR(int16_t pwm)
{
    driveMotor(1, (int8_t)constrain(-pwm, MIN_PWM_VALUE, MAX_PWM_VALUE));
}

/**
 * @brief I2C経由モータドライバー制御
 * 
 * @param byte ch モータチャネル（0:左、1:右）
 * @param int8_t speed PWM速度値（-127～127）
 * @return void
 * 
 * 処理概要：
 * - モータドライバー（I2Cアドレス0x38）への直接制御
 * - I2C通信プロトコル:
 *   1. beginTransmission(0x38)でアドレス0x38のデバイスとの通信開始
 *   2. write(ch)でモータチャネル指定
 *   3. write(speed)でPWM値送信
 *   4. endTransmission()で通信終了
 * - 正値: 前進、負値: 後進、0: 停止
 */
void driveMotor(byte ch, int8_t speed)
{
    Wire.beginTransmission(MOTOR_I2C_ADDRESS);
    Wire.write(ch);
    Wire.write(speed);
    Wire.endTransmission();
}

/**
 * @brief モータ緊急停止と状態リセット
 * 
 * @param void
 * @return void
 * 
 * 処理概要：
 * - 安全機能: 左右両モータを即座に停止
 * - driveMotorL(0), driveMotorR(0)でPWM値を0に設定
 * - 過負荷保護カウンターをリセット（counter_over_power = 0）
 * - 転倒検出時、システム異常時、初期化時に呼び出される
 * - モータの暴走を防ぐ重要な安全機能
 */
void resetMotor(void)
{
    driveMotorL(0);
    driveMotorR(0);
    counter_over_power = 0;
}

/**
 * @brief 制御変数の完全初期化
 * 
 * @param void
 * @return void
 * 
 * 処理概要：
 * - 倒立制御で使用する全ての動的変数を0.0または初期値にリセット
 * - 制御出力: power = 0.0
 * - 移動制御: move_target, move_rate = 0.0  
 * - 回転制御: spin_dest, spin_target, spin_step = 0.0, spin_continuous = false
 * - 角度・速度: yaw_angle, var_angle, var_omg = 0.0
 * - PID制御: var_spd, var_dst, var_i_angle = 0.0
 * - 転倒時、システムリセット時、キャリブレーション前に実行
 * - 制御の初期状態を保証する重要な処理
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
 * @brief シリアル通信デバッグ情報送信
 * 
 * @param void
 * @return void
 * 
 * 処理概要：
 * - 制御状態の監視・デバッグ用データをシリアル出力
 * - 出力データ:
 *   - 経過時間: millis() - time0
 *   - 倒立状態: standing（true/false）
 *   - 加速度X軸: acc_x_data（傾斜角検出用）
 *   - モータ出力: power（PID制御出力）
 *   - 傾斜角: var_angle（制御目標からの偏差）
 * - 100ループ毎（約1秒毎）に実行
 * - PCでのリアルタイム制御状態監視に使用
 */
void sendStatus(void)
{
    Serial.print(millis() - time0);              // 制御周期の実際の時間を出力
    Serial.print("  stand="); Serial.print(standing);     // 倒立状態フラグ
    Serial.print("  acc_x="); Serial.print(acc_x_data);   // 前後加速度（生値）
    Serial.print("  power="); Serial.print(power);        // 制御出力値
    Serial.print("  angle="); Serial.print(var_angle);    // 現在の傾斜角
    Serial.print(", ");
    Serial.print(millis() - time0);              // 制御周期時間（再度出力）
    Serial.println();                            // 改行
}

/**
 * @brief IMU初期化と測定レンジ設定
 * 
 * @param void
 * @return void
 * 
 * 処理概要：
 * - M5StickC内蔵IMUセンサー（MPU6886）の初期化
 * - M5.Imu.Init()でIMUハードウェア初期化
 * - MPU6886検出時の測定レンジ設定:
 *   - ジャイロスコープ: ±250dps（度/秒）
 *   - 加速度センサー: ±4G
 * - 倒立振子制御に最適な感度設定
 * - シリアル出力でセンサー検出状態を報告
 * - 初期化失敗時はエラーメッセージ出力
 */
void imuInit(void)
{
    M5.Imu.Init();               // IMUハードウェア初期化
    if (M5.Imu.imuType == M5.Imu.IMU_MPU6886) {
        // MPU6886センサー検出時の設定
        M5.Mpu6886.SetGyroFsr(M5.Mpu6886.GFS_250DPS);  // ジャイロ測定範囲：±250度/秒
        M5.Mpu6886.SetAccelFsr(M5.Mpu6886.AFS_4G);     // 加速度測定範囲：±4G
        if (serial_monitor == true) Serial.println("MPU6886 found");  // 検出成功をシリアル出力
    } else if (serial_monitor == true) {
        Serial.println("MPU6886 not found");    // 検出失敗をシリアル出力
    }
}

/**
 * @brief バッテリー電圧のLCD表示
 * 
 * @param void
 * @return void
 * 
 * 処理概要：
 * - M5StickCのバッテリー電圧を取得・表示
 * - M5.Axp.GetBatVoltage()で現在の電池電圧を取得
 * - LCD画面の指定位置(5, LCDV_MID)にカーソル移動
 * - 電圧値を小数点以下2桁の形式（例: "4.15v"）で表示
 * - 100ループ毎（約1秒毎）に更新
 * - バッテリー残量の目安として重要な情報
 * - 電圧低下時の動作異常を早期発見可能
 */
void displayBatteryVoltage(void)
{
    M5.Lcd.setCursor(5, LCDV_MID);   // カーソル位置を画面中央に設定
    v_batt = M5.Axp.GetBatVoltage(); // 現在のバッテリー電圧を取得
    M5.Lcd.printf("%4.2fv ", v_batt); // 電圧値を小数点以下2桁で表示
}
