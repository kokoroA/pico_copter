#include "test.hpp"
// #define I2C_TIME_OUT_BASE   10
// #define I2C_TIME_OUT_BYTE   1
// #define I2C_PORT i2c1
// #define I2C_ADDRESS 0x0001
// #define i2C_CLOCK (400*1000)
// #define NORMAL_WAIT 500
// #define SDA_PIN 26  // GP27 = Pin.31 = SDA
// #define SCL_PIN 27  // GP26 = Pin.32 = SCL

//Get Altitude
int8_t Status = 0;
int16_t OffsetValue = 0;
uint16_t dev = 0x29;
uint16_t distance = 0;
uint8_t isDataReady = 0;
uint8_t rangeStatus = 0;
uint8_t state = 0;
uint8_t tmp = 0;
int sleep_time = 2000;
int ms = 200;
uint64_t start_time,current_time;

//KalmanFilter

//位置の誤差のばらつき（分散）
MatrixXf Sigma_yn_pre(2,2);
Sigma_yn_pre << 1,0,
                0,1;
Eigen::MatrixXf Sigma_yn_est(2,2);
Sigma_yn_est << 1,0,
                0,1;
Eigen::MatrixXd last_Sigma_yn_pre = Eigen::MatrixXd::Zero(2,1);
//速度、位置
Eigen::MatrixXf mu_Yn_pre(2,1);
mu_Yn_pre << 0,
             1;
Eigen::MatrixXf mu_Yn_est(2,1);
mu_Yn_est << 0,
             1;
Eigen::MatrixXd last_mu_Yn_pre = Eigen::MatrixXd::Zero(2,1);
uint8_t vo = 0;
uint8_t yo = 0;
//システムノイズの分散(予想)
uint8_t stdv_Q = 0;
uint8_t Q = std::pow(stdv_Q,2.0);
//観測した値ののばらつき（分散）
float stdv_R = 0.02;
float R = std::pow(stdv_R,2.0);
//時間
uint8_t t =  10;
uint8_t t_s = 0;
uint8_t tc = 0;
//刻み幅
float h = 0.01;
float h_kalman = 0.02;
//要素数
uint16_t n = static_cast<float>(t)/h;
uint16_t n_kalman = static_cast<float>(t)/h_kalman;

//PID

//誤差
uint8_t error = 0;
//目標値
uint8_t r = 0;
//前回の誤差
uint8_t last_error = 0
//制御周期
float Control_T = 0.02;
//Pゲイン
uint8_t Kp = 20;
//Iゲイン
uint8_t Ki = 5;
//Dゲイン
uint8_t Kd = 1;
//誤差の微分の近似計算
float de = 0;
//誤差の積分の近似計算
float ie = 0;
//システムノイズのばらつき
float stdv_y = 0.0009;

//運動方程式

//重量
uint8_t m = 0.8;
//重力
float g = 9.81;
//プロペラ定数
uint8_t Ct = 4e-5;

//観測行列
Eigen::MatrixXf observation_mat(1,2);
observation_mat << 0,1;
observation_mat_transposed = observation_mat.transpose();
//システム行列
Eigen::MatrixXf system_mat(2,2);
system_mat << 1,0,
              (1-vo)*h_kalman,1;
//制御入力
uint8_t = 0;
//制御行列
Eigen::MatrixXf control_mat(2,1);
control_mat << Ct*h_kalman/m,
                0;
//単位行列
Eigen::MatrixXf unit_mat(2,2);
unit_mat << 1,0,
            0,1;
//カルマンゲイン
Eigen::MatrixXf K = Sigma_yn_pre * observation_mat_transposed * 1 / (observation_mat * Sigma_yn_pre * observation_mat_transposed + R)


int main (void) {
    // uint8_t byte_data_read[16]; //readした値の格納用
    // uint8_t byte_data_write[16];//書き込む値の格納用
    // uint8_t _I2CBuffer[256];
    // byte_data_write[0] = 2; 
    // byte_data_write[1] = 1;
    // uint16_t index = 0x00E5;
    // uint16_t test =  0x010F;
    // _I2CBuffer[0] = index>>8;
	// _I2CBuffer[1] = index&0xFF;
    // uint8_t byteREAD[16];

    stdio_init_all();
    i2c_init(I2C_PORT,i2C_CLOCK);//ハードウェアの初期化
    gpio_set_function(SDA_PIN,GPIO_FUNC_I2C);//GPIO機能をi2cに選択(SDA)
    gpio_set_function(SCL_PIN,GPIO_FUNC_I2C);//GPIO機能をi2cに選択(SCL)
    gpio_set_pulls(SDA_PIN, true, false);// enable internal pull-up of SDA_PIN=GP26
    gpio_set_pulls(SCL_PIN, true, false);// enable internal pull-up of SCL_PIN=GP27
    // /* Platform Initialization code here*/
    // /* Wait for device booted*/

    sleep_ms(3000);

    // Status = i2c_write_timeout_us(I2C_PORT,dev,byte_data_write,2,false,600); 
    // Status = i2c_read_timeout_us(I2C_PORT,dev,byte_data_read,1,false,600);

    while((state&1) == 0 ) {
        Status = VL53L1X_BootState(dev, &state);
        sleep_ms(ms);
    };

    // /* Sensor Initialization */
    Status = VL53L1X_SensorInit(dev);

    // /* Modify the default configuration */
    // // Status = VL53L1X_SetInterMeasurementPeriod();
    // Status = VL53L1X_SetOffset(dev,OffsetValue);
    Status = VL53L1X_SetDistanceMode(dev,1);
    Status = VL53L1X_SetTimingBudgetInMs(dev,15);
    Status = VL53L1X_SetInterMeasurementInMs(dev,500);

    //Enable the ranging
    Status = VL53L1X_StartRanging(dev);

    // /* ranging loop */
    start_time = time_us_64();
    current_time = start_time;
    while(1){
        while(isDataReady==0){
            Status = VL53L1X_CheckForDataReady(dev, &isDataReady);
        }
        isDataReady =0;
        Status = VL53L1X_GetRangeStatus(dev,&rangeStatus);
        Status = VL53L1X_GetDistance(dev,&distance);
        Status = VL53L1X_ClearInterrupt(dev);
        // printf(" Status(ranging loop) : %d\n ",Status);
        printf("%9.6f %4d\n",(current_time-start_time)/1000000.0,distance);

        // //カルマンフィルタ＋PID制御
        // last_mu_Yn_pre = mu_Yn_est;
        // last_Sigma_Yn_pre = Sigma_yn_est;
        // last_error = error;

        // //カルマンフィルタ
        //mu_Yn_pre = 


        current_time = time_us_64();
        // printf("distance : %d\n",distance);

    }
}