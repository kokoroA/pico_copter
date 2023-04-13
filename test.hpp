// #include "platform/vl53l1_platform.h"
#include "vl53l1_platform.h"
#include "VL53L1X_api.h"
#include <Eignen/Dense>

using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::Matrix;
using namespace Eigen;

typedef unsigned int uint;
// class Altitude
// {
//     //variable
//     private:

//     // int8_t Status = 0;
//     // int16_t OffsetValue =0;
//     // uint16_t dev = 0x52;
//     // uint16_t distance;
//     // uint8_t isDataReady = 0;
//     // uint8_t rangeStatus;
//     // uint8_t state = 1;

//     // public:
//     // int main();
// };