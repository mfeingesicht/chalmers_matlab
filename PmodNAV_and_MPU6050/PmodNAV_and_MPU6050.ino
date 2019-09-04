/************************************************************************

  Test of Pmod NAV (Based on Jim Lindblom's program)

  AHRS filter based on Kris Winer's Arduino code

   date: June 18, 2019

*************************************************************************
  /*
  Description: Pmod_NAV
  All data (accelerometer, gyroscope, magnetometer) are displayed
  In the serial monitor

  Material
  1. Arduino Uno
  2. Pmod NAV (dowload library
  https://github.com/sparkfun/SparkFun_LSM9DS1_Arduino_Library)
  Licence Beerware

  Wiring
  Module <----------> Arduino
  J1 broche 6 3.3V
  J1 broche 5 GND
  J1 broche 4 A5
  J1 broche 2 A4
************************************************************************/

#include <Wire.h>
#include <SparkFunLSM9DS1.h>

// Define PmodNAV registers
#define LSM9DS1_M 0x1E
#define LSM9DS1_AG 0x6B

// Define the I2C address of the MPU6050
#define gyro_address_MPU6050 0x69

// Angle conversions
#define degConv 57.29577
#define radConv 0.01745

LSM9DS1 imu_PmodNAV; // creation of the imu object

// Redo a calibration
int recalibration = 0;


// Bike data
float a = 0.985;
float b = 1.095; // length between wheel centers [m]
float h = 0.43;


// Configuration of the module
#define PRINT_CALCULATED
#define PRINT_SPEED 250
static unsigned long lastPrint = 0;

#define AHRS true                      // set to false for basic data read
#define SerialDebug true               // set to true to print serial output for debugging
#define serial_display_update_rate 50  // time in ms between two display on the serial communication ; set to 0 to update at each loop

// The earth's magnetic field varies according to its location.
// Add or subtract a constant to get the right value
// of the magnetic field using the following site
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION 3.50 // declination (in degrees) for Göteborg.


// Define variables for calibration and offset (bias) or accelerometer, gyroscope and magnetometer
// PmodNAV
float accelCalibration_PmodNAV[3] = {0, 0, 0}; // Calibration for accelerometer in mg
float gyroCalibration_PmodNAV[3] = {0, 0, 0};  // Calibration for gyroscope in deg/s
float magCalibration_PmodNAV[3] = {0, 0, 0};   // Calibration for magnetometer in milligauss

float accelBias_PmodNAV[3] = {37.3825, -48.2377, 2.9257}; // Bias corrections for accelerometer in mg
float gyroBias_PmodNAV[3] = {4.8861, -1.0762, -3.4032};   // Bias corrections for gyroscope in deg/s
float magBias_PmodNAV[3] = {0, 0, 0};                     // Bias corrections for magnetometer in milligauss

// MPU6050
float gyroScale_MPU6050 = 131.0;
float accScale_MPU6050 = 16384.0;

float accelBias_MPU6050[3] = {0.0907, -0.0802, -0.0119};  // Bias corrections for accelerometer in mg
float gyroBias_MPU6050[3] = {-5.7309, 0.6936, -0.0618}; // Bias corrections for gyroscope in deg/s


// Global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError_PmodNAV = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
//float GyroMeasError_PmodNAV = PI * (0.5f / 180.0f);    // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift_PmodNAV = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError_PmodNAV;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift_PmodNAV;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 1.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.1f


// Complementary filter coefficient
float complementary_coef = 0.985;

// Define timing variables
float deltat = 0.0f;                      // integration interval for filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval


// Magnetometer measurement frequency
uint32_t MagRate = 10; // set magnetometer read rate in Hz; 10 to 100 (max) Hz are reasonable values


// Define variables contaning roll, pitch and yaw angles for each filter
float pitch_madgwick_PmodNAV = 0, yaw_madgwick_PmodNAV = 0, roll_madgwick_PmodNAV = 0; // variables to hold angles computed by the Madgwick filter
float pitch_mahony_PmodNAV = 0, yaw_mahony_PmodNAV = 0, roll_mahony_PmodNAV = 0;       // variables to hold angles computed by the Mahony filter
float pitch_comp_PmodNAV = 0, yaw_comp_PmodNAV = 0, roll_comp_PmodNAV = 0;             // variables to hold angles computed by the complementary filter without steering angle correction
float pitch_intGyro_PmodNAV = 0, yaw_intGyro_PmodNAV = 0, roll_intGyro_PmodNAV = 0;    // variables to hold angles computed by the angular rates integration


// Define variables to hold sensor data
float ax_PmodNAV, ay_PmodNAV, az_PmodNAV, gx_PmodNAV, gy_PmodNAV, gz_PmodNAV, mx_PmodNAV, my_PmodNAV, mz_PmodNAV;       // variables to hold latest sensor data values

long temperature_MPU6050;
long acc_x_MPU6050, acc_y_MPU6050, acc_z_MPU6050, gyro_pitch_MPU6050, gyro_roll_MPU6050, gyro_yaw_MPU6050;
long acc_total_vector_MPU6050;
float angle_roll_acc_MPU6050, angle_roll_MPU6050, ax_MPU6050, ay_MPU6050, az_MPU6050, ay_roll_compensated_MPU6050, gx_MPU6050, gy_MPU6050, gz_MPU6050;
float angle_roll_rad_MPU6050, gx_rad_MPU6050;
float angle_roll_comp_rad_MPU6050, gx_filtered_MPU6050, gx_filtered_new_MPU6050, dgx_MPU6050, angle_roll_acc_comp_MPU6050, angle_roll_comp_MPU6050;
float roll_gyroInt_MPU6050, roll_comp_MPU6050, roll_comp_latComp_MPU6050;
float gyro_angle_MPU6050 = 0.0;


// Define variables to hold intermediary variables for the filters
float q_PmodNAV[4] = {1.0f, 0.0f, 0.0f, 0.0f};          // vector to hold quaternion
float q_madgwick_PmodNAV[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // vector to hold quaternion for the madgwick filter
float q_mahony_PmodNAV[4] = {1.0f, 0.0f, 0.0f, 0.0f};   // vector to hold quaternion for the Mahony filter
float eInt_PmodNAV[3] = {0.0f, 0.0f, 0.0f};             // vector to hold integral error for Mahony method
float phi_acc_PmodNAV = 0;                              // variable to hold the lateral acceleration compensation for the complementary filter


// Define variable to hold received characters from serial communication
char inchar;


void setup()
{
  Serial.begin(115200);  // initialization of serial communication
  //Serial.println("ax,ay,az,gx,gy,gz,mx,my,mz,q_madgwick0,q_madgwickx,q_madgwicky,q_madgwickz,q_mahony0,q_mahonyx,q_mahonyy,q_mahonyz,roll_madgwick,pitch_madgwick,yaw_madgwick,roll_mahony,pitch_mahony,yaw_mahony,roll_comp,roll_intGyro,average_rate");

  pinMode(13, OUTPUT); // MPU6050 INT pin
  digitalWrite(13, HIGH);

  // initialization of the module
  imu_PmodNAV.settings.device.commInterface = IMU_MODE_I2C;
  imu_PmodNAV.settings.device.mAddress = LSM9DS1_M;
  imu_PmodNAV.settings.device.agAddress = LSM9DS1_AG;
  if (!imu_PmodNAV.begin())
  {
    Serial.println("Communication issue with LSM9DS1.");
    while (1);
  }

  
  // Recalibration
  if (recalibration)
  {
    for (int i=0;i<3;i++)
    {
      accelBias_PmodNAV[i] = 0; // Bias corrections for accelerometer in mg
      gyroBias_PmodNAV[i] = 0;  // Bias corrections for gyroscope in deg/s
      accelBias_MPU6050[i] = 0; // Bias corrections for accelerometer in mg
      gyroBias_MPU6050[i] = 0;  // Bias corrections for gyroscope in deg/s
    }
  }

  MPU6050_gyro_setup(); //Initiallize the gyro and set the correct registers.
}


void loop()
{
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Read PmodNAV data
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (imu_PmodNAV.gyroAvailable())
  {
    imu_PmodNAV.readGyro(); // acquisition des données du gyroscope
  }
  if (imu_PmodNAV.accelAvailable())
  {
    imu_PmodNAV.readAccel(); //Acquisition of accelerometer data
  }
  if (imu_PmodNAV.magAvailable())
  {
    imu_PmodNAV.readMag(); // Acquisition of the magnetometer
  }

  // Calculate the accelerometer values into g
  ax_PmodNAV = imu_PmodNAV.calcAccel(imu_PmodNAV.ax) - accelBias_PmodNAV[0]/1000.0;
  ay_PmodNAV = imu_PmodNAV.calcAccel(imu_PmodNAV.ay) - accelBias_PmodNAV[1]/1000.0;
  az_PmodNAV = imu_PmodNAV.calcAccel(imu_PmodNAV.az) - accelBias_PmodNAV[2]/1000.0;

  // Calculate the gyroscope values into °/s
  gx_PmodNAV = imu_PmodNAV.calcGyro(imu_PmodNAV.gx) - gyroBias_PmodNAV[0];
  gy_PmodNAV = imu_PmodNAV.calcGyro(imu_PmodNAV.gy) - gyroBias_PmodNAV[1];
  gz_PmodNAV = imu_PmodNAV.calcGyro(imu_PmodNAV.gz) - gyroBias_PmodNAV[2];

  // Calculate the magnetometer values into milligauss
  mx_PmodNAV = imu_PmodNAV.calcMag(imu_PmodNAV.mx)*1000 - magBias_PmodNAV[0];
  my_PmodNAV = imu_PmodNAV.calcMag(imu_PmodNAV.my)*1000 - magBias_PmodNAV[1];
  mz_PmodNAV = imu_PmodNAV.calcMag(imu_PmodNAV.mz)*1000 - magBias_PmodNAV[2];


  // Axis correction : see page 10 of https://www.st.com/resource/en/datasheet/lsm9ds1.pdf for axis
  
  // Accelerometer corrections
  // To respect the Right Hand Rule, the y-axis is in the wrong direction
  ay_PmodNAV = -ay_PmodNAV;
  
  // Gyroscope correction
  // To respect the Right Hand Rule, the y-axis is in the wrong direction
  gy_PmodNAV = -gy_PmodNAV;


  // Magnetometer corrections
  // Sensors x (y)-axis of the accelerometer is aligned with and opposite to the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  mx_PmodNAV = mx_PmodNAV + my_PmodNAV; my_PmodNAV = mx_PmodNAV - my_PmodNAV; mx_PmodNAV = mx_PmodNAV - my_PmodNAV; // Swap mx and my
  mx_PmodNAV = -mx_PmodNAV; // Change sign of my
  my_PmodNAV = -my_PmodNAV; // Change sign of mx
  mz_PmodNAV = -mz_PmodNAV; // Change sign of mz

  
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Read MPU6050 data
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  read_imu_data();
  
  // Normalize and scale the data
  ax_MPU6050 = acc_x_MPU6050 / accScale_MPU6050; ax_MPU6050 -= accelBias_MPU6050[0];
  ay_MPU6050 = acc_y_MPU6050 / accScale_MPU6050; ay_MPU6050 -= accelBias_MPU6050[1];
  az_MPU6050 = acc_z_MPU6050 / accScale_MPU6050; az_MPU6050 -= accelBias_MPU6050[2];
  gx_MPU6050 = gyro_roll_MPU6050 / gyroScale_MPU6050; gx_MPU6050 -= gyroBias_MPU6050[0];
  gy_MPU6050 = gyro_pitch_MPU6050 / gyroScale_MPU6050; gy_MPU6050 -= gyroBias_MPU6050[1];
  gz_MPU6050 = gyro_yaw_MPU6050 / gyroScale_MPU6050; gz_MPU6050 -= gyroBias_MPU6050[2];

  // Axis Correction
  ay_MPU6050 = -ay_MPU6050;
  az_MPU6050 = -az_MPU6050;
  gy_MPU6050 = -gy_MPU6050;
  gz_MPU6050 = -gz_MPU6050;


  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Update timer
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;


  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Compute angles for the PmodNAV
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Compute quarternions according to madgwick and Mahony filters
  // Pass acceleration as mg, gyro rate as rad/s and magnetometer as milligauss
  MadgwickQuaternionUpdate(ax_PmodNAV, ay_PmodNAV, az_PmodNAV, gx_PmodNAV*PI/180.0f, gy_PmodNAV*PI/180.0f, gz_PmodNAV*PI/180.0f, mx_PmodNAV, my_PmodNAV, mz_PmodNAV);
  MahonyQuaternionUpdate(ax_PmodNAV, ay_PmodNAV, az_PmodNAV, gx_PmodNAV*PI/180.0f, gy_PmodNAV*PI/180.0f, gz_PmodNAV*PI/180.0f, mx_PmodNAV, my_PmodNAV, mz_PmodNAV);


  // Complementary filter without steering angle correction
  if (roll_comp_PmodNAV == 0)
  {
    roll_comp_PmodNAV = atan2(ay_PmodNAV, az_PmodNAV);
  }
  phi_acc_PmodNAV = (180.0f/PI)*atan2(ay_PmodNAV, az_PmodNAV);
  roll_comp_PmodNAV = complementary_coef * (roll_comp_PmodNAV + gx_PmodNAV*deltat) + (1 - complementary_coef) * phi_acc_PmodNAV;
  
  
  // Angular rates integration
  if (roll_intGyro_PmodNAV == 0)
  {
    roll_intGyro_PmodNAV = atan2(ay_PmodNAV, az_PmodNAV);
  }
  roll_intGyro_PmodNAV += gx_PmodNAV*deltat;
          
  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth.
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
  yaw_madgwick_PmodNAV   = atan2(2.0f * (q_madgwick_PmodNAV[1] * q_madgwick_PmodNAV[2] + q_madgwick_PmodNAV[0] * q_madgwick_PmodNAV[3]), q_madgwick_PmodNAV[0] * q_madgwick_PmodNAV[0] + q_madgwick_PmodNAV[1] * q_madgwick_PmodNAV[1] - q_madgwick_PmodNAV[2] * q_madgwick_PmodNAV[2] - q_madgwick_PmodNAV[3] * q_madgwick_PmodNAV[3]);
  pitch_madgwick_PmodNAV = -asin(2.0f * (q_madgwick_PmodNAV[1] * q_madgwick_PmodNAV[3] - q_madgwick_PmodNAV[0] * q_madgwick_PmodNAV[2]));
  roll_madgwick_PmodNAV  = atan2(2.0f * (q_madgwick_PmodNAV[0] * q_madgwick_PmodNAV[1] + q_madgwick_PmodNAV[2] * q_madgwick_PmodNAV[3]), q_madgwick_PmodNAV[0] * q_madgwick_PmodNAV[0] - q_madgwick_PmodNAV[1] * q_madgwick_PmodNAV[1] - q_madgwick_PmodNAV[2] * q_madgwick_PmodNAV[2] + q_madgwick_PmodNAV[3] * q_madgwick_PmodNAV[3]);
  pitch_madgwick_PmodNAV *= 180.0f / PI;
  yaw_madgwick_PmodNAV   *= 180.0f / PI;
  yaw_madgwick_PmodNAV   -= DECLINATION; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
  roll_madgwick_PmodNAV  *= 180.0f / PI;
  
  yaw_mahony_PmodNAV   = atan2(2.0f * (q_mahony_PmodNAV[1] * q_mahony_PmodNAV[2] + q_mahony_PmodNAV[0] * q_mahony_PmodNAV[3]), q_mahony_PmodNAV[0] * q_mahony_PmodNAV[0] + q_mahony_PmodNAV[1] * q_mahony_PmodNAV[1] - q_mahony_PmodNAV[2] * q_mahony_PmodNAV[2] - q_mahony_PmodNAV[3] * q_mahony_PmodNAV[3]);
  pitch_mahony_PmodNAV = -asin(2.0f * (q_mahony_PmodNAV[1] * q_mahony_PmodNAV[3] - q_mahony_PmodNAV[0] * q_mahony_PmodNAV[2]));
  roll_mahony_PmodNAV  = atan2(2.0f * (q_mahony_PmodNAV[0] * q_mahony_PmodNAV[1] + q_mahony_PmodNAV[2] * q_mahony_PmodNAV[3]), q_mahony_PmodNAV[0] * q_mahony_PmodNAV[0] - q_mahony_PmodNAV[1] * q_mahony_PmodNAV[1] - q_mahony_PmodNAV[2] * q_mahony_PmodNAV[2] + q_mahony_PmodNAV[3] * q_mahony_PmodNAV[3]);
  pitch_mahony_PmodNAV *= 180.0f / PI;
  yaw_mahony_PmodNAV   *= 180.0f / PI;
  yaw_mahony_PmodNAV   -= DECLINATION; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
  roll_mahony_PmodNAV  *= 180.0f / PI;
  
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Compute angles for the MPU6050
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Compute the roll angle
  angle_roll_acc_MPU6050 = degConv * atan2(ay_MPU6050, az_MPU6050);
  angle_roll_MPU6050 = a * (angle_roll_MPU6050 + gx_MPU6050 * deltat)  + (1 - a) * angle_roll_acc_MPU6050;  // Correct the drift of the gyro roll angle with the accelerometer roll angle.
  gyro_angle_MPU6050 = gyro_angle_MPU6050 + gx_MPU6050 * deltat;


  // Lateral Acceleration Compensation 
  gx_rad_MPU6050 = gx_MPU6050 * radConv;
  gx_filtered_new_MPU6050 = b * gx_filtered_new_MPU6050 + (1 - b) * gx_rad_MPU6050;
  dgx_MPU6050 = (gx_filtered_new_MPU6050 - gx_filtered_MPU6050) / deltat;
  ay_roll_compensated_MPU6050 = ay_MPU6050 + (dgx_MPU6050 * h) / 9.81;
  angle_roll_acc_comp_MPU6050 = degConv * atan2(ay_roll_compensated_MPU6050, az_MPU6050);
  angle_roll_comp_MPU6050 = a * (angle_roll_comp_MPU6050 + gx_MPU6050 * deltat)  + (1 - a) * angle_roll_acc_comp_MPU6050;
  gx_filtered_MPU6050 = gx_filtered_new_MPU6050;


  // Convert the angles to radians
  angle_roll_rad_MPU6050 = angle_roll_MPU6050 * radConv;
  angle_roll_comp_rad_MPU6050 = angle_roll_comp_MPU6050 * radConv;


  roll_gyroInt_MPU6050 = gyro_angle_MPU6050;
  roll_comp_MPU6050 = angle_roll_MPU6050;
  roll_comp_latComp_MPU6050 = angle_roll_comp_MPU6050;
  

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Send data through serial communication in CSV format
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  inchar=Serial.read();
  if(inchar=='1')
  {
    Serial.print("$");
    
    // PmodNAV
    Serial.print((int)1000*ax_PmodNAV,4); Serial.print(",");
    Serial.print((int)1000*ay_PmodNAV,4); Serial.print(",");
    Serial.print((int)1000*az_PmodNAV,4); Serial.print(",");
    Serial.print(gx_PmodNAV,4); Serial.print(",");
    Serial.print(gy_PmodNAV,4); Serial.print(",");
    Serial.print(gz_PmodNAV,4); Serial.print(",");
    Serial.print((int)mx_PmodNAV,4); Serial.print(",");
    Serial.print((int)my_PmodNAV,4); Serial.print(",");
    Serial.print((int)mz_PmodNAV,4); Serial.print(",");
    
    Serial.print(q_madgwick_PmodNAV[0],4); Serial.print(",");
    Serial.print(q_madgwick_PmodNAV[1],4); Serial.print(",");
    Serial.print(q_madgwick_PmodNAV[2],4); Serial.print(",");
    Serial.print(q_madgwick_PmodNAV[3],4); Serial.print(",");
    
    Serial.print(q_mahony_PmodNAV[0],4); Serial.print(",");
    Serial.print(q_mahony_PmodNAV[1],4); Serial.print(",");
    Serial.print(q_mahony_PmodNAV[2],4); Serial.print(",");
    Serial.print(q_mahony_PmodNAV[3],4); Serial.print(",");
    
    Serial.print(roll_madgwick_PmodNAV);Serial.print(",");
    Serial.print(pitch_madgwick_PmodNAV);Serial.print(",");
    Serial.print(yaw_madgwick_PmodNAV);Serial.print(",");
    
    Serial.print(roll_mahony_PmodNAV);Serial.print(",");
    Serial.print(pitch_mahony_PmodNAV);Serial.print(",");
    Serial.print(yaw_mahony_PmodNAV);Serial.print(",");
    
    Serial.print(roll_comp_PmodNAV);Serial.print(",");
    
    Serial.print(roll_intGyro_PmodNAV);Serial.print(",");
    
    Serial.print(1.0f / deltat, 2);Serial.print(",");

    // MPU6050
    Serial.print(ax_MPU6050,4);Serial.print(",");
    Serial.print(ay_MPU6050,4);Serial.print(",");
    Serial.print(az_MPU6050,4);Serial.print(",");
    Serial.print(gx_MPU6050,4);Serial.print(",");
    Serial.print(gy_MPU6050,4);Serial.print(",");
    Serial.print(gz_MPU6050,4);Serial.print(",");
    
    Serial.print(roll_gyroInt_MPU6050,4);Serial.print(",");
    Serial.print(roll_comp_MPU6050,4);Serial.print(",");
    Serial.print(roll_comp_latComp_MPU6050,4);
    
    Serial.println();
  }
}
