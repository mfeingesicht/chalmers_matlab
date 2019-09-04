///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This part reads the raw gyro and accelerometer data from the MPU-6050
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void read_imu_data(void) {
  Wire.beginTransmission(gyro_address_MPU6050);                       //Start communication with the gyro.
  Wire.write(0x3B);                                           //Send the address of the register from which we will read data (3B for a MPU-6050)
  
  // For a MPU-6050, the data is organized as follows (register address on top, data on bottom), totalling 14 bytes to read:
  //       3B                 3C                3D                 3E                3F                 40               41               42              43                44                45                46                47                48
  // ACCEL_XOUT[15:8] | ACCEL_XOUT[7:0] | ACCEL_YOUT[15:8] | ACCEL_YOUT[7:0] | ACCEL_ZOUT[15:8] | ACCEL_ZOUT[7:0] | TEMP_OUT[15:8] | TEMP_OUT[7:0] | GYRO_XOUT[15:8] | GYRO_XOUT[7:0] | GYRO_YOUT[15:8] | GYRO_YOUT[7:0] | GYRO_ZOUT[15:8] | GYRO_ZOUT[7:0]
  
  if (Wire.endTransmission()){                                //End the transmission with the gyro.
    return;                                                   //Exit function on error.
  }else{
    if (Wire.requestFrom(gyro_address_MPU6050, 14)<14){               //Request 14 bytes from the MPU 6050.
      return;                                                 //Exit function on error.
    }else{
      acc_x_MPU6050 = (Wire.read() << 8 | Wire.read());              //Add the low and high byte to the acc_x variable.
      acc_y_MPU6050 = (Wire.read() << 8 | Wire.read());              //Add the low and high byte to the acc_y variable.
      acc_z_MPU6050 = (Wire.read() << 8 | Wire.read());               //Add the low and high byte to the acc_z variable.
      temperature_MPU6050 = Wire.read() << 8 | Wire.read();           //Add the low and high byte to the temperature variable.
      gyro_roll_MPU6050 = Wire.read() << 8 | Wire.read();             //Read high and low part of the angular data.
      gyro_pitch_MPU6050 = Wire.read() << 8 | Wire.read();            //Read high and low part of the angular data.
      gyro_yaw_MPU6050 = Wire.read() << 8 | Wire.read();              //Read high and low part of the angular data.
    }
  }
}
