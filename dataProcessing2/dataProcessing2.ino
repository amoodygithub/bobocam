#include <Wire.h>

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;
long ax_cal, ay_cal, az_cal;

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;
long gx_cal, gy_cal, gz_cal;

int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
int lcd_loop_counter;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;

float gx,gy,gz;



void setup() {
  Serial.begin(9600);
  Wire.begin();
  setupMPU();
  gyroCalibration();
  accelCalibration();
}


void loop() {
  recordAccelRegisters();
  recordGyroRegisters();
  calculateAngularDisplacement();
  printData();
  
}

void setupMPU(){
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00001000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processAccelData();
}

void processAccelData(){
  gForceX = (accelX - ax_cal)/ 8192.0;
  gForceY = (accelY - ay_cal)/ 8192.0; 
  gForceZ = (accelZ - az_cal)/ 8192.0;
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processGyroData();
}

void gyroCalibration(){
  int accuracyGyro = 2000; // the higher the more accurate
  for (int cal_int = 0; cal_int < accuracyGyro ; cal_int ++){                  //Run this code 2000 times
    if(cal_int % 125 == 0){ 
      Serial.println("Calibrating...");                                //Print "calibrating every couple secs"
      }
    recordGyroRegisters();                                                   //Read the raw acc and gyro data from the MPU-6050
    gx_cal += gyroX;                                              //Add the gyro x-axis offset to the gx_cal variable
    gy_cal += gyroY;                                              //Add the gyro y-axis offset to the gy_cal variable
    gz_cal += gyroZ;                                              //Add the gyro z-axis offset to the gz_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  gx_cal /= accuracyGyro;                                                  //Divide the gx_cal variable by 2000 to get the avarage offset
  gy_cal /= accuracyGyro;                                                  //Divide the gy_cal variable by 2000 to get the avarage offset
  gz_cal /= accuracyGyro;  
  Serial.println("OFFSET VALUES");
  Serial.println(gx_cal);
  Serial.println(gy_cal);
  Serial.println(gz_cal);
  delay(1000);
}

void accelCalibration(){ // 8192.0 per G accelleration 
  int accuracyAccel = 1000;
  for (int cal_int = 0; cal_int < accuracyAccel; cal_int ++){                  //Run this code 2000 times
    if(cal_int % 125 == 0){ 
      Serial.println("Calibrating...");                                //Print "calibrating every couple secs"
      }
    recordAccelRegisters();                                                   //Read the raw acc and gyro data from the MPU-6050
    ax_cal += accelX;                                              //Add the gyro x-axis offset to the gx_cal variable
    ay_cal += accelY;                                              //Add the gyro y-axis offset to the gy_cal variable
    az_cal += (8192- accelZ);                                              //Add the gyro z-axis offset to the gz_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  ax_cal /= accuracyAccel;                                                  //Divide the gx_cal variable by 2000 to get the avarage offset
  ay_cal /= accuracyAccel;                                                  //Divide the gy_cal variable by 2000 to get the avarage offset
  az_cal /= accuracyAccel;  
  Serial.println("OFFSET VALUES");
  Serial.println(ax_cal);
  Serial.println(ay_cal);
  Serial.println(az_cal);
  delay(1000);
}

void processGyroData() {
  rotX = (gyroX - gx_cal)/ 65.5;
  rotY = (gyroY - gy_cal)/ 65.5; 
  rotZ = (gyroZ - gz_cal)/ 65.5;
}

void printData() {
//  Serial.print("Gyro (deg)");
//  Serial.print(" X=");
//  Serial.print(rotX);
//  Serial.print(" Y=");
//  Serial.print(rotY);
//  Serial.print(" Z=");
//  Serial.print(rotZ);
//  Serial.print(" Accel (g)");
//  Serial.print(" X=");
//  Serial.print(gForceX);
//  Serial.print(" Y=");
//  Serial.print(gForceY);
//  Serial.print(" Z=");
//  Serial.print(gForceZ);
  Serial.print(" angle_pitch=");
  Serial.print(angle_pitch);
  Serial.print(" angle_roll=");
  Serial.print(angle_roll);
  Serial.print(" angle_pitch_output=");
  Serial.print(angle_pitch_output);
  Serial.print(" angle_roll=");
  Serial.println(angle_roll_output);
}

void calculateAngularDisplacement(){
  
  gx = gyroX - gx_cal;                                                //Subtract the offset calibration value from the raw gx value
  gy = gyroY - gy_cal;                                                //Subtract the offset calibration value from the raw gy value
  gz = gyroZ - gz_cal;                                                //Subtract the offset calibration value from the raw gz value
  


  
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gx * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gy * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gz * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gz * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel

  acc_x = (accelX - ax_cal);
  acc_y = (accelY - ay_cal);
  acc_z = (accelZ - az_cal);
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  if(set_gyro_angles){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value

  angle_pitch_buffer = angle_pitch_output*10;
  angle_roll_buffer = angle_roll_output*10;
  
  
  while(micros() - loop_timer < 4000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();              

  
}
