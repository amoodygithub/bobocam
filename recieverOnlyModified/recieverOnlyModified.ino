
/* TODO
 *  - recieve gyroscope data from other sensor (DONE)
 *  
 *  - turn gyro data into anglular displacement 
 *  
 *  - make calibration code that activates at push of 
 *  button and when buttonpush is recieved over radio
 *  
 *  - implement dc motor control to make arduino sensors face same direction
 *  
 *  
 */

 /* OFFSETS
  //ax 5299
  //ay 5258
  //az 8617
  //gx -569
  //gy -107
  //gz -1
  */

 /* PINOUT
  *  MP6050:
  *  SCL -> SCL
  *  SDA -> SDA
  *  
  *  nRF24L01:
  *  CSN -> 8
  *  CE -> 7
  *  MOSI -> 11
  *  MISO -> 12
  *  SCK -> 13
  *  IRQ nothing
  */


  /*
   * configure sensitiveity (default +/- 2g)
   * Wire.beginTransmission(MPU);
   * Wire.write(0x1C);
   * Wire.write(0x10); //set the register bits as 00010000 +/- 8g
   * Wire.endTransmission(true);
   * Gyro config is 0x1B
   * wire.write( 0x10 ) is 1000deg/s
   * just use the datasheet
   */
//  calculate_IMU_error();



#include <SPI.h>
#include <RF24.h>
//#include <SD.h>
#include<TMRpcm.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"


// Declaring global variables
const int MPU_address = 0x68;//MPU6050 I2C address
float AccX,AccY,AccZ;
float Temp;
float gx,gy,gz;
float accAngleX, accAngleY, GyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapstedTime, currentTime, previousTime;
int c = 0;



//Declaring some global variables
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gx_cal, gy_cal, gz_cal;
long loop_timer;
int lcd_loop_counter;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;

TMRpcm tmrpcm;


RF24 radio(7,8); // ce,csn

typedef struct
{
  float GyroX;
  float GyroY;
  float GyroZ;
} data;

data payload;

const byte addresses[][6] = {"0","1"};


int iteration = 1;

void setup() {
  Serial.begin(9600);
  Serial.println("Running Setup");
  Serial.println("Calibrating Gyroscope, please hold sensor horizontal and still ...");
  
  setup_mpu_6050_registers();
  delay(1000);
  gyroCalibration();
   
  delay(20);
  setupRadioRecieving();
  
  Serial.println("endSetup");

}
void setupRadioRecieving(){
  radio.begin();
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate( RF24_250KBPS );
  radio.setChannel(115);
  radio.openReadingPipe(1, addresses[0]);
  radio.startListening();
}
void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register (this one makes sure the sensor is not in sleep mode) 
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-4g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0b00001000);                                              //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}

void gyroCalibration(){
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Run this code 2000 times
    if(cal_int % 125 == 0){ 
      Serial.println("Calibrating...");                                //Print "calibrating every couple secs"
      }
    readDataForCalibration();                                                       //Read the raw acc and gyro data from the MPU-6050
    gx_cal += gx;                                              //Add the gyro x-axis offset to the gx_cal variable
    gy_cal += gy;                                              //Add the gyro y-axis offset to the gy_cal variable
    gz_cal += gz;                                              //Add the gyro z-axis offset to the gz_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  gx_cal /= 2000;                                                  //Divide the gx_cal variable by 2000 to get the avarage offset
  gy_cal /= 2000;                                                  //Divide the gy_cal variable by 2000 to get the avarage offset
  gz_cal /= 2000;  
  Serial.println("OFFSET VALUES");
  Serial.println(gx_cal);
  Serial.println(gy_cal);
  Serial.println(gz_cal);
  delay(2000);
}

void readData(){
  Wire.beginTransmission(MPU_address);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_address,14,true);
  AccX=Wire.read()<<8|Wire.read();
  AccY=Wire.read()<<8|Wire.read();
  AccZ=Wire.read()<<8|Wire.read();
  Temp=Wire.read()<<8|Wire.read();
  gx=Wire.read()<<8|Wire.read();
  gy=Wire.read()<<8|Wire.read();
  gz=Wire.read()<<8|Wire.read();
//  Serial.print(" || AccX = "); Serial.print(AccX);
//  Serial.print(" || AccY = "); Serial.print(AccY);
//  Serial.print(" || AccZ = "); Serial.print(AccZ);
//  Serial.print(" || Temp = "); Serial.print(Temp/340.00+36.53);
  Serial.print(" || gx = "); Serial.print(gx);
  Serial.print(" || gy = "); Serial.print(gy);
  Serial.print(" || gz = "); Serial.println(gz);
  delay(100);
}

void readDataForCalibration(){
  Wire.beginTransmission(MPU_address);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_address,14,true);
  AccX=Wire.read()<<8|Wire.read();
  AccY=Wire.read()<<8|Wire.read();
  AccZ=Wire.read()<<8|Wire.read();
  Temp=Wire.read()<<8|Wire.read();
  gx=Wire.read()<<8|Wire.read();
  gy=Wire.read()<<8|Wire.read();
  gz=Wire.read()<<8|Wire.read();
}

void printData(){
  Serial.print("gx:");
  Serial.println(gx);
  Serial.print("gy:");
  Serial.println(gy);
  Serial.print("gz:");
  Serial.println(gz);
  
}

void printPayload(){
  Serial.print("Payload.GyroX: ");
  Serial.println(payload.GyroX);
  Serial.print("Payload.GyroY: ");
  Serial.println(payload.GyroY);
  Serial.print("Payload.GyroZ: ");
  Serial.println(payload.GyroZ);
}



// ********************************************************************************
void loop() {
  readData();

  gx -= gx_cal;                                                //Subtract the offset calibration value from the raw gx value
  gy -= gy_cal;                                                //Subtract the offset calibration value from the raw gy value
  gz -= gz_cal;                                                //Subtract the offset calibration value from the raw gz value
  
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gx * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gy * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gz * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gz * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel
  
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
  
  while(micros() - loop_timer < 4000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();              

  printData();
  if(radio.available())
  {
    radio.read(&payload,sizeof(payload));
    Serial.println("Recieving data!");
  }
  printPayload();


} 
