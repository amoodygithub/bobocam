/* 
 *  
 *  JAN 4 
 *  The reciever and transmitter are working together
 *  transmitter is sending gyro data, though i dont know 
 *  if the gyro data means anything right now.. 
 *  
 *  TODO : 
 *  
 *  - turn gyro data into anglular displacement 
 *  
 *  
 *  
 */



#include <SPI.h>
#include <Wire.h>
#include <RF24.h>

RF24 radio(7,8); // cns,ce

//Declaring some global variables
int gx, gy, gz;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gx_cal, gy_cal, gz_cal;
long loop_timer;
int lcd_loop_counter;
float angle_x, angle_y;
int angle_x_buffer, angle_y_buffer;
boolean set_gyro_angles;
float angle_y_acc, angle_x_acc;
float angle_x_output, angle_y_output;
const int MPU_address = 0x68; // 69 if A0 is high
float AccX,AccY,AccZ;
float Temp;
float accAngleX, accAngleY, GyroAngleX, gyroAngleY, gyroAngleZ;
float y, x, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapstedTime, currentTime, previousTime;
float angle_in_degrees_gx;
float angle_in_degrees_gy;
float angle_in_degrees_gz;


int c = 0;

const byte addresses[][6] = {"0"};
typedef struct
{
  float GyroX;
  float GyroY;
  float GyroZ;
} data;

data payload;


void setup() {
  
  Serial.begin(9600);
  Serial.println("Running Setup");
  Serial.println("Calibrating Gyroscope ...");
  
  setup_mpu_6050_registers();
  delay(1000);
  gyroCalibration();
  
  setupRadioTransmitting(); // sets up RF for transmission

}

void setupRadioTransmitting(){
  radio.begin();
  radio.setChannel(115);
  radio.openWritingPipe(addresses[0]);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate( RF24_250KBPS );
  radio.stopListening(); // sets to transmitter

  
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

void printData(){
  Serial.print("gx:");
  Serial.println(gx);
  Serial.print("gy:");
  Serial.println(gy);
  Serial.print("gz:");
  Serial.println(gz);
  
}

void printPackageData(){
  Serial.print("\nPackage:");
  Serial.println(payload.GyroX);
  Serial.println(payload.GyroY);
  Serial.println(payload.GyroZ);
}


void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
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
  Serial.println(gx_cal);
  Serial.println(gy_cal);
  Serial.println(gz_cal);
  delay(2000);
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

void processData(){
  gx-=gx_cal;
  gy-=gy_cal;
  gz-=gz_cal;

  // Angle calculations
  // 0.0000611=(1/(250Hz/65.5))
  angle_x += gx*0.0000611;
  angle_y += gy*0.0000611;
  
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_x += angle_y * sin(gz * 0.000001066);   //If the IMU has yawed transfer the y angle to the x angel
  angle_y -= angle_x * sin(gz * 0.000001066);   //If the IMU has yawed transfer the x angle to the y angel
  
}

void assignDataToPayload(){
  payload.GyroX = angle_x;
  payload.GyroY = angle_y;
  payload.GyroZ = angle_z;
}

void loop() {
  
  Serial.println("Transmitting");
  readData();
  processData();
  assignDataToPayload();
  printPackageData();
  radio.write(&payload,sizeof(payload));

  
 
  
  delay(500);
}
