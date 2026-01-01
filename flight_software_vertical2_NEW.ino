#include "Wire.h"       // allows communication to i2c devices connected to arduino
#include "I2Cdev.h"     // I2Connection library (communication to serial port)
#include "MPU6050.h"    // IMU library
#include "Servo.h"      // servo control library
#include "MPU6050.h"
#include <Wire.h>
#include <Kalman.h>
MPU6050 mpu; //defines the chip as a MPU so it can be called in the future

int16_t ax, ay, az;  // x y z orientation values from accelerometer
int16_t gx, gy, gz;  // x y z orientation values from gyrscope
int sen = analogRead(az);

////////////////////////////////////////////////////

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"                                  // statement to read the raw accel gyro data for verification.
#endif

#define OUTPUT_READABLE_ACCELGYRO

///////////////////////////////////////////////////////////////////////////////////////

 

////////////////////////////////////////////////////////////////////////////////////////
int led = 12;
int buzzer = 3;
int buttonpin = 1;     // initializing the pins and the global variables.
int button;
int ledb = 11;
int ign_pin= 9 ;
int safe_button = 7;



///////////////////////////////////////////////////////////////////////////////////////

int valo;     // outer val
int prevValo; // outer prev val

///////////////////////////////////////////////////////////////////////////////////////

int vali;  //inner val
int prevVali; //outer prev val
int buttonState = 0;
///////////////////////////////////////////////////

boolean buz = true;
boolean start = false;
boolean count = true;
boolean test = true;      // defining the boolean values for the while loop.
boolean armed = true;
boolean disarmed = false;
boolean but;

///////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////
int j=0;
int harshad;
int n = 0;
int i = 0;
int x = 1;
int y = 0;              // initializing the global variables.
int z = 0;
int t = 0;
int g = 0;
int w = 0;
int r = 0;
int k = 0;
//////////////////////////////////////////
//////////////victory song

#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
#define REST 0

// change this to make the song slower or faster
int tempo=144; 

// change this to whichever pin you want to use

// notes of the moledy followed by the duration.
// a 4 means a quarter note, 8 an eighteenth , 16 sixteenth, so on
// !!negative numbers are used to represent dotted notes,
// so -4 means a dotted quarter note, that is, a quarter plus an eighteenth!!
int melody[] = {

  //Based on the arrangement at https://www.flutetunes.com/tunes.php?id=192
  
  NOTE_E5, 4,  NOTE_B4,8,  NOTE_C5,8,  NOTE_D5,4,  NOTE_C5,8,  NOTE_B4,8,
  NOTE_A4, 4,  NOTE_A4,8,  NOTE_C5,8,  NOTE_E5,4,  NOTE_D5,8,  NOTE_C5,8,
  NOTE_B4, -4,  NOTE_C5,8,  NOTE_D5,4,  NOTE_E5,4,
  NOTE_C5, 4,  NOTE_A4,4,  NOTE_A4,8,  NOTE_A4,4,  NOTE_B4,8,  NOTE_C5,8,

  NOTE_D5, -4,  NOTE_F5,8,  NOTE_A5,4,  NOTE_G5,8,  NOTE_F5,8,
  NOTE_E5, -4,  NOTE_C5,8,  NOTE_E5,4,  NOTE_D5,8,  NOTE_C5,8,
  NOTE_B4, 4,  NOTE_B4,8,  NOTE_C5,8,  NOTE_D5,4,  NOTE_E5,4,
  NOTE_C5, 4,  NOTE_A4,4,  NOTE_A4,4, REST, 4,

  NOTE_E5, 4,  NOTE_B4,8,  NOTE_C5,8,  NOTE_D5,4,  NOTE_C5,8,  NOTE_B4,8,
  NOTE_A4, 4,  NOTE_A4,8,  NOTE_C5,8,  NOTE_E5,4,  NOTE_D5,8,  NOTE_C5,8,
  NOTE_B4, -4,  NOTE_C5,8,  NOTE_D5,4,  NOTE_E5,4,
  NOTE_C5, 4,  NOTE_A4,4,  NOTE_A4,8,  NOTE_A4,4,  NOTE_B4,8,  NOTE_C5,8,

  NOTE_D5, -4,  NOTE_F5,8,  NOTE_A5,4,  NOTE_G5,8,  NOTE_F5,8,
  NOTE_E5, -4,  NOTE_C5,8,  NOTE_E5,4,  NOTE_D5,8,  NOTE_C5,8,
  NOTE_B4, 4,  NOTE_B4,8,  NOTE_C5,8,  NOTE_D5,4,  NOTE_E5,4,
  NOTE_C5, 4,  NOTE_A4,4,  NOTE_A4,4, REST, 4,
  

  NOTE_E5,2,  NOTE_C5,2,
  NOTE_D5,2,   NOTE_B4,2,
  NOTE_C5,2,   NOTE_A4,2,
  NOTE_GS4,2,  NOTE_B4,4,  REST,8, 
  NOTE_E5,2,   NOTE_C5,2,
  NOTE_D5,2,   NOTE_B4,2,
  NOTE_C5,4,   NOTE_E5,4,  NOTE_A5,2,
  NOTE_GS5,2,};
  //////////////////////////////////////////
  // sizeof gives the number of bytes, each int value is composed of two bytes (16 bits)
// there are two values per note (pitch and duration), so for each note there are four bytes
int notes=sizeof(melody)/sizeof(melody[0])/2; 

// this calculates the duration of a whole note in ms (60s/tempo)*4 beats
int wholenote = (60000 * 4) / tempo;

int divider = 0, noteDuration = 0;

//////////////////////////////////////////

// Source: https://github.com/TKJElectronics/KalmanFilter

//#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
//
//Kalman kalmanX; // Create the Kalman instances
//Kalman kalmanY;
//
///* IMU Data */
//double accX, accY, accZ;
//double gyroX, gyroY, gyroZ;
//int16_t tempRaw;
//
//double gyroXangle, gyroYangle; // Angle calculate using the gyro only
//double compAngleX, compAngleY; // Calculated angle using a complementary filter
//double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
boolean tvc = false;

float h;
int s = 0;
boolean no = false;
boolean par = false;
boolean landed = false;
//uint32_t timer;
//uint8_t i2cData[14]; // Buffer for I2C data

// for millis() function as a delay :)


int ledState = LOW;  // ledState used to set the LED

unsigned long previousMillis = 0;       


const long interval = 100; 

///////////////////////////////////////////////////////////////////////////////////////////////////////

  
float PIDX, PIDY, errorX, errorY, previous_errorX, previous_errorY, pwmX, pwmY;
#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <Servo.h>
#define RESTRICT_PITCH // Comment out to restrict roll to 90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
Servo servoY;
Servo servoX;
Servo pyro_servo;
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;

int desired_angleX = 90;//servoY
int desired_angleY = 0; // servoX
int16_t tempRaw;

double gyroXangle, gyroYangle;// Angle calculate using the gyro only
double compAngleX, compAngleY;
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

float pidX_p=0;
float pidX_i=0;
float pidX_d=0;
float pidY_p=0;
float pidY_d=0;
float pidY_i=0;
/////////////////PID CONSTANTS/////////////////
double kp=0.4;//3.55
double ki=0;//2.05
double kd=0.04;//2.05
double tau = 0.07;
///////////////////////////////////////////////
int State = 0;
////////////////////////////////////////////////////////////////



const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  if (rcode) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
  }
  return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
  }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Serial.println(F("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }
  return 0; // Success
}
////////////////////////////////////////////////



void setup()

{
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();                                             // statement to read and refresh mpu raw data.
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();

  servoY.attach(5); //servo on pin 9 for large ring y
  servoX.attach(6);//servo on pin 10 for small ring x
  pyro_servo.attach(10);// servo on pin 11 for servo pyro
  pinMode(ledb, OUTPUT);
  pinMode(safe_button, INPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(led, OUTPUT);
 pinMode(ign_pin, OUTPUT);
  Wire.begin();
  Serial.begin(38400);// note that the baud rate is set to 38400.
  servoY.write(150);   // determining the pre-position angle


  Serial.println("Initializing MPU Sensors Onbord");
  mpu.initialize();

  while (buz) {
    tone(buzzer, 800);
    digitalWrite(ledb, HIGH);
    delay(20);
    noTone(buzzer);
    digitalWrite(ledb, LOW);
    delay(10);
    i++;
    if (i == 40) {
      buz = false; //////////////////////////// mpu initiallizing time.
    }
  }
  delay(500);
  servoY.write(90);
  pyro_servo.write(0);     
  delay(500);

  tone(buzzer, 900);
  delay(1000);
  noTone(buzzer);
  delay(1000);
  tone(buzzer, 800);
  delay(200);
  noTone(buzzer);
  tone(buzzer, 900);
  delay(100);
  noTone(buzzer);
  tone(buzzer, 1000);
  delay(100);
  noTone(buzzer);

  Serial.println(mpu.testConnection() ? "Connected succesfully" : "Connection failed error");

  delay(200);

}

///////////////////////////////////////////////////////////////////////////////////////
void loop()

{


  char nval = "X";
  char k = 0;

  while (disarmed) {
    if (k == 0) {
      Serial.println("ROCKET DISARMED SUCCESFULLY!!"); // print disarmed in the serial moniter.
      k++;
    }
    tone(buzzer, 300);
    delay(500);
    while (n < 1000) {
      noTone(buzzer);
      delay(1);
      if (digitalRead(2) == HIGH) {
        start = true;
        test = true;                       // declare all the booleans true to arm the rocket again.
        count = true;
        armed = true;
        disarmed = false;

        break;
      }
      n++;
    }
    n = 0;
    if (digitalRead(2) == HIGH) {
      start = true;
      test = true;
      count = true;        // declare all the booleans true to arm the rocket again.
      armed = true;
      r = 0;
      disarmed = false;

      break;
    }
  }
  while (start) {

    test = true;
    Serial.println("iam in start mode");
    if (r == 0) {
      Serial.println("");
      Serial.println("ROCKET IN STAND BY MODE");// print standby

      r++;
    }
    delay(500);
    tone(buzzer, 900);
    digitalWrite(led, HIGH); ///////////////////////////// Stand BY mode on
    delay(50);



    while (z < 1000) {
      noTone(buzzer);
      digitalWrite(led, LOW);
      delay(1);
      if (digitalRead(2) == HIGH) {
        test = true;               // if statement to exit the loop.
        y = 0;
        start = false;
        z = 101;
      }
      z++;
    }
    z = 0;
    if (digitalRead(2) == HIGH) {
      test = true;
      y = 0;
      start = false;
      break;
    }

  }



  while (test) {

    if (y == 0) {
      Serial.println("");
      Serial.println("Testing in progress");
      digitalWrite(led, HIGH);
      servoY.write(50);
      delay(200);
      servoY.write(90);
      delay(200);
      servoY.write(130);
      delay(200);
      servoY.write(90);
      servoX.write(90);
      Serial.println("Testing completed");

      y++;
    }
    delay(1300);
    test = false;
    

  }


  ///////////////////////////////////////////////////////////////////////////////////////////////////

  while (armed) {

    if (t == 0) {                                 // if statement to print the raw data only once.
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

      Serial.println("Rocket Armed succesfully");

      Serial.print("a/g:\t");
      Serial.print(ax); Serial.print("\t");      //////// checking some raw data from the sensor
      Serial.print(ay); Serial.print("\t");
      Serial.print(az); Serial.print("\t");
      Serial.print(gx); Serial.print("\t");
      Serial.print(gy); Serial.print("\t");
      Serial.println(gz);
      Serial.print("a/g:\t");
      Serial.print(ax); Serial.print("\t");
      Serial.print(ay); Serial.print("\t");
      Serial.print(az); Serial.print("\t");
      Serial.print(gx); Serial.print("\t");
      Serial.print(gy); Serial.print("\t");
      Serial.println(gz);
      Serial.println("READY FOR LAUNCH!!!");
      t++;
    }

    // UPDATING ALLTHE DATA OF THE MPU//

    while (i2cRead(0x3B, i2cData, 14));
    accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
    accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
    accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
    tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
    gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
    gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
    gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();

    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

    double gyroXrate = gyroX / 131.0; // Convert to deg/s
    double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
      kalmanX.setAngle(roll);
      compAngleX = roll;
      kalAngleX = roll;
      gyroXangle = roll;
    } else
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleX) > 90)
      gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
      kalmanY.setAngle(pitch);
      compAngleY = pitch;
      kalAngleY = pitch;
      gyroYangle = pitch;
    } else
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleY) > 90)
      gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

    gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;
    //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
    //gyroYangle += kalmanY.getRate() * dt;

    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

    // Reset the gyro angle when it has drifted too much
    if (gyroXangle < -180 || gyroXangle > 180)
      gyroXangle = kalAngleX;
      kalAngleX = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
      gyroYangle = kalAngleY;


   // Serial.print("kalAngleX is :::----");Serial.println(kalAngleX);
    digitalWrite(ledb, HIGH);
    digitalWrite(led, LOW);
    Serial.print("accy ==");Serial.println(accY);
    //delay(500);
    buttonState = digitalRead(safe_button);
//Serial.println(buttonState);
  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  //Serial.print(" X ::");Serial.print(kalAngleX); Serial.println("\t");
   //Serial.print(" y ::");Serial.print(kalAngleY); Serial.print("\t");


  //tone(buzzer, 700);
  //delay(2000);
  //noTone(buzzer);
  //tone(buzzer, 600);
  //delay(200);
  //noTone(buzzer);
   digitalWrite(ign_pin,HIGH);
  
   
     if (accY > -16204 ) {
      
      Serial.println("LAUNCH DETECTED");
      digitalWrite(ledb,LOW);
     
      tvc = true;
      armed= false;
}

   

  }


  while (tvc) {

    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;

      // if the LED is off turn it on and vice-versa:
      if (ledState == LOW) {
        ledState = HIGH;
      } else {
        ledState = LOW;
      }

      // set the LED with the ledState of the variable:
      digitalWrite(led, ledState);
    }
    while (i2cRead(0x3B, i2cData, 14));
    accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
    accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
    accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
    tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
    gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
    gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
    gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();

    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

    double gyroXrate = gyroX / 131.0; // Convert to deg/s
    double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
      kalmanX.setAngle(roll);
      compAngleX = roll;
      kalAngleX = roll;
      gyroXangle = roll;
    } else
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
      kalAngleX =kalAngleX  +90;

    if (abs(kalAngleX) > 90)
      gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
      kalmanY.setAngle(pitch);
      compAngleY = pitch;
      kalAngleY = pitch;
      gyroYangle = pitch;
    } else
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleY) > 90)
      gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

    gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;
    //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
    //gyroYangle += kalmanY.getRate() * dt;

    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

    // Reset the gyro angle when it has drifted too much
    if (gyroXangle < -180 || gyroXangle > 180)
      gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
      gyroYangle = kalAngleY;
  //  Serial.println (accX);
    noTone(buzzer);
    digitalWrite(ledb, LOW);
    digitalWrite(led, LOW);



    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    pwmX = max(pwmX, (desired_angleX - 20));
    pwmY = max(pwmY, (desired_angleY - 20));
    pwmX = min(pwmX, (desired_angleX + 20));
    pwmY = min(pwmY, (desired_angleY + 20));

    errorX = kalAngleX - desired_angleX;
    errorY = kalAngleY - desired_angleY;
    pidX_p = kp * errorX;
    pidX_d = (2*kd * (errorX - previous_errorX) + (2*kd - dt) * pidX_d )/ (2*tau+dt);
    ////////////////////////////////
    
    pidY_p = kp * errorY;
    /////////////////////////////////////
      pidY_d = (2*kd * (errorY - previous_errorY) + (2*kd - dt) * pidY_d )/ (2*tau+dt);
    pidX_i = ki * errorX * dt;
    pidY_i = ki * errorY * dt;
    
    ////////////////////////////////
    previous_errorX = errorX;
    previous_errorY = errorY;
    PIDX = pidX_p + pidX_i + pidX_d;
    PIDY = pidY_p + pidY_d+pidY_i;
    ////////////////////////////
    pwmX = 1 * ((PIDX * 4) + 90);
    pwmY = ((PIDY * 4) + 90);
    /////////////////////// 4 is the gear ratio of the servo




   servoX.write(pwmX); //////// WRITING THE VALUES TO THE SERVOS
   servoY.write(pwmY);
    Serial.print("kaly is :::::::");
    Serial.print(pwmY); 
    Serial.print("   kalx is :::::::: ");
    Serial.println(pwmX);
    //Serial.println(accY);
  if(kalAngleY <-10000 || kalAngleY>10000 ){


//    if (kalAngleY > 35 || kalAngleY < -35)  /// ABORT THE FLIGHT IF THE VALUES ARE MORE THAN 35 DEGREES
      flightabort();
      no = true;
      tvc = false;
    }


  }

  while (no) {
    Serial.println("ABORT");
    noTone(buzzer);
    delay(500);
    par = true;
    break;
  }
  while (par) {
    Serial.println("PYROS FIRED");
    digitalWrite(3, HIGH);
    tone(buzzer, 450);
    delay(3000);
    digitalWrite(3, LOW);
    noTone(buzzer);
    landed = true;
    break;

  }
  while (landed) {

    if (s == 0) {
      delay(4000);
      s++;
    }

    Serial.println("THE ROCKET HAS GROUNDED SUCCECFULLY!!");
    tone(buzzer, 900);
    digitalWrite(ledb, HIGH);
    delay(50);
    noTone(buzzer);
    digitalWrite(ledb, LOW);
    delay(1000);

  }




}



/////////////////////////////////////////////////////

void connection() {
  while (1) {

    if (Serial.available() > 0) {
      k = Serial.read();

      switch (k)
      {
        case 'p': digitalWrite(13, HIGH);
          Serial.println("GOING POSITIVE");
          servoY.write(i);
          i = i + 10;
          break;

        case 'n': digitalWrite(13, LOW);
          Serial.println("GOING NEGETIVE");
          servoY.write(i);
          i = i - 10;
          break;

        case 'f': Serial.println("FIRING PYROS");
          pyros();
          break;

        case 's': Serial.println("READING DATA");
        readData();
          break;

      }
    }

  }
}
///////////////////////////////////////////////////


void flightabort() {
  Serial.println("Flight aborted");
  Serial.println("Firing pyros");
  //pyro_servo.write(0);         // FUNCTION FOR ABORTING THE FLIGHT :(
  //delay(2000);
    pyro_servo.write(90);
       delay(1000);
    pyro_servo.write(0);
   digitalWrite(ign_pin,LOW);
  //digitalWrite(pyro, LOW);
  landed2();                         // go to check if grounded
}



void pyros() {
  Serial.println("Firing PYRO");
  //digitalWrite(pyro, HIGH);
  delay(2000);                      // FIRE PYRO ON COMMAND FROM APP ('V.I.R CONTROL')
  //digitalWrite(pyro, LOW);
}
///////////////////////////////////////////////


void landed2() {

while(1){
  if(kalAngleX <-41 || kalAngleX>40 ||kalAngleY <-41 || kalAngleY>40 ){
    delay(4000);
     if(kalAngleX <-41 || kalAngleX>40 ||kalAngleY <-41 || kalAngleY>40 ){
   
    while(1){
      Serial.println("landed succecfully");
     
       // iterate over the notes of the melody. 
  // Remember, the array is twice the number of notes (notes + durations)
  for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {

    // calculates the duration of each note
    divider = melody[thisNote + 1];
    if (divider > 0) {
      // regular note, just proceed
      noteDuration = (wholenote) / divider;
    } else if (divider < 0) {
      // dotted notes are represented with negative durations!!
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5; // increases the duration in half for dotted notes
    }

    // we only play the note for 90% of the duration, leaving 10% as a pause
    tone(buzzer, melody[thisNote], noteDuration*0.9);

    // Wait for the specief duration before playing the next note.
    delay(noteDuration);
    
    // stop the waveform generation before the next note.
    noTone(buzzer);
   
  }
    }
  }
  }
}
}
///////////////////////////////////////////////////////////////

void readData(){

while (i2cRead(0x3B, i2cData, 14));
    accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
    accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
    accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
    tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
    gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
    gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
    gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;
}
