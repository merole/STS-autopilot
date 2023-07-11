#include <FastIMU.h>
#include <SPI.h>
#include <Adafruit_BMP085.h>
#include <SoftwareSerial.h>

#include "algebra.c"
#include "Madgwick.h"

MPU6050 IMU;
Adafruit_BMP085 bmp;
SoftwareSerial HC12(0,1);

#define greenLed 6
#define redLed 5
#define yellowLed 7
#define button 4
#define delim ","
#define IMU_ADDRESS 0x68    //Change to the address of the IMU
#define PERFORM_CALIBRATION

calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;
MagData magData;
Madgwick filter;

float seaLevelP = 104626.5; // https://www.chmi.cz/aktualni-situace/aktualni-stav-pocasi/ceska-republika/stanice/profesionalni-stanice/mapy/tlak-vzduchu?l=cz

// A = np.array([[1.002951, -0.000413, 0.002362],  # 'A^-1' matrix from Magneto
//               [-0.000413, 1.004408, -0.000001],
//               [0.002362, -0.000001, 0.965185]])
// 'Combined bias (b)' vector from Magneto
// b = np.array([0.056450, -0.030570, -0.091198])

float A[3][3] = {{1.002951, -0.000413, 0.002362},
     {-0.000413, 1.004408, -0.000001},
     {0.002362, -0.000001, 0.965185}};
float b[3][1] = {{0.056450}, {-0.030570}, {-0.091198}};
float a[3][1] = {{0.},{0.},{0.}};
float acc[3];
float rot_acc[4];
float rot[4];
float a_temp[3][1] = {{0.},{0.},{0.}};
float v[3] = {0.0, 0.0, 0.0};
float s[3] = {0.0, 0.0, 0.0};
float up[3] = {0.0, 0.0, 1.0};
float q[4];
float dir[4];
float flight_dir[3];
float s_b = 0;
int dx = 50;
int target_alt;
int time;
int wait_time = 3000; // 3 s
int separated = 0;
float b_alt;
float norm;
int start_time;
int apogee_time;
int flight_status = 1;
float mag;
  
void setup() {
    Serial.begin(115200);
    // init LEDs; TODO configure pin numbers`
    pinMode(greenLed, OUTPUT);
    pinMode(redLed , OUTPUT);
    pinMode(yellowLed, OUTPUT);
    digitalWrite(yellowLed, LOW);

    // initialize imu
    int errm = IMU.init(calib, IMU_ADDRESS);
    if (errm != 0) {
      Serial.print("Error initializing IMU: ");
      Serial.println(errm);
      while (true) {
        ;
      }
    }

    // initialize baro
    if (!bmp.begin()) {
      Serial.println("Could not find a valid BMP085 sensor, check wiring!");
      while (1) {}
    }
  
  // calibration; TODO is this necessary??? do I need to calibrate mag???
  delay(1000);
  Serial.println("Keep IMU level.");
  digitalWrite(yellowLed, HIGH);
  IMU.calibrateAccelGyro(&calib);
  delay(5000);
  IMU.init(calib, IMU_ADDRESS);
  digitalWrite(yellowLed, LOW);
}

static uint32_t prev_ms = millis();
void loop() {
  // update imu every dx seconds
    if (millis() > prev_ms + dx) {
      // update flight data
      b_alt = bmp.readAltitude(seaLevelP);
      IMU.update();
      IMU.getAccel(&accelData);

      // process accceleration data for use not in filter!! TODO calibrate gyro and mag???
      a[0][0] = accelData.accelX;
      a[1][0] = accelData.accelY;
      a[2][0] = accelData.accelZ;
      subMat(a_temp, a, b);
      mulMat(acc, A, a_temp);
      mag = vecMag(acc);
      normalize(acc);
      prev_ms = millis();

      // print some data; TODO remove before flight
      {
      Serial.print(accelData.accelX);
      Serial.print("\t");
      Serial.print(accelData.accelY);
      Serial.print("\t");
      Serial.print(accelData.accelZ);
      Serial.print("\t");
      Serial.print(vecMag(acc));
      Serial.print("\t");
      Serial.println(b_alt);
      }

      // flight status detection
      /* 1 = before take off -> acc
         2 = powered and unpowered ascent -> (baro and time) or time
         3 = apogee reached, controlled descent -> acc = 1 g
         4 = landing
      */
      switch (flight_status) {
          case 1 : {
            if (vecMag(acc) > 2.0)
            {
              digitalWrite(yellowLed, HIGH);
              flight_status = 2;
              start_time = millis();
            }    
            break; 
          }
          case 2 : {
            updateFlight();
            // should I use alt or pure pressure??? TODO
            // apogee reached
            if ((b_alt > target_alt - 20 && millis() - start_time > 9000 && vecMag(acc) < 1.) || millis() - start_time > 9800) // TODO mozna na 10 000
            {
              apogee_time = millis();
              digitalWrite(yellowLed, LOW);         
              // for rocket arduino: wait x seconds
              flight_status = 3;
            }
            break;
          }
          case 3 : {
            updateFlight();
            if (millis() - apogee_time > 200 /* TODO ??? */) {
              // release chute
              flight_status = 4;
            }
          }
          case 4 : {
            ping(); // TODO check and finish
          }
        }
      
    }
}

// process direction vector for sending to ground station TODO integrate the direction and acceleration
void updateFlight() {
  IMU.getGyro(&gyroData);
  IMU.getMag(&magData);
  filter.update(gyroData.gyroX, gyroData.gyroY, gyroData.gyroZ, accelData.accelX, accelData.accelY, accelData.accelZ, magData.magX, magData.magY, magData.magZ);

  // convert to quat
  crossProduct(rot_acc, up, acc);
  rot_acc[3] = sqrt(pow(vecMag(acc), 2)) + dotProduct(up, acc);
  norm = sqrt(pow(rot[0], 2) + pow(rot[1], 2) + pow(rot[2], 2) + pow(rot[3], 2));
  for (int i = 0;i < 4; i++) {
    q[i] /= norm;
  }

  // get filter quaternion
  dir[0] = filter.getQuatW();
  dir[1] = filter.getQuatX();
  dir[2] = filter.getQuatY();
  dir[3] = filter.getQuatZ();          

  // convert rotation quaterniont to direction vector
  hamilton(rot, rot_acc, dir);
  toDirVec(flight_dir, dir);

  for (int i = 0; i < 3; i++) {
    v[i] += flight_dir[i] * mag;
    s[i] += v[i];
  }
}

void ping() {
  // send data to ground station TODO replace with s (  potentionally)
  char buf1[4];
  char buf2[4];
  char buf3[4];
  int ret1 = snprintf_P(buf1, sizeof buf1, "%f", dir[0]);
  int ret2 = snprintf_P(buf2, sizeof buf2, "%f", dir[1]);
  int ret3 = snprintf_P(buf3, sizeof buf3, "%f", dir[2]);
  
  char dest[15];
  strcpy(dest, ret1);
  strcat(dest, "\t");
  strcat(dest, ret2);
  strcat(dest, "\t");
  strcat(dest, ret3);

  HC12.write(dest);
}