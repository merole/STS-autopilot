#include <FastIMU.h>
#include <SPI.h>
#include <Adafruit_BMP085.h>
#include <SoftwareSerial.h>

#include "algebra.c"

MPU6050 IMU;
Adafruit_BMP085 bmp;
// TODO configure pin numbers
SoftwareSerial HC12(0, 1);

#define greenLed 6
#define redLed 5
#define yellowLed 7
#define button 4
#define delim ","
#define IMU_ADDRESS 0x68    //Change to the address of the IMU
#define PERFORM_CALIBRATION 

calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data

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
float a_temp[3][1] = {{0.},{0.},{0.}};
float v[3] = {0.0, 0.0, 0.0};
float s[3] = {0.0, 0.0, 0.0};
float top[3] {0.0, 0.0, 1.0};
float s_b = 0;
int dx = 50;
int target_alt;
int apogee_time;
int wait_time = 3000; // 3 s
int separated = 0;
float b_alt;
int start_time;
int flight_status = 1;
  
void setup() {
    Serial.begin(115200);
    // init LEDs TODO configure pin numbers
    pinMode(greenLed, OUTPUT);
    pinMode(redLed , OUTPUT);
    pinMode(yellowLed, OUTPUT);
    digitalWrite(yellowLed, LOW);

    // init IMU
    int errm = IMU.init(calib, IMU_ADDRESS);
    if (errm != 0) {
      Serial.print("Error initializing IMU: ");
      Serial.println(errm);
      while (true) {
        ;
      }

    // init baro
    }
    if (!bmp.begin()) {
      Serial.println("Could not find a valid BMP085 sensor, check wiring!");
      while (1) {}
    }
  
  // calibration; TODO remove this and do new calibration with matriecies
  delay(1000);
  Serial.println("Keep IMU level.");
  digitalWrite(yellowLed, HIGH);
  IMU.calibrateAccelGyro(&calib);
  delay(5000);
  IMU.init(calib, IMU_ADDRESS);
  digitalWrite(yellowLed, LOW);
}

int prev_ms = millis();
int beep_timer = 0;
void loop() {
  if (millis() > prev_ms + dx) {
    prev_ms = millis();

    updateFlight();
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
      b_alt = bmp.readAltitude(seaLevelP);
      
      IMU.update();
      IMU.getAccel(&accelData);
      prev_ms = millis();
      a[0][0] = accelData.accelX;
      a[1][0] = accelData.accelY;
      a[2][0] = accelData.accelZ;
      subMat(a_temp, a, b);
      mulMat(acc, A, a_temp);

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
            if ((b_alt > target_alt - 20 && millis() - start_time > 9000 && vecMag(acc) < 1.) || millis() - start_time > 9800) // TODO mozna na 10 000
          {
            apogee_time = millis();
            digitalWrite(yellowLed, LOW);         

            // separate stages TODO
            flight_status = 3;
          }
          break;
        }
        case 3 : {
          updateFlight();
          if (millis() - apogee_time > 200 /* TODO ??? */) {
            // release chute TODO
            flight_status = 4;            // ping with flight status TODO
          }
        }
        case 4 : {
          ping(); // TODO check and finish
        }
      }   
    }
}

void updateFlight() {
}

void ping() {

}
