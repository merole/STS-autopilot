#include <MPU9250.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP085.h>
#include "algebra.c"
#include <TXOnlySerial.h>

MPU9250 mpu;
File myFile;
Adafruit_BMP085 bmp;
TXOnlySerial HC12(0, 1);

#define greenLed 6
#define redLed 5
#define yellowLed 7
#define button 4
#define delim ","

int const pinSS = 10;
int const pinCS = 4;
bool debug = true; //set to true or false

int seaLevelP = 102200; // https://www.chmi.cz/aktualni-situace/aktualni-stav-pocasi/ceska-republika/stanice/profesionalni-stanice/mapy/tlak-vzduchu?l=cz

//A = np.array([[0.996635, 0.000786, -0.000194],  # 'A^-1' matrix from Magneto
//              [0.000786, 0.995838, -0.001037],
//              [-0.000194, -0.001037, 0.979470]])
//# 'Combined bias (b)' vector from Magneto
//b = np.array([0.125724, 0.078920, -0.173425])

float A[3][3] = {{0.9966, 0.0008, -0.0002},
     {0.0008, 0.9958, -0.0010},
     {-0.0002, -0.0010, 0.9794}};
float b[3][1] = {{0.1257}, {0.0789}, {-0.1734}};
float a[3] = {0.,0.,0.};
float v[3] = {0.0, 0.0, 0.0};
float s[3] = {0.0, 0.0, 0.0};
float s_b = 0;
int dx = 150;
int target_alt;
int time;
int wait_time = 3000; // 3 s
int separated = 0;

int flight_status[3] = [1, /* Before launch */, 2, /* Lift off */, 3 /* Apogee reached */]

void setup() {
    Serial.begin(115200);
    Wire.begin();
    pinMode(pinSS, OUTPUT);
    pinMode(greenLed, OUTPUT);
    pinMode(redLed , OUTPUT);
    pinMode(yellowLed, OUTPUT);
    digitalWrite(yellowLed, HIGH);
    delay(2000);

    while (!mpu.setup(0x68)) {  // change to your own address
      Serial.println("MPU9250 connection failed, trying again in 5s.");
      digitalWrite(redLed, HIGH);
      delay(5000);
    }

    while (!bmp.begin()) {
      Serial.println("BMP!180 connection failed, trying again in 5s.");
    }

    while(!SD.begin(pinCS) && !debug)
    {
      Serial.println("Sd Card connection failed, tryinig again in 5s.");
      delay(5000);
    }

    digitalWrite(greenLed, HIGH);
    delay(500);
    digitalWrite(yellowLed, HIGH);

    // calibrate();

    delay(500);
    digitalWrite(redLed, HIGH);
    delay(500);
    digitalWrite(redLed, LOW);
    digitalWrite(yellowLed, LOW);
}

static uint32_t prev_ms = millis();
void loop() {
    if (millis() > prev_ms + dx) {
      prev_ms = millis();
      if (mpu.update_accel_gyro()) {
            prev_ms = millis();

            printAcc();
            printAlt();
            if (fligh_status == 2) {
              integrate();            
            } 
            else if (flight_status == 3 && time + wait_time > millis() && !separated) {
              separate();
              separated = 1;
            }
        }
        switch flight_status: {
          case 1 {
            if (a[2] > 0.5)
            {
              // for rocket arduino, start integrating height and velocity
              // for plane arduino, nothing
            flight_status = 2;
            break; 
            }    
          }
          case 2 {
            if (a[2] < 3)
            {
              time = millis()
              // for rocket arduino: wait x seconds, release recovery system
              // for plane arudino: release parafoil, wait y seconds, engage autopilot
            flight_status = 3;
            break;
            }
          }
          case 3 {
            // idk
            break;
          }
        }
        
    }
}

void calibrate() {
    Serial.println("Acc gyro cal will begin i 3s.");
    delay(3000);
    mpu.calibrateAccelGyro();

    Serial.println("Mag calibration will start in 5sec.");
    Serial.println("Please Wave device in a figure eight until done.");
    delay(3000);
    mpu.calibrateMag();
    Serial.print("DONE");
}

void printAcc() {
  float ar[3][1];
  for (int i=0; i<3; i++) {
    ar[i][0] = mpu.getAcc(i);
  }
  getCalAcc(a, ar);
  Serial.print(sqrt(pow(a[0],2)+pow(a[1],2)+pow(a[2],2)));
  Serial.print(delim);
  printData(a, "Accel: ");
}

void printAlt() {
  float A[3] = {0.0, 0.0, 0.0};
  A[0] = bmp.readAltitude(seaLevelP);
  printData(A, "Alt");
}

void getCalAcc(float a[3], float ar[3][1]) {
  float res_inter[3][1] = {{0.}, {0.}, {0.}};
  subMat(res_inter, ar, b);
  mulMat(a, A, res_inter);
}

void printData(float data[3], char desc[]) {
    if (debug) {
    Serial.print(desc);
    Serial.print(data[0],2);
    Serial.print(delim);
    Serial.print(data[1],2);
    Serial.print(delim);
    Serial.println(data[2],2);
  } else {
    while (!myFile) {
      Serial.println("File could not be opened, check SD card. Retrying in 5 seconds");
      digitalWrite(redLed, HIGH);
      delay(5000);
    }
    digitalWrite(redLed, LOW);
    myFile.print(desc);
    myFile.print(data[0],2);
    myFile.print(delim);
    myFile.print(data[1],2);
    myFile.print(delim);
    myFile.println(data[2],2);
  }
}

void integrate() {
  for (int i=0;i++;i<3)
    v[i] += (1.0/dx) * a[i]; // WARNING: doesnt subtract g
    s[i] += (1.0/dx) * v[i];
  }
}

void separate() {
  
}