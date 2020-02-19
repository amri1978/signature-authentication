// Include required C headers
#include <Adafruit_Sensor.h> 
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Wire.h>

// Addressing for BN0-055
#define BNO055_ADDRESS_A (0x28)
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS_A);


#define INT_PIN 0

// Declare data storage variables
float ax; float ay; float az; // acceleration components in SENSOR coordinates


// SETUP
void setup() {
  Serial.begin(115200); // Open serial communications and wait for port to open:
  Wire.begin(); // initialize the I2C interface
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  Serial.println("Serial port is connected");
  bool status; // initialize the BNO055 sensor and throw error if not detected
  status = bno.begin();
  if (!status) {
    Serial.println("BNO055 A is not detected");
    while (1);
  }

}// END SETUP



// LOOP
void loop() {

  if(Serial.available() > 0) // did something come in?
    if(Serial.read() == 'n') {
      Serial.println("-,-,-");
      for ( int i=0; i < 90+random(38); i++ ) {

          sensors_event_t event; // define event
          bno.getEvent(&event); // get event
          // extract Euler angles and linear acceleration vector.
          imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
          imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
          imu::Quaternion quat = bno.getQuat();           // Request quaternion data from BNO055

          /* Create Roll Pitch Yaw Angles from Quaternions */
          double yy = quat.y() * quat.y(); // 2 Uses below
              
          //double roll = atan2(2 * (quat.w() * quat.x() + quat.y() * quat.z()), 1 - 2*(quat.x() * quat.x() + yy));
          //double pitch = asin(2 * quat.w() * quat.y() - quat.x() * quat.z());
          double yaw = atan2(2 * (quat.w() * quat.z() + quat.x() * quat.y()), 1 - 2*(yy+quat.z() * quat.z()));
              
//              /*  Convert Radians to Degrees */
//              float rollDeg  = 57.2958 * roll;
//              float pitchDeg = 57.2958 * pitch;
//              float yawDeg   = 57.2958 * yaw;
        

          ax = 100*accel.x();
          ay = 100*accel.y();
          az = 100*accel.z();
          Serial.print(ay, 1);
          Serial.print(",");
          Serial.print(az , 1);
          Serial.print(",");
          Serial.print(yaw * 100 , 1);
          Serial.println();
          delay(10);
        }
    }
} // END MAIN
