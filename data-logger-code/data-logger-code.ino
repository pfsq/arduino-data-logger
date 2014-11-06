#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include <avr/sleep.h>
#include <Wire.h>

// SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&Serial1);

uint8_t degree;
float dotdegree;

// Accelerometer setup
// Axis definition
// Correct directions x,y,z = gyro, accelerometer, magnetometer
int SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1};
// LSM303 accelerometer: 8 g sensitivity
// 3.9 mg/digit: 1 g = 256
#define GRAVITY 256
#define ToRad(x) ((x)*0.01745329252) // *pi/180
#define ToDeg(x) ((x)*57.2957795131) // *180/pi
// L3G4200D gyro: 2000 dps full scale
// 70 mdps/digit: 1 dps = 0.07
#define Gyro_Gain_X 0.07 // X axis Gyro gain
#define Gyro_Gain_Y 0.07 // Y axis Gyro gain
#define Gyro_Gain_Z 0.07 // Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) // Return the scaled ADC raw data of the gyro in radians per second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y))
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z))
// LSM303 magnetometer calibration constants
#define M_X_MIN -545
#define M_Y_MIN -714
#define M_Z_MIN -401
#define M_X_MAX 571
#define M_Y_MAX 321
#define M_Z_MAX 490

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

/* For debugging purposes */
#define OUTPUTMODE 1

#define PRINT_ANALOGS 0
#define PRINT_EULER 1

float G_Dt=0.02;

long timer=0;
long timer_old;
long timer24=0;
int AN[6];
int AN_OFFSET[6]={0,0,0,0,0,0};

int gyro_x, gyro_y, gyro_z;
int accel_x, accel_y, accel_z;
int magnetom_x, magnetom_y, magnetom_z;
float c_magnetom_x, c_magnetom_y, c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3]={0,0,0};
float Gyro_Vector[3]={0,0,0};
float Omega_Vector[3]={0,0,0};
float Omega_P[3]={0,0,0};
float Omega_I[3]={0,0,0};
float Omega[3]={0,0,0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3]={0,0,0};
float errorYaw[3]={0,0,0};

unsigned int counter=0;
byte gyro_sat=0;

float DCM_Matrix[3][3]={
  {1,0,0},
  {0,1,0},
  {0,0,1}
};
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}};
float Temporary_Matrix[3][3]={
  {0,0,0},
  {0,0,0},
  {0,0,0}
};

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true
/* set to true to only log to SD when GPS has a fix, for debugging, keep it false */
#define LOG_FIXONLY true  

// Set the pins used
#define chipSelect 10
#define ledPin 13

File logfile;

// read a Hex value and return the decimal equivalent
uint8_t parseHex(char c) {
  if (c < '0')
    return 0;
  if (c <= '9')
    return c - '0';
  if (c < 'A')
    return 0;
  if (c <= 'F')
    return (c - 'A')+10;
}

// blink out an error code
void error(uint8_t errno) {
  while(1) {
    uint8_t i;
    for (i=0; i<errno; i++) {
      digitalWrite(ledPin, HIGH);
      delay(100);
      digitalWrite(ledPin, LOW);
      delay(100);
    }
    for (i=errno; i<10; i++) {
      delay(200);
    }
  }
}

void setup() {
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  // Serial.begin(115200);
  // Serial.println("\r\nUltimate GPSlogger Shield");
  pinMode(ledPin, OUTPUT);

  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // I2C initialization
  I2C_Init();
  delay(1500);
  Accel_Init();
  Gyro_Init();
  
  for(uint8_t i=0; i<32; i++){
    Read_Gyro();
    Read_Accel();
    for(uint8_t y=0; y<6; y++){
      AN_OFFSET[y] += AN[y];
    }
    delay(20);
  }
  for(uint8_t y=0; y<6; y++){
    AN_OFFSET[y] = AN_OFFSET[y]/32;
  }
  AN_OFFSET[5] -= GRAVITY*SENSOR_SIGN[5];
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect, 11, 12, 13)) {
//  if (!SD.begin(chipSelect)) {      // if you're using an UNO, you can use this line instead
//     Serial.println("Card init. failed!");
    error(2);
  }
  // error(5);
  char filename[15];
  strcpy(filename, "LOG00.csv");
  for (uint8_t i = 0; i < 100; i++) {
    filename[3] = '0' + i/10;
    filename[4] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (!SD.exists(filename)) {
      break;
    }
  }

  logfile = SD.open(filename, FILE_WRITE);
  if( ! logfile ) {
//     Serial.print("Couldnt create "); Serial.println(filename);
    error(3);
  }
//   Serial.print("Writing to "); Serial.println(filename);
  logfile.println("DATE;TIME;LATITUDE;LONGITUDE;ALTITUDE;SPEED (KPH);HEADING;FIX;LON G;LAT G;VER G;GYRO X;GYRO Y;GYRO Z"); logfile.flush();

  // connect to the GPS at the desired rate
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For logging data, we don't suggest using anything but either RMC only or RMC+GGA
  // to keep the log files at a reasonable size
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);   // 1 or 5 Hz update rate

  // Turn off updates on antenna status, if the firmware permits it
  GPS.sendCommand(PGCMD_NOANTENNA);
  
  // Serial.println("Ready!");
}

void loop() {
   char c = GPS.read();
   if (GPSECHO)
      if (c)   Serial.print(c);

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
        
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
    
    // Sentence parsed! 
    Serial.println("OK");
    if (LOG_FIXONLY && !GPS.fix) {
//         Serial.print("No Fix");
        return;
    }
    
    Read_Gyro();
    Read_Accel();
    // Calculations
    Matrix_update();
    Normalize();
    Drift_correction();
    Euler_angles();

    // Rad. lets log it!
//     Serial.println("Log");
    
    logfile.print("20");
    logfile.print(GPS.year, DEC); logfile.print('-');
    logfile.print(GPS.month, DEC); logfile.print('-');
    logfile.print(GPS.day, DEC); logfile.print(';');
    logfile.print(GPS.hour, DEC); logfile.print(':');
    logfile.print(GPS.minute, DEC); logfile.print(':');
    logfile.print(GPS.seconds, DEC); logfile.print('.');
    logfile.print(GPS.milliseconds, DEC); logfile.print(';');
    // LATITUDE
    if (GPS.lat == 'S')
        logfile.print('-');
    degree = (int)GPS.latitude / 100;
    dotdegree = degree + (GPS.latitude - degree*100)/60.0;
    logfile.print(dotdegree,6); logfile.print(';');
    // LONGITUDE
    if (GPS.lon == 'W')
        logfile.print('-');
    degree = (int)GPS.longitude / 100;
    dotdegree = degree + (GPS.longitude - degree*100)/60.0;
    logfile.print(dotdegree,6); logfile.print(';');
    // ALTITUDE
    logfile.print(GPS.altitude); logfile.print(';');
    // SPEED
    logfile.print((float)GPS.speed * 1.852); logfile.print(';');
    // HEADING
    logfile.print(GPS.angle); logfile.print(';');
    // FIX
    logfile.print(GPS.fix); logfile.print(';');
    // LON G
    logfile.print(Accel_Vector[0]/GRAVITY); logfile.print(';');
    // LAT G
    logfile.print(Accel_Vector[1]/GRAVITY); logfile.print(';');
    // VER G
    logfile.print(Accel_Vector[2]/GRAVITY); logfile.print(';');
    // GYRO X
    logfile.print(Gyro_Vector[0]); logfile.print(';');
    // GYRO Y
    logfile.print(Gyro_Vector[1]); logfile.print(';');
    // GYRO Z
    logfile.println(Gyro_Vector[2]);
    logfile.flush();
  }
}


/* End code */
