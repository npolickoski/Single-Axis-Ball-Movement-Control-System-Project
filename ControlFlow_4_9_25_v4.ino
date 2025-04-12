
 /*                 ROVER Tests
 *              Mega Auto Mode Test Code
 *                   3 June 2024             
 *             Eliminatel Compass
 * 2023                   
 *   5/9  R/C  smooth speed control
 *   5/10 Encoder Tests w/Mega interrupts
 *   5/10 IMU
 *   5/10 Odometry w/Mega Timer: Es = 0.9520
 *   5/12 Horn
 *   5/13 Speed Control - 26 May 2024 Turned Off
 *   5/16 Camera Servo
 * 2024  
 *   3/11  cal encoders  Cm = 
 *   5/26
 *     No Lidar
 *     No Compass
*/                 

/* R/C Pulsewidths 
 * R/C Mode, Speed and Turn input from R/C Receiver or Mega
 * R/C Control Set HIGH when R/C Mode STOP or Drive; LOW when Auto
 * R/C Active Set HIGH when R/C Mode STOP or Drive
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); //board I2C address

#define SERVOMIN   80           // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  465           // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50           // Analog servos run at ~50 Hz updates

volatile long right_cnt = 0;
volatile long left_cnt  = 0;

bool LogDrive = true;           // true - logo records in Drive Mode; false - logo records in Auto Mode

//  GPS
const byte numChars = 50;
char receivedChars[numChars];
char tempChars[numChars];       // temporary array for use by strtok() function
// variables to hold the parsed data
float latitude_input = 0.0;
float longitude_input = 0.0;
//Created on 12-10-2024 for GPS recurrence traversal
float EndLat =  34.722358703613281;           //Yellow End Point
float EndLong = -86.639678955078125;
float StartLat = 34.722347259521484 ;         //Red Start point
float StartLong= -86.640014648437500;
float Plat ;
float Plong ;
float Clat ;
float Clong ;
float Elat ;
float Elong ;
int numWaypoints = 2;                         // Number of Waypoints in the mission // Specify for Greyson to Send to us
float waypointArray[2];                       // Move this into input section later every pair of coordinates latitudes are odd and even or longitudes 
int waypointFlag = 0;                         // Flag tracks what waypoint the program is on
bool haverFlag = 0;
int turndecider = 1;

boolean newData = false;
bool accelFlag = true;

byte LtSpd;
byte RtSpd;
byte sendCmd;

int rc_active = 6;              //Signal from NanoEvery indicating R/C Control when high
int horn = 7;
int busy = 0;
int leftDelta;
int rightDelta;
int lastLeft;
int lastRight;
int mtrSpeed;
int turnSpeed;
int SaberSpeed;
int tUpdate;                    // odometry and speed control update, ms
int servonum = 0;
int pulselen;

float pi = 3.1415926535;
float RadTDeg = 180/pi;
float servoAngle;
float x;
float y;
float U;
float theta;
float pitch;
float roll;
float speedConst;

// (cnts/cm)  = (xxxx cnts/rev)*(rev/(pi*wheelDiam)) = yyyy cnts/cm
//  Cm value depends on which encoder transitions counted and tire inflation
//float Cm  = 4.72479e-3;   // cm/cnt  5/10/2023
float Cm  = 4.79675e-3;     // cm/cnt 3/11/2024
float Es = 0.9520;          //QuadRover Labbot - measured over 30m
float CmStar = Es*Cm;

long T0;
long T;

// New Additions Below
int debug = 0;                  // Setting this to will print debug statements
int flag = 0;                   // Flag to control what drive state the motor is in
int ClockStartFlag = 0;         // This is used to make the clock not restart every iteration of the loop
int EndClockFlag = 0;           // This is used to reset the clock flag so it will restart the 
int ResetEndClockFlag = 0;      // Another value that is used to reset the flags when the rover is taken in or out of auto
int GPS_Get_Flag = 0;
int Turn_To_Heading_Flag = 0;
int Three_Sixty_Heading = 0;
unsigned long millisTime = 0;       // Variable to hold millis so its only called once per iteration of the loop
unsigned long MotorStartMillis; // This value is used to grab the time that the delays are based on 
unsigned long motorDelay5000 = 5000;
unsigned long motorDelay7000 = 7000;
int heading = 0;                // This is the value the rover will try to point itself to*/
int DriveTime = 0;              // Flag for creating delay when driving to get to next point
int SetDriveTime = 0;           // Flag to stop that previous from being reset
int DriveTimeStart = 0;             // Variable to hold the time when mode 3 in entered
int DriveTimeLength = 5000;    // How long to run when in ms
float tolerance = 3;          // 3 Meters around the way point

int ForwardSpeed = 30;
int TurnSpeed = 3;
int newHeading = 0;
int forwardClockFlag = 0;
unsigned long forwardStartMillis;

void Drive_Forward()
{
  if(ClockStartFlag == 0)     // This segment of code takes the time only at the first iteration of case 
  {                           // one in the broader loop function
    MotorStartMillis = millis();
    ClockStartFlag = 1;
  }
  if(millis() - MotorStartMillis <= 5000)
  {                           // Run motors only in 5 second interval
    MtrFwd(ForwardSpeed);
    MtrLeft(3);
  }
  else if(millis() - MotorStartMillis > 5000)
  {
    MtrStop();
    flag = 2;                 // Move on to case 2 when time is exceeded
    newHeading = 0;
  }

}
unsigned long startHaversine, endHaversine;
void Calculate_Heading()
{
  Serial.println("calcHeading");
  if (newHeading == 0)
  {
    Serial.println("Calculating New Heading");
    startHaversine = millis();
    Plat = Clat;                                  // Loads the Calibration Point into the Precious Latitude
    Plong = Clong;                                // Loads the Calibration Point into the Previous Longitude
    // Takes The GPS Point and stores it into current gps point
    Clat = latitude_input;                        // Current Latitude will be modified [As in change these initializations]
    Clong = longitude_input;                      // Current Longitude will be modified
          
    // Checks if we are at the waypoint by checking it is in the tolerance range of the waypoint
    if(arePointsWithinTolerance(Clat,Clong,Elat,Elong, tolerance) == true)
    {                                             // If points are within tolerance, go to case four
      flag = 4;                                   // Commented out so it won't cause any short term problems when doing control flow
    }
    //Calculates the heading of the rover
    float side1 = haversine(Plat,Plong,Clat,Clong);
    float side2 = haversine(Plat,Plong,Elat,Elong);
    float side3 = haversine(Clat,Clong,Elat,Elong);
    float Cangle = calcAngle(side1, side3, side2);
    float angle1 = calcAngle(side3, side2, side1);
    float angle2 = calcAngle(side3, side1, side2);
    float angle3 = calcAngle(side2, side1, side3);
    Serial.print("Cangle: ");
    Serial.println(Cangle);
     Serial.print("angle1: ");
    Serial.println(angle1);
     Serial.print("angle3: ");
    Serial.println(angle3);
    heading = abs(180-Cangle);
    // Utilizes checkCrossProduct to determine if the heading is to the right, left or within +/-1 degree of our desired heading
    int turndecider= checkCrossProduct(Plat,Plong,Clat,Clong,EndLat,EndLong);


    // Testing with Rover test points at the lawn
    // Start:34.723121643066406, -86.639823913574218
    // Cal: 34.723247528076171, -86.639915466308593
    // End: 34.722866058349609, -86.639823913574218
    //  float side1 = haversine(34.722866058349609,-86.639823913574218,34.723121643066406,-86.639823913574218);
    //  float side2 = haversine(34.723121643066406,-86.639823913574218,34.723247528076171,-86.639915466308593);
    //  float side3 = haversine(34.722866058349609,-86.639823913574218,34.723247528076171,-86.639915466308593);
    //  float Cangle = calcAngle(side2, side1, side3);
    //  float Bangle = calcAngle(side3, side1, side2);
    //  float Aangle = calcAngle(side2, side3, side1);
    // Serial.print("Cangle: ");
    // Serial.println(Cangle);
    // Serial.print("Bangle: ");
    // Serial.println(Bangle);
    // Serial.print("Aangle: ");
    // Serial.println(Aangle);
    //  heading = abs(180-Bangle);
    // turndecider= checkCrossProduct(34.723121643066406,-86.639823913574218,34.723247528076171,-86.639915466308593,34.722866058349609,-86.639823913574218);
    
    Serial.print("Calculated Heading: ");
    Serial.println(heading);
    Serial.print("Calculated turndecider: ");
    Serial.println(turndecider);
    
    
    if (turndecider > 0)
    {
      heading = -heading;
    }
  else if (turndecider < 0)
    {
      heading = heading;
    }
    newHeading = 1;
    Serial.print("Turndecider Heading: ");
    Serial.println(heading);
    endHaversine = millis();
    Serial.print("startHaversine: ");
    Serial.println(startHaversine);
    Serial.print("endHaversine: ");
    Serial.println(endHaversine);
    Serial.print("Time Taken: ");
    Serial.println(endHaversine-startHaversine);
    Serial.print("GPS Data: ");
    Serial.print(latitude_input, 15);
    Serial.print(", ");
    Serial.println(longitude_input, 15);
    Serial.print("Previous Point : ");
    Serial.print(Plat,15);
    Serial.print(", ");
    Serial.println(Plong,15);
    Serial.print("Current Point : ");
    Serial.print(Clat,15);
    Serial.print(", ");
    Serial.println(Clong,15);
    Serial.print("End Point : ");
    Serial.print(Elat,15);
    Serial.print(", ");
    Serial.println(Elong,15);
    Turn_To_Heading_Flag = 1;
  }

  //Serial.println(heading);
  //Serial.println(turndecider);
}

void Turn_To_Heading()
{
  Serial.println("turntoHeading");
  theta = IMU();
  theta= ( ( int(theta) + 360) % 360);
  heading= ((heading+360)%360);
  Serial.println(theta);
  Serial.println(heading);

  //if(heading < theta) heading += 360; // Don't think we need this if heading is already out of 360
  int left = heading - theta;
  // Also note that I may have gotten the positions of heading and theta reversed, the post doesn't make it quite clear
  // Take smallest turn
  int marginOferror = 5;
  if (theta > heading + marginOferror)
    {
      MtrRight(64);// Speed of 63 was what previous team used but they had different motor controls, differential is what we have now.
    }
  else if (theta < heading -marginOferror )
    {
      MtrLeft(64);
    }
  else if ((theta > heading - marginOferror) && (theta < heading + marginOferror)) 
    {
      MtrStop();
      // Ths section will likely need adjusting
      //heading = IMU();
      SetDriveTime = 1; // A designated set time that the rover will drive straight until it rechecks
      flag = 3;         // Go to case 3 to drive to heading when aligned
    }
}

void Drive_On_Heading()
{
  //
  InitializeIMU();
  Serial.println(theta);
  Serial.println("towardsHeading");
  if((theta > heading - 1) && (theta < heading + 1)) // check if close enough to heading to go straight
  {
    //Serial.println("Near Heading");
    MtrFwd(ForwardSpeed);
    MtrLeft(3);
  }
  if(forwardClockFlag == 0)     // This segment of code takes the time only at the first iteration of case 
  {                           // one in the broader loop function
    forwardStartMillis = millis();
    forwardClockFlag = 1;
  }
  if(millis() - forwardStartMillis <= 7000)
  {                           // Run motors only in 5 second interval
    MtrFwd(ForwardSpeed);
    MtrLeft(3);
  }
  else if(millis() - forwardStartMillis > 7000)
  {
    MtrStop();
    flag = 4;                 // Move on to case 2 when time is exceeded
  }

}

void Load_Waypoint()
{
  Serial.println("loadingWaypoint");

}

void Print_Case()   // Tried to make it print once per second so it didn't spam the terminal but couldn't get it to work
{
  //if(millisTime % 1000 == 0)
  //{
    Serial.print("Case ");
    Serial.println(flag);
  //}
}

// Leave space for potential start point and end point make sure to have start and end points

void setup() {
  delay(1000);
  InterruptSetup(); 
  InitializeSerialPorts();
  InitializeIMU();
  SetPinModes();
  InitializeVariables();
//  Set speed for SpeedControl  
  speedConst = 100;  // cm/s

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates  
  
// set camera pointing servo level
  servoAngle = 62;
  pulselen = Point(servoAngle);
  pwm.setPWM(servonum, 0, pulselen);  
  delay(500);   
  T0 = millis();
  GPS(); 

// Predefined Waypoints for hardcoded
// Point 1: 34.7206610, -86.6394960
// Point 2: 34.7206800, -86.6392290
// Point 3: 34.7206880, -86.6388550
// Point 4: 34.7206460, -86.6384350
// Point 5: 34.7204510, -86.6388780

// Lawn Points
// Start:34.723121643066406, -86.639823913574218
// Cal: 34.723247528076171, -86.639915466308593
// End: 34.722866058349609, -86.639823913574218
// Testing Code
waypointArray[0] = 34.723121643066406;
waypointArray[1] = -86.639823913574218;
waypointArray[2] = 34.723247528076171;
waypointArray[3] = -86.639915466308593;

latitude_input = waypointArray[waypointFlag];      // 1st, 3rd etc are on indexes 0, 2 and onward therefore the indexes must be even
longitude_input = waypointArray[waypointFlag + 1]; // 2nd, 4th ... are on indexes 1, 3 and onward therefore the indexes must be odd
 Plat = latitude_input;                        // Storing Previous GPS Latitude
 Plong = longitude_input;                      // Storing Previous GPS Longitude
 Clat = Plat;                            // Updates the Current Latitude based on GPS Point Taken
 Clong = Plong;                          // Updates the Current Longitude based on GPS Point Taken
 Elat = waypointArray[2];
 Elong = waypointArray[3];

 haverFlag = 0;
 
}

void loop() { 
    busy = digitalRead(rc_active);                      //busy = 0 when in Auto Mode  
    
    millisTime = millis();                              // Get millis once at the start of each loop
    //Serial.print("millisTime: ");
    //Serial.println(millisTime);
    if(busy == 0) 
    { 
      T = millisTime;
      //Serial.println(flag);
      switch(flag)
      {
        case 0:
          GPS_With_Flag();                              // Use GPS with flag to get ensure initial point is retreived from GPS call
          Serial.println("Case 0");
          if(GPS_Get_Flag == 1)                         // Don't run the rest of this until GPs has sent new data
          {
            waypointArray[0] = latitude_input;            // Load latitude into initial GPS point
            waypointArray[1] = longitude_input;           // Load longitude into inital GPS point
            GPS_Get_Flag = 0;                             // Reset GPS get flag so second point works
            flag = 1;                                     // Go onto case 1
          }
          break;  
        case 1:
          //Print_Case();
          Serial.println("Case 1");
          Drive_Forward();
          break;
        case 2:
          //Print_Case();
          Serial.println("Case 2");
          
          if(GPS_Get_Flag == 0)
          {
            GPS_With_Flag(); // This function sets flag once new data
          }
        
          if(GPS_Get_Flag == 1)
          {
            Calculate_Heading();
          }
          if(Turn_To_Heading_Flag ==1)
          {
            Turn_To_Heading();
            // need to reset GPS_Get_Flag once out of case two so this will all work when the case is re-entered
          }
          break;
        case 3: 
          Print_Case();
          Drive_On_Heading();
          break;
        case 4: // Forgot to include: Calculate how far the rover is to waypoint, if the rover is at the waypoint, then load in the next waypoint
          Print_Case();
          Load_Waypoint();
          break;
      }
      if((T - T0) >= tUpdate)   // Log Data
      {
        PrintDrive();
        theta = IMU();
        PrintIMU();         
        GPS();    
        T0 = millis();
      }  

    }
    else 
    {  
      if (LogDrive == false) 
      {
//      Serial.println("Drive Mode False = ");
//      accelFlag = true;
        T0 = millis();
        MtrStop();
        delay(20);
        x = 0;
        y = 0;
        U = 0; 
        left_cnt = 0;
        right_cnt = 0;    
        leftDelta = 0;
        rightDelta = 0;
      }   
      else 
      {
//      Log In Drive Mode
//      Serial.println("Drive Mode True ");
        T = millis();
        if((T - T0) >= tUpdate) 
        {
          PrintDrive();          
          theta = IMU();
          PrintIMU();         
          GPS();    
          T0 = millis();
        }      
      }   
   }

  if(busy == 1)         // Reset everything when taken out of auto
  {
    flag = 0;
    ClockStartFlag = 0; 
    GPS_Get_Flag = 0;
    newHeading = 0;
  }
}

