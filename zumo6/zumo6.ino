#include <Wire.h>
#include <ZumoShield.h>
#include <SoftwareSerial.h>

#define rxPin 3
#define txPin 6  //TX에 꼽기
#define TEST 0

//com8, port86
// SENSOR_THRESHOLD is a value to compare reflectance sensor
// readings to to decide if the sensor is over a black line
#define SENSOR_THRESHOLD 700

// ABOVE_LINE is a helper macro that takes returns
// 1 if the sensor is over the line and 0 if otherwise
#define ABOVE_LINE(sensor)((sensor) > SENSOR_THRESHOLD)

// Motor speed when turning. TURN_SPEED should always
// have a positive value, otherwise the Zumo will turn
// in the wrong direction.
#define TURN_SPEED 150

// Motor speed when driving straight. SPEED should always
// have a positive value, otherwise the Zumo will travel in the
// wrong direction.
#define SPEED 90
#define DELAYN 500

// Thickness of your line in inches
#define LINE_THICKNESS .78

#define INCHES_TO_ZUNITS 17142.0

// When the Zumo reaches the end of a segment it needs
// to find out if there is a straight segment ahead of it, and which
// segment to take. OVERSHOOT tells the Zumo how far it needs
// to overshoot the segment to find out any of these things.
#define OVERSHOOT(line_thickness)(((INCHES_TO_ZUNITS * (line_thickness)) / SPEED))

SoftwareSerial esp8266(txPin, rxPin);
String Data = "";
String res = "";
int flag = 0;
int turnflag = 0;
int avoidflag = 0;
unsigned long start = 0;
unsigned long end = 0;
//ZumoBuzzer buzzer;
ZumoReflectanceSensorArray reflectanceSensors;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);

void setup()
{
  //randomSeed(analogRead(0));  // Initialize random number generator
  Serial.begin(9600);
  esp8266.begin(9600);
  reflectanceSensors.init();

  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);        // turn on LED to indicate we are in calibration mode
  
  button.waitForButton();

  // Calibrate the Zumo by sweeping it from left to right
  unsigned long startTime = millis();
  Serial.println("check");
  while(millis() - startTime < 4000)   // make the calibration take 10 seconds
  {
    reflectanceSensors.calibrate();
    motors.setSpeeds(TURN_SPEED, -1 * TURN_SPEED);
  }
  /*for(int i = 0; i < 4; i++)
  {
    // Zumo will turn clockwise if turn_direction = 1.
    // If turn_direction = -1 Zumo will turn counter-clockwise.
    //int turn_direction = i % 2 == 0 ? 1 : -1;
    reflectanceSensors.calibrate();
    motors.setSpeeds(turn_direction * TURN_SPEED, -1*turn_direction * TURN_SPEED);

    // This while loop monitors line position
    // until the turn is complete.
    unsigned int sensors[6];
    unsigned int lastSensors[6] = {0};
    unsigned int count = 0;
    while(count < 2)
    {
      reflectanceSensors.calibrate();
      reflectanceSensors.read(sensors);
      
      // Check if the outer sensor has changed from line to no-line or vice versa
      if ((ABOVE_LINE(sensors[0]) != ABOVE_LINE(lastSensors[0])) ||
          (ABOVE_LINE(sensors[5]) != ABOVE_LINE(lastSensors[5])))
      {
        count++;
      }
      
      memcpy(lastSensors, sensors, sizeof(sensors));
    }
  }
  */

  motors.setSpeeds(0, 0);

  // Sound off buzzer to denote Zumo is finished calibrating
  ////buzzer.play("L16 cdegreg4");

  // Turn off LED to indicate we are through with calibration
  digitalWrite(13, LOW);

  //button.waitForButton();

  sendData("AT+CIPMUX=1\r\n", 1000);  // 다중 연결 허용
  sendData("AT+CIPSERVER=1,80\r\n", 1000);  // 포트 80에서 TCP 서버 시작
  sendData("AT+CIFSR\r\n",1000);

}

// ... rest of the code remains the same ...

void loop()
{
#if TEST
  turn('R');
  exploreRandomly();
  motors.setSpeeds(0, 0);
  button.waitForButton();
  turn('R');
  exploreRandomly();
  motors.setSpeeds(0, 0);
  button.waitForButton();
  turn('R');
  exploreRandomly();
  motors.setSpeeds(0, 0);
  button.waitForButton();
  turn('R');
  exploreRandomly();
  motors.setSpeeds(0, 0);
  button.waitForButton();
#endif
  //exploreRandomly();
#if !TEST
  if(esp8266.available()){
    char command = esp8266.read();
    //Serial.println(command);
    if(command == '4'){
      clearWifiBuffer();
      res = String(0);
      responseData("AT+CIPSEND=0," + String(res.length()) + "\r\n", 50);
      responseData(res,50);
      clearWifiBuffer();
      turn('S');
      exploreRandomly();
      res = String(1);
      responseData("AT+CIPSEND=0," + String(res.length()) + "\r\n", 50);
      responseData(res,50);
      clearWifiBuffer();
    }
    else if(command == '5'){
      clearWifiBuffer();
      res = String(0);
      responseData("AT+CIPSEND=0," + String(res.length()) + "\r\n", 50);
      responseData(res,50);
      clearWifiBuffer();
      turn('L');
      exploreRandomly();
      res = String(1);
      responseData("AT+CIPSEND=0," + String(res.length()) + "\r\n", 50);
      responseData(res,50);
      clearWifiBuffer();
    }
    else if(command == '6'){
      clearWifiBuffer();
      res = String(0);
      responseData("AT+CIPSEND=0," + String(res.length()) + "\r\n", 50);
      responseData(res,50);
      clearWifiBuffer();
      turn('R');
      exploreRandomly();
      res = String(1);
      responseData("AT+CIPSEND=0," + String(res.length()) + "\r\n", 50);
      responseData(res,50);
      clearWifiBuffer();
    }
    else if(command == '7'){
      clearWifiBuffer();
      res = String(0);
      responseData("AT+CIPSEND=0," + String(res.length()) + "\r\n", 50);
      responseData(res,50);
      clearWifiBuffer();
      turn('B');
      exploreRandomly();
      res = String(1);
      responseData("AT+CIPSEND=0," + String(res.length()) + "\r\n", 50);
      responseData(res,50);
      clearWifiBuffer();
    }
    else if(command == '8'){
      clearWifiBuffer();
      res = String(0);
      responseData("AT+CIPSEND=0," + String(res.length()) + "\r\n", 50);
      responseData(res,50);
      clearWifiBuffer();
      avoidflag = 1;
      turn('L');
      exploreRandomly();
      motors.setSpeeds(0, 0);
      res = String(1);
      responseData("AT+CIPSEND=0," + String(res.length()) + "\r\n", 50);
      responseData(res,50);
      clearWifiBuffer();
      avoidflag = 0;
    }
    else if(command == '9'){
      clearWifiBuffer();
      res = String(0);
      responseData("AT+CIPSEND=0," + String(res.length()) + "\r\n", 50);
      responseData(res,50);
      clearWifiBuffer();
      avoidflag = 1;
      turn('R');
      exploreRandomly();
      motors.setSpeeds(0, 0);
      res = String(1);
      responseData("AT+CIPSEND=0," + String(res.length()) + "\r\n", 50);
      responseData(res,50);
      clearWifiBuffer();
      avoidflag = 0;

    }
  }
#endif
}

void responseData(String command, const int timeout){
  esp8266.println(command);
  delay(timeout);
}
void sendData(String command, const int timeout) {
  esp8266.println(command);  // 명령어 전송
  long int time = millis();
  while ((time + timeout) > millis()) {
    while (esp8266.available()) {
      char c = esp8266.read();  // ESP8266에서 응답 읽기
      Serial.write(c);  // PC 모니터로 출력
    }
  }
}

// Turns according to the parameter dir, which should be
// 'L' (left), 'R' (right), 'S' (straight), or 'B' (back).
void turn(char dir)
{
  switch(dir)
  {
    case 'L':
      // Turn left.
      motors.setSpeeds(SPEED, SPEED);
      delay(DELAYN);
      motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
      delay(250);
      turnFunc();  // Adjust this delay to make a proper 90-degree turn
      break;
    case 'R':
      // Turn right.
      motors.setSpeeds(SPEED, SPEED);
      delay(DELAYN);
      motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
      delay(300);
      turnFunc();
      
      //delay(730);  // Adjust this delay to make a proper 90-degree turn
      break;
    case 'B':
      // Turn back.
      motors.setSpeeds(-(TURN_SPEED), TURN_SPEED);
      delay(200);
      turnFunc();
      //backturnFunc();
      //delay(1400);  // Adjust this delay to make a proper 180-degree turn
      break;
    case 'S':
      motors.setSpeeds(SPEED, SPEED);
      delay(300);
    // Don't do anything for straight
    default:
      // Do nothing for unrecognized commands
      break;
  }
}

// This function decides which way to turn at an intersection
char selectRandomTurn(bool found_left, bool found_straight, bool found_right)
{
  int available_directions = 0;
  char directions[3];
  
  if(found_left)
    directions[available_directions++] = 'L';
  if(found_straight)
    directions[available_directions++] = 'S';
  if(found_right)
    directions[available_directions++] = 'R';

  if(available_directions == 0)
    return 'B';  // Dead end, turn back

  // Choose a random direction from the available ones
  return directions[random(available_directions)];
}

void followSegment()
{
  unsigned int position;
  unsigned int sensors[6];
  int offset_from_center;
  int power_difference;

  while(1)
  {
    // Get the position of the line.
    position = reflectanceSensors.readLine(sensors);
    //Serial.println(position);
    // The (offset_from_center should be 0 when we are on the line.
    offset_from_center = ((int)position) - 2500;
    //Serial.println(offset_from_center);
    // Compute the difference between the two motor power settings,
    // m1 - m2.  If this is a positive number the robot will turn
    // to the left.  If it is a negative number, the robot will
    // turn to the right, and the magnitude of the number determines
    // the sharpness of the turn.
    power_difference = offset_from_center;

    // Compute the actual motor settings.  We never set either motor
    // to a negative value.
    if(power_difference > SPEED)
      power_difference = SPEED;
    if(power_difference < -SPEED)
      power_difference = -SPEED;

    if(power_difference < 0)
      motors.setSpeeds(SPEED + power_difference, SPEED - power_difference/2);
    else
      motors.setSpeeds(SPEED + power_difference/2, SPEED - power_difference);

    // We use the inner four sensors (1, 2, 3, and 4) for
    // determining whether there is a line straight ahead, and the
    // sensors 0 and 5 for detecting lines going to the left and
    // right.

    if(!ABOVE_LINE(sensors[0]) && !ABOVE_LINE(sensors[1]) && !ABOVE_LINE(sensors[2]) && !ABOVE_LINE(sensors[3]) && !ABOVE_LINE(sensors[4]) && !ABOVE_LINE(sensors[5]))
    {
      // There is no line visible ahead, and we didn't see any
      // intersection.  Must be a dead end.
      return;
    }
    else if(ABOVE_LINE(sensors[0]) || ABOVE_LINE(sensors[5]))
    {
      // Found an intersection.
      return;
    }
  }
}

void turnFunc()
{
  bool middle_sensor = false;
  bool found_straight = false;
  bool found_right = false;
  while(1)
  {
    unsigned int sensors[6];
    reflectanceSensors.readLine(sensors);
    /*Serial.print(sensors[0]);
    Serial.print(" ");
    Serial.print(sensors[1]);
    Serial.print(" ");
    Serial.print(sensors[2]);
    Serial.print(" ");
    Serial.print(sensors[3]);
    Serial.print(" ");
    Serial.print(sensors[4]);
    Serial.print(" ");
    Serial.print(sensors[5]);
    Serial.print(" ");
    Serial.println();*/
    // Check for straight exit.
    // reflectanceSensors.readLine(sensors);

    //Serial.println(middle_sensor);
    if( !ABOVE_LINE(sensors[2]) && !ABOVE_LINE(sensors[3]) )
      middle_sensor = true;

    if( (ABOVE_LINE(sensors[2]) || ABOVE_LINE(sensors[3])) && (middle_sensor == true) ){
      //motors.setSpeeds(0, 0);
      //button.waitForButton();
      break;
    }
      


  }
}

void backturnFunc()
{
  int middle_sensor = 0;
  bool found_straight = false;
  bool found_right = false;
  while(1)
  {
    unsigned int sensors[6];
    reflectanceSensors.readLine(sensors);
    /*Serial.print(sensors[0]);
    Serial.print(" ");
    Serial.print(sensors[1]);
    Serial.print(" ");
    Serial.print(sensors[2]);
    Serial.print(" ");
    Serial.print(sensors[3]);
    Serial.print(" ");
    Serial.print(sensors[4]);
    Serial.print(" ");
    Serial.print(sensors[5]);
    Serial.print(" ");
    Serial.println();*/
    // Check for straight exit.
    // reflectanceSensors.readLine(sensors);

    Serial.println(middle_sensor);
    if( !ABOVE_LINE(sensors[2]) && !ABOVE_LINE(sensors[3]) )
      middle_sensor = 1;

    else if( (ABOVE_LINE(sensors[2]) || ABOVE_LINE(sensors[3])) && (middle_sensor == 1) ){
      middle_sensor = 2;
      delay(300);
    }else if( (!ABOVE_LINE(sensors[2]) || !ABOVE_LINE(sensors[3])) && (middle_sensor == 2) ){
      middle_sensor = 3;
    
    }else if( (ABOVE_LINE(sensors[2]) || ABOVE_LINE(sensors[3])) && (middle_sensor == 3) ){
      
      motors.setSpeeds(0, 0);
      button.waitForButton();
      break;
    }
      


  }
}

void exploreRandomly()
{
  int avoidcnt = 0;
  while(1)
  {
    // Navigate current line segment
    avoidcnt++;
    followSegment();
    
    // These variables record whether the robot has seen a line to the
    // left, straight ahead, and right, while examining the current
    // intersection.
    bool found_left = false;
    bool found_straight = false;
    bool found_right = false;

    // Now read the sensors and check the intersection type.
    unsigned int sensors[6];
    reflectanceSensors.readLine(sensors);

    // Check for left and right exits.
    //if(ABOVE_LINE(sensors[0]))
    //  found_left = true;
    //if(ABOVE_LINE(sensors[5]))
    //  found_right = true;

    // Drive straight a bit more - this helps us in case we entered the
    // intersection at an angle.
    //motors.setSpeeds(SPEED, SPEED);
    //delay(OVERSHOOT(LINE_THICKNESS));

    // Check for straight exit.
    //reflectanceSensors.readLine(sensors);
    if(ABOVE_LINE(sensors[1]) || ABOVE_LINE(sensors[2]) || ABOVE_LINE(sensors[3]) || ABOVE_LINE(sensors[4]))
      found_straight = true;

    // Check again for left and right exits.
    if(ABOVE_LINE(sensors[0]) ){
      found_left = true;
    }
    if(ABOVE_LINE(sensors[5]) ){
      found_right = true;
    }


    if ((found_left == true || found_right == true) || (avoidflag == 1 && avoidcnt > 5000) ){
      motors.setSpeeds(0, 0);
      Serial.println("find");
      //button.waitForButton();
      break;
    }
    // Randomly select a direction to turn
    //char dir = selectRandomTurn(found_left, found_straight, found_right);

    // Make the turn



    //turn(dir);

    // Sound off buzzer to indicate a turn was made
    //buzzer.play(">g32");
  }
}
void clearWifiBuffer() {
  while(esp8266.available() > 0)
    esp8266.read();
}