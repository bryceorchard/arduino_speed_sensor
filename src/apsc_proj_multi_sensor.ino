
#include "SevSeg.h"
#include <math.h>
SevSeg sevseg; //Instantiate a seven segment controller object

#define HALL_L1 10
#define HALL_L2 11
#define HALL_R1 12
#define HALL_R2 13
#define TOP 10
#define WAITING_TOP 11
#define BOTTOM 20
#define WAITING_BOTTOM 21
#define M_PIF 3.141593e+00F
static const float angle = M_PIF/8.0;
static const float radius = 0.1;
static const float constant = (angle * radius * 3600000.0)/2.0;

static const float threshold = 2000000.0;

// Prototypes
void PlayTone(int frequency);
void FlashRed();
void WaitButtonClick();
void Display(int array[30], int round);
int CheckWrong(int array[30], int round);

void setup() {
  
  pinMode(HALL_L1, INPUT_PULLUP);
  pinMode(HALL_L2, INPUT_PULLUP);
  pinMode(HALL_R1, INPUT_PULLUP);
  pinMode(HALL_R2, INPUT_PULLUP);
  byte numDigits = 4;
  byte digitPins[] = {A3, A2, A1, A0};
  byte segmentPins[] = {3, 9, 7, 5, 2, 4, 8, 6};
  bool resistorsOnSegments = false; // 'false' means resistors are on digit pins
  byte hardwareConfig = COMMON_ANODE; // See README.md for options
  bool updateWithDelays = false; // Default 'false' is Recommended
  bool leadingZeros = false; // Use 'true' if you'd like to keep the leading zeros
  bool disableDecPoint = false; // Use 'true' if your decimal point doesn't exist or isn't connected
  
  sevseg.begin(hardwareConfig, numDigits, digitPins, segmentPins, resistorsOnSegments,
  updateWithDelays, leadingZeros, disableDecPoint);
  sevseg.setBrightness(90);
  sevseg.setNumber(0, -1);

  Serial.begin(9600); // Start Serial communication at 9600 baud

}

void loop() {
  int flag = 0;
  static unsigned long timer_L = 0, timer_R = 0;
  static unsigned long smoothing_timer = 50.0;
  static float deltaT_L = 0, deltaT_R = 0;   
  static float velocity = 0;
  static float smoothedVelocity = 0; // Smoothed velocity
  static const float alpha = 0.14;    // Smoothing factor (0.0 < alpha <= 1.0)
  static int status_L = -1, status_R = -1;
  int sensor_L_TOP = digitalRead(HALL_L1), sensor_L_BOTTOM = digitalRead(HALL_L2);
  int sensor_R_TOP = digitalRead(HALL_R1), sensor_R_BOTTOM = digitalRead(HALL_R2);


  // Sensor L
  if (sensor_L_TOP != sensor_L_BOTTOM) {
    if ((status_L == WAITING_TOP && sensor_L_TOP == LOW) || (status_L == WAITING_BOTTOM && sensor_L_BOTTOM == LOW)) {
      deltaT_L = 1.0/(micros() - timer_L);
      flag = 1;
    } 
    if (sensor_L_TOP == LOW)
        status_L = BOTTOM;
    if (sensor_L_BOTTOM == LOW)
      status_L = TOP;
    timer_L = micros();
    
  } else if (sensor_L_TOP == sensor_L_BOTTOM) {
    if (status_L == TOP)
      status_L = WAITING_TOP;
    if (status_L == BOTTOM)
      status_L = WAITING_BOTTOM;
  }

  // Sensor R
  if (sensor_R_TOP != sensor_R_BOTTOM) {
    if ((status_R == WAITING_TOP && sensor_R_TOP == LOW) || (status_R == WAITING_BOTTOM && sensor_R_BOTTOM == LOW)) {
      deltaT_R = 1.0/(micros() - timer_R);
      flag = 1;
    } 
    if (sensor_R_TOP == LOW)
        status_R = BOTTOM;
    if (sensor_R_BOTTOM == LOW)
      status_R = TOP;
    timer_R = micros();
    
  } else if (sensor_R_TOP == sensor_R_BOTTOM) {
    if (status_R == TOP)
      status_R = WAITING_TOP;
    if (status_R == BOTTOM)
      status_R = WAITING_BOTTOM;
  }

  if (flag == 1) {
    velocity = constant * (deltaT_L + deltaT_R);
    smoothedVelocity = alpha * velocity + (1 - alpha) * smoothedVelocity;
    Serial.println(smoothedVelocity);
    sevseg.setNumber(smoothedVelocity, -1);
  }
  if (smoothedVelocity < 0.5){
      sevseg.setNumber(0, -1);
      sevseg.refreshDisplay();
      return;

  // if (((millis() - smoothing_timer) > 50.0) && ((micros() - timer_L) > threshold) && ((micros() - timer_R) > threshold)) {
  //   smoothedVelocity *= (threshold/(threshold + sqrt(micros() - ((timer_L + timer_R)/2.0) - threshold)));
    // Essentially the threshold divided by the threshold plus the sqrt of the excess time
    // Say the threshold is 100ms, and the timer is at 120ms, it looks like V * 100/(100 + sqrt(20))'
    // smoothing_timer = millis();
  }
  sevseg.refreshDisplay(); // Must run repeatedly
}