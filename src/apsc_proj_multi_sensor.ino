#include "SevSeg.h"
#include <math.h>
SevSeg sevseg; //Instantiate a seven segment controller object

#define HALL_L1 10
#define HALL_L2 11
#define HALL_R1 12
#define HALL_R2 13
#define TOP_NEXT 10
#define WAITING_TOP 11
#define BOTTOM_NEXT 20
#define WAITING_BOTTOM 21
#define M_PIF 3.141593e+00F
static const float angle = M_PIF/8.0;
static const float radius = 0.1;
static const float constant = (angle * radius * 3600000.0)/2.0;

static const float threshold = 2000000.0;

void updateWheel(
    int sensorTop,
    int sensorBottom,
    int& status,
    float& deltaT,
    unsigned long& timer,
    int& flag
);

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

void updateWheel(
    int sensorTop,
    int sensorBottom,
    int& status,
    float& deltaT,
    unsigned long& timer,
    int& flag
) {
    // Sensors differ -> currently over a magnet
    if (sensorTop != sensorBottom) {
        bool detectedExpectedTop = (status == WAITING_TOP && sensorTop == LOW);
        bool detectedExpectedBottom = (status == WAITING_BOTTOM && sensorBottom == LOW);

        // Valid magnet transition detected
        if (detectedExpectedTop || detectedExpectedBottom) {
            unsigned long now = micros();
            // Frequency proportional to wheel speed
            deltaT = 1.0 / (now - timer);
            timer = now;
            // Signal that a new velocity reading is available
            flag = 1;
        }
        // Determine which sensor triggered, and therefore which sensor we expect next
        if (sensorTop == LOW) status = BOTTOM_NEXT;
        if (sensorBottom == LOW) status = TOP_NEXT;
    }
    // Sensors equal -> in between magnets, so arm the next valid transition
    else {
        if (status == TOP_NEXT) status = WAITING_TOP;
        if (status == BOTTOM_NEXT) status = WAITING_BOTTOM;
    }
}

void loop() {
  int flag = 0;
  static unsigned long timer_L = 0, timer_R = 0;
  static unsigned long smoothing_timer = 0;
  static float deltaT_L = 0, deltaT_R = 0;   
  static float velocity = 0;
  static float smoothedVelocity = 0; // Smoothed velocity
  static const float alpha = 0.14;    // Smoothing factor (0.0 < alpha <= 1.0)
  static int status_L = -1, status_R = -1;
  
  int sensor_L_TOP = digitalRead(HALL_L1), sensor_L_BOTTOM = digitalRead(HALL_L2);
  int sensor_R_TOP = digitalRead(HALL_R1), sensor_R_BOTTOM = digitalRead(HALL_R2);

  updateWheel(
    sensor_L_TOP,
    sensor_L_BOTTOM,
    status_L,
    deltaT_L,
    timer_L,
    flag
  );

  updateWheel(
    sensor_R_TOP,
    sensor_R_BOTTOM,
    status_R,
    deltaT_R,
    timer_R,
    flag
  );


  unsigned long nowMicros = micros();
  unsigned long nowMillis = millis();

  if (flag == 1) {
    velocity = constant * (deltaT_L + deltaT_R);
    smoothedVelocity = (alpha * velocity) + ((1 - alpha) * smoothedVelocity);
  }

  // Apply a non-linear decay to the smoothed velocity if no new readings have been received for a certain time, to prevent it from staying artificially high
  else if (
    (smoothedVelocity > 0) &&
    ((nowMillis - smoothing_timer) > 50) && // Only apply smoothing every 50ms
    ((nowMicros - timer_L) > threshold) && ((nowMicros - timer_R) > threshold) // Only apply smoothing if both timers have exceeded the threshold
  ) {
    float averageTimer = (timer_L + timer_R) / 2.0;
    float excessTime = nowMicros - averageTimer - threshold;
    smoothedVelocity *= (threshold/(threshold + sqrt(excessTime)));
    // This is essentially the threshold divided by the threshold plus the sqrt of the excess time
    // Say the threshold is 100ms, and the timer is at 120ms, it looks like V * 100/(100 + sqrt(20))
    smoothing_timer = nowMillis;
  }

  if (smoothedVelocity < 0.5) {
    smoothedVelocity = 0;
  }
  sevseg.setNumber(smoothedVelocity, -1);
  sevseg.refreshDisplay(); // Must run repeatedly
}