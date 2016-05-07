#define PIN_MOTORA1 7
#define PIN_MOTORA2 8
#define PIN_MOTORAPWM 9
#define ADC_SLIDER_IN 0
#define MOTOR_DEADBAND 128

int sliderVal = 0;
int setpoint = 512;
float slider_Kp = 0.5;

void setup() {
  pinMode(PIN_MOTORA1, OUTPUT);
  pinMode(PIN_MOTORA2, OUTPUT);
  pinMode(PIN_MOTORAPWM, OUTPUT);

  digitalWrite(PIN_MOTORA1, LOW);
  digitalWrite(PIN_MOTORA2, LOW);
  analogWrite(PIN_MOTORAPWM, 0);

  Serial.begin(57600);
}

void loop() {
  // put your main code here, to run repeatedly:
  sliderVal = analogRead(0);
  Serial.println("Target: " + String(setpoint) + " Current: " + String(sliderVal));

  setMotor((int) ((setpoint - sliderVal) * slider_Kp));

  setpoint = (int) millis()/5 % 1000 + 5;
}

/*
 * Set a motor to a given output value between 0 and 100. Positive values indicate
 * forward direction, negative values indicate backward direction.
 */
void setMotor(int value)
{
  //Serial.println("Setting Value: " + String(value));
  int pwmlevel = abs(value);
  // Set controller direction
  if (pwmlevel < 1)
  {
    // Disengage if pwmlevel is very small
    pwmlevel = 0;
    digitalWrite(PIN_MOTORA1, LOW);
    digitalWrite(PIN_MOTORA2, LOW);
    analogWrite(PIN_MOTORAPWM, 0);
    return;
  }
  else if (value < 0)
  {
    // Backward
    digitalWrite(PIN_MOTORA2, LOW);
    digitalWrite(PIN_MOTORA1, HIGH);
  }
  else
  {
    // Forward
    digitalWrite(PIN_MOTORA1, LOW);
    digitalWrite(PIN_MOTORA2, HIGH);
  }

  // Check bounds on pwmlevel
  if (pwmlevel > 100)
  {
    pwmlevel = 100;
    analogWrite(PIN_MOTORAPWM, 255);
  }
  else
  {
    // Write new PWM value
    analogWrite(PIN_MOTORAPWM, map(pwmlevel, 0, 100, MOTOR_DEADBAND, 255));
  }
}

