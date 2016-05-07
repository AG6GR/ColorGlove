#include <Wire.h>
#include <Stepper.h>
#include <SparkFun_APDS9960.h>

#define PIN_BUTTON 0
#define PIN_BUTTONGND 1

#define PIN_LED_RED 3
#define PIN_LED_GREEN 4
#define PIN_LED_BLUE 5

#define PIN_S_SLIDER_IN1 7
#define PIN_S_SLIDER_IN2 6
#define PIN_S_SLIDER_PWM 8
#define ADC_S_SLIDER 0

#define PIN_V_SLIDER_IN1 11
#define PIN_V_SLIDER_IN2 10
#define PIN_V_SLIDER_PWM 9
#define ADC_V_SLIDER 1

#define ADC_H_RING 2

// Color sensor I2C: SDA pin 18, SCL pin 19

#define STEPPER_STEPS 513
#define PIN_STEPPER_A1 20
#define PIN_STEPPER_A2 21
#define PIN_STEPPER_B1 22
#define PIN_STEPPER_B2 23

Stepper stepper = Stepper(STEPPER_STEPS, PIN_STEPPER_A1, PIN_STEPPER_A2,
    PIN_STEPPER_B1, PIN_STEPPER_B2); // Stepper motor object

// Deadband power level for sliders
#define MOTOR_DEADBAND 128

int stepperPosition = 0; // Position of the stepper (in steps)
int stepperTarget = 0; // Target position of the motor
int sensorValH = 0; // store the value coming from the sensor for H
int sensorValS = 0; // store the value coming from the sensor for S
int sensorValB = 0; // store the value coming from the sensor for B

// getRGB function stores RGB values in this array
// use these values for the red, blue, green led.
int rgb_colors[3];

int hue;
int saturation;
int brightness;

// Raw values from color sensor
SparkFun_APDS9960 apds = SparkFun_APDS9960();
uint16_t ambient_light = 0;
uint16_t red_light = 0;
uint16_t green_light = 0;
uint16_t blue_light = 0;

float slider_Kp = 0.5;
bool isActive = true;

void setup() {
    Wire.begin();
    Serial.begin(57600);

    // Button
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    pinMode(PIN_BUTTONGND, OUTPUT);
    digitalWrite(PIN_BUTTONGND, LOW);

    // LED output pins
    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PIN_LED_BLUE, OUTPUT);
    analogWrite(PIN_LED_RED, 0);
    analogWrite(PIN_LED_GREEN, 0);
    analogWrite(PIN_LED_BLUE, 0);

    // Saturation slider pins
    pinMode(PIN_S_SLIDER_IN1, OUTPUT);
    pinMode(PIN_S_SLIDER_IN2, OUTPUT);
    pinMode(PIN_S_SLIDER_PWM, OUTPUT);
    digitalWrite(PIN_S_SLIDER_IN1, LOW);
    digitalWrite(PIN_S_SLIDER_IN2, LOW);
    analogWrite(PIN_S_SLIDER_PWM, 0);

    // Value slider pins
    pinMode(PIN_V_SLIDER_IN1, OUTPUT);
    pinMode(PIN_V_SLIDER_IN2, OUTPUT);
    pinMode(PIN_V_SLIDER_PWM, OUTPUT);
    digitalWrite(PIN_V_SLIDER_IN1, LOW);
    digitalWrite(PIN_V_SLIDER_IN2, LOW);
    analogWrite(PIN_V_SLIDER_PWM, 0);

    // Stepper setup
    stepper.setSpeed(60);

    // Initialize APDS-9960 (configure I2C and initial values)
    if ( apds.init() ) {
        Serial.println(F("APDS-9960 initialization complete"));
    } else {
        Serial.println(F("Something went wrong during APDS-9960 init!"));
    }

    // Start running the APDS-9960 light sensor (no interrupts)
    if ( apds.enableLightSensor(false) ) {
        Serial.println(F("Light sensor is now running"));
    } else {
        Serial.println(F("Something went wrong during light sensor init!"));
    }

    //Testing
    saturation = 128;
    brightness = 128;
    hue = 180;
}

void loop() {
    // Get HSV values
    sensorValH = analogRead(ADC_H_RING);
    sensorValS = analogRead(ADC_S_SLIDER);
    sensorValB = analogRead(ADC_V_SLIDER);

    //hue = (millis()/10) % 359;
    //saturation = (millis()/10) % 255;
    //brightness = (millis()/10) % 255;
    //Serial.println("Hue: " + String(sensorValH) + " Saturation: " + String(sensorValS) + " Brightness: " + String(sensorValB) + " isActive: " + String(isActive));

    // Get current mode from button
    if(digitalRead(PIN_BUTTON) == LOW)
    {
        isActive = false;
    }
    else
    {
        isActive = true;
    }

    if (isActive == true) // Actively display sensed color
    {
        // Get color from color sensor, save in hue,saturation,value variables
        if (  !apds.readAmbientLight(ambient_light) ||
            !apds.readRedLight(red_light) ||
        !apds.readGreenLight(green_light) ||
        !apds.readBlueLight(blue_light) )
        {
            Serial.println("Error reading light values");
        }
        else
        {
            /*
            Serial.print("Ambient: ");
            Serial.print(ambient_light);
            Serial.print(" Red: ");
            Serial.print(red_light);
            Serial.print(" Green: ");
            Serial.print(green_light);
            Serial.print(" Blue: ");
            Serial.println(blue_light);
            */

            RGBtoHSV(map(red_light, 0,ambient_light,0,255),
                map(green_light, 0,ambient_light,0,255),
                map(blue_light, 0,ambient_light,0,255));
            //Serial.println("Hue: " + String(hue) + " Saturation: " + String(saturation) + " Brightness: " + String(brightness));

            //brightness = 255;
        }

        // Set sliders
        setSlider(PIN_S_SLIDER_IN1, PIN_S_SLIDER_IN2, PIN_S_SLIDER_PWM,
            (int)((map(saturation, 0, 255, 0, 1023) - sensorValS) * slider_Kp));
        setSlider(PIN_V_SLIDER_IN1, PIN_V_SLIDER_IN2, PIN_V_SLIDER_PWM,
            (int)((map(brightness, 0, 255, 0, 1023) - sensorValB) * slider_Kp));

        // Set dial
        stepperTarget = map(hue, 0, 359, 0, STEPPER_STEPS);
        if (stepperPosition < stepperTarget)
        {
            stepper.step(min(3, stepperTarget - stepperPosition));
            stepperPosition += min(3, stepperTarget - stepperPosition);
        }
        else
        {
            stepper.step(max(-3, stepperTarget - stepperPosition));
            stepperPosition += max(-3, stepperTarget - stepperPosition);
        }
        saturation = 255;


    }
    else // Let user set color
    {

        if (sensorValH > 400)
        {
            hue = map(sensorValH, 400, 1024, 0, 360);
        }
        saturation = map(sensorValS, 0, 1024, 0, 255);
        brightness = map(sensorValB, 0, 1024, 0, 255);
        Serial.println("Hue: " + String(hue) + " Saturation: " + String(saturation) + " Brightness: " + String(brightness));

        // Disable slider motors
        setSlider(PIN_S_SLIDER_IN1, PIN_S_SLIDER_IN2, PIN_S_SLIDER_PWM, 0);
        setSlider(PIN_V_SLIDER_IN1, PIN_V_SLIDER_IN2, PIN_V_SLIDER_PWM, 0);
    }

    // Set color of leds
    HSVtoRGB(hue, brightness, saturation);
    analogWrite(PIN_LED_RED, 255 - rgb_colors[0]);
    analogWrite(PIN_LED_GREEN, 255 - rgb_colors[1]);
    analogWrite(PIN_LED_BLUE, 255 - rgb_colors[2]);
}

/*
* Set a motor to a given output value between 0 and 100. Positive values indicate
* forward direction, negative values indicate backward direction.
*/
void setSlider(int pin_1, int pin_2, int pin_pwm, int value)
{
    int pwmlevel = abs(value);
    //Serial.println("Setting: " + String(pin_1) + "," + String(pin_2) + "," + String(value));
    // Set controller direction
    if (pwmlevel < 2)
    {
        // Disengage if pwmlevel is very small
        pwmlevel = 0;
        digitalWrite(pin_1, LOW);
        digitalWrite(pin_2, LOW);
        analogWrite(pin_pwm, 0);
        return;
    }
    else if (value < 0)
    {
        // Backward
        digitalWrite(pin_2, LOW);
        digitalWrite(pin_1, HIGH);
    }
    else
    {
        // Forward
        digitalWrite(pin_1, LOW);
        digitalWrite(pin_2, HIGH);
    }

    // Check bounds on pwmlevel
    if (pwmlevel > 100)
    {
        pwmlevel = 100;
        analogWrite(pin_pwm, 255);
    }
    else
    {
        // Write new PWM value
        analogWrite(pin_pwm, map(pwmlevel, 0, 100, MOTOR_DEADBAND, 255));
    }
}

// Color conversion code from https://www.cs.rit.edu/~ncs/color/t_convert.html#RGB to HSV & HSV to RGB

void HSVtoRGB(int h_val, int s_val, int v_val)
{
    int i;
	float h,s,v,f, p, q, t, r, g, b;
	h = (float) h_val;
	s = (float) s_val / 255.0;
	v = (float) v_val / 255.0;
	if( s == 0 ) {
		// achromatic (grey)
		r = g = b = v;
		return;
	}
	h /= 60;			// sector 0 to 5
	i = floor( h );
	f = h - i;			// factorial part of h
	p = v * ( 1 - s );
	q = v * ( 1 - s * f );
	t = v * ( 1 - s * ( 1 - f ) );
	switch( i ) {
	case 0:
	    r = v;
	    g = t;
	    b = p;
	    break;
	case 1:
	    r = q;
	    g = v;
	    b = p;
	    break;
	case 2:
	    r = p;
	    g = v;
	    b = t;
	    break;
	case 3:
	    r = p;
	    g = q;
	    b = v;
	    break;
	case 4:
	    r = t;
	    g = p;
	    b = v;
	    break;
	default:		// case 5:
	    r = v;
	    g = p;
	    b = q;
	    break;
	}
	rgb_colors[0] = (int)(255 * r);
	rgb_colors[1] = (int)(255 * g);
	rgb_colors[2] = (int)(255 * b);
}

void RGBtoHSV(int r_val, int g_val, int b_val)
{
	float r,g,b,h,s,v;
    float min, max, delta;

    r = r_val / 255.0;
    g = g_val / 255.0;
    b = b_val / 255.0;

	min = min(min(r, g), b);
	max = max(max(r, g), b);
	v = max;				// v
	brightness = (int) (v * 255);
	delta = max - min;
	if( max != 0 )
		s = delta / max;		// s
	else {
		// r = g = b = 0		// s = 0, v is undefined
		saturation = 0;
		hue = 0;
		return;
	}
	if( r == max )
		h = ( g - b ) / delta;		// between yellow & magenta
	else if( g == max )
		h = 2 + ( b - r ) / delta;	// between cyan & yellow
	else
		h = 4 + ( r - g ) / delta;	// between magenta & cyan
	h *= 60;				// degrees
    h += 180;
	if( h < 0 )
		h += 360;
	hue = ((int)h) % 360;
	saturation = (int) (s * 255);
}