#define CALIBRATION_PWM 32
#define CALIBRATION_SAMPLE_TIME 200

// Pitch Motordriver pins
#define PITCH_EN 11
#define PITCH_R_PWM 6
#define PITCH_L_PWM 13

// Roll Motordriver pins
#define ROLL_EN 8
#define ROLL_R_PWM 9
#define ROLL_L_PWM 10

// multiplexer for buttons
/* #define MUX_S0 A0 */
/* #define MUX_S1 A1 */
/* #define MUX_S2 A2 */
/* #define MUX_S3 A3 */

// Multiplexer Yoke Buttons
/* #define MUX_EN_YOKE 5 */
/* #define MUX_SIGNAL_YOKE A5 */

// Multiplexer Adjustemts for Calib Button, Force Potis, End Switches

// Adjustments array positions
#define ADJ_ENDSWITCH_PITCH_DOWN 18
#define ADJ_ENDSWITCH_PITCH_UP 19
#define ADJ_ENDSWITCH_ROLL_LEFT 20 
#define ADJ_ENDSWITCH_ROLL_RIGHT 21

// If you use center sensor use 
// RES_1 connector for Roll Roll Sensor
// RES_2 connector for Pitch Sensor
/* #if defined(LANGUAGE_EN) */
/*   #define ADJ_MIDDLESWITCH_ROLL 5 */
/*   #define ADJ_MIDDLESWITCH_PITCH 6 */
/* #endif */


/***************
  Pin setup
****************/
void ArduinoSetup()
{

  // Pitch Pins
  pinMode(PITCH_EN, OUTPUT);
  pinMode(PITCH_R_PWM, OUTPUT);
  pinMode(PITCH_L_PWM, OUTPUT);

  // Roll Pins
  pinMode(ROLL_EN, OUTPUT);
  pinMode(ROLL_R_PWM, OUTPUT);
  pinMode(ROLL_L_PWM, OUTPUT);

  pinMode(ADJ_ENDSWITCH_PITCH_DOWN, INPUT);
  pinMode(ADJ_ENDSWITCH_PITCH_UP, INPUT);
  pinMode(ADJ_ENDSWITCH_ROLL_LEFT, INPUT);
  pinMode(ADJ_ENDSWITCH_ROLL_RIGHT, INPUT);


  // define pin default states
  // Pitch
  digitalWrite(PITCH_EN, LOW);
  digitalWrite(PITCH_R_PWM, LOW);
  digitalWrite(PITCH_L_PWM, LOW);
  //Roll
  digitalWrite(ROLL_EN, LOW);
  digitalWrite(ROLL_R_PWM, LOW);
  digitalWrite(ROLL_L_PWM, LOW);

  // not for all Arduinos!
  // This sets the PWM Speed to maximun for noise reduction

  // Timer1: pins 9 & 10
  TCCR1B = _BV(CS10); // change the PWM frequencey to 31.25kHz - pins 9 & 10

  // Timer4: pin 13 & 6
  TCCR4B = _BV(CS40); // change the PWM frequencey to 31.25kHz - pin 13 & 6
} //ArduinoSetup

/**************************
  Enables the motordrivers
****************************/
void EnableMotors() {
  digitalWrite(PITCH_EN, HIGH);
  digitalWrite(ROLL_EN, HIGH);
} //EnableMotors

/***************************
  Disables the motordrivers
****************************/
void DisableMotors() {
  digitalWrite(PITCH_EN, LOW);
  digitalWrite(ROLL_EN, LOW);

  analogWrite(ROLL_L_PWM, 0);  // stop left
  analogWrite(ROLL_R_PWM, 0);  // stop right

  analogWrite(PITCH_L_PWM, 0);  // stop left
  analogWrite(PITCH_R_PWM, 0);  // stop right
} //DisableMotors

/******************************************************
  calculates the motor speeds and controls the motors
******************************************************/
void DriveMotors() {
  DriveMotor(PITCH_R_PWM,
             PITCH_L_PWM,
             forces[MEM_PITCH] < 0,
             ForceToPwm(forces[MEM_PITCH],
                        adjForceMax[MEM_PITCH],
                        adjPwmMin[MEM_PITCH],
                        adjPwmMax[MEM_PITCH])
             );
  DriveMotor(ROLL_L_PWM,
             ROLL_R_PWM,
             forces[MEM_ROLL] < 0,
             ForceToPwm(forces[MEM_ROLL],
                        adjForceMax[MEM_ROLL],
                        adjPwmMin[MEM_ROLL],
                        adjPwmMax[MEM_ROLL])
             );
}


int ForceToPwm(int16_t gForce, int forceMax, byte pwmMin, byte pwmMax)
{
  int pForce = constrain(abs(gForce), 0, forceMax);

  if (pForce == 0) {
    return 0;
  }

  return map(pForce, 0, forceMax, pwmMin, pwmMax);
} //ForceToPwm

/******************************************************
  calculates the motor speeds
******************************************************/
int DriveMotor(byte pinLPWM,
                byte pinRPWM,
                bool leftDirection,
                int pwm)
{
  Serial.print("Setting force to: ");
  Serial.println(pwm);
  // which direction?
  if (leftDirection) {
    analogWrite(pinRPWM, 0);
    analogWrite(pinLPWM, pwm);
  }
  else {
    analogWrite(pinLPWM, 0);
    analogWrite(pinRPWM, pwm);
  }
} //CalculateSpeed


void CalibrateAxisOneDirection(byte pinLPWM,
                                byte pinRPWM,
                                bool leftDirection,
                                int &endstopValue,
                                byte pinEndstop,
                                Encoder &counter
                                )
{
  EnableMotors();
  DriveMotor(pinLPWM, pinRPWM, leftDirection, CALIBRATION_PWM);
  unsigned long currentMillis;
  unsigned long nextSampleMillis = millis() + 500; // some time to let it start the motion
  int lastCounterValue = -32768;

  while (endstopValue == -32768) {
    currentMillis = millis();

    if (currentMillis >= nextSampleMillis) {
      nextSampleMillis = currentMillis + CALIBRATION_SAMPLE_TIME;
      if (lastCounterValue == counter.read()) {
        endstopValue = counter.read();
        DisableMotors();
      }

      lastCounterValue = counter.read();
    }
  }
}

/******************************************************
  Calibration
******************************************************/
void CheckCalibrationMode() {
  if (blStart == false) {
    return;
  }

  blStart = false;

  int poti_roll_min = -32768;                             //counter min for Init;
  int poti_roll_max = -32768;                             //counter max for Init;
  int poti_pitch_min = -32768;                             //counter min for Init;
  int poti_pitch_max = -32768;                            //counter max for Init;

  DisableMotors();


  CalibrateAxisOneDirection(PITCH_L_PWM, PITCH_R_PWM, true, poti_pitch_min, ADJ_ENDSWITCH_PITCH_DOWN, counterPitch);
  CalibrateAxisOneDirection(PITCH_L_PWM, PITCH_R_PWM, false, poti_pitch_max, ADJ_ENDSWITCH_PITCH_UP, counterPitch);
  CalibrateAxisOneDirection(ROLL_L_PWM, ROLL_R_PWM, true, poti_roll_min, ADJ_ENDSWITCH_ROLL_LEFT, counterRoll);
  CalibrateAxisOneDirection(ROLL_L_PWM, ROLL_R_PWM, false, poti_roll_max, ADJ_ENDSWITCH_ROLL_RIGHT, counterRoll);

  JOYSTICK_minY = poti_pitch_min;
  JOYSTICK_maxY = poti_pitch_max;
  JOYSTICK_minX = poti_roll_min;
  JOYSTICK_maxX = poti_roll_max;

  setRangeJoystick();
  EnableMotors();

}

/******************************************
   Reads the button states over multiplexer
*******************************************/
/*
void ReadMux() {
  iYokeButtonPinStates = 0;
  iSensorPinStates = 0;

  // for every 16 imput lines of a multiplexer
  for (byte x = 0; x < 16; x++) {
    // direct port manipulation is faster than digitalwrite!
    // s0 = Pin A0 - PF7
    if (x & B00000001) // if bit 0 of counter is high
    {
      PORTF = PORTF | B10000000;  // set bit 7 of port (A0)
    } else {
      PORTF = PORTF & B01111111;  // clear bit 7 of port (A0)
    }

    // s1 =  Pin A1 - PF6
    if (x & B00000010)  // if bit 1 of counter is high
    {
      PORTF = PORTF | B01000000;  // set bit 6 of port (A1)
    } else {
      PORTF = PORTF & B10111111;  // clear bit 6 of port (A1)
    }

    // s2 = Pin A2 PF5
    if (x & B00000100) // if bit 2 of counter is high
    {
      PORTF = PORTF | B00100000;  // set bit 5 of port (A2)
    } else {
      PORTF = PORTF & B11011111; // clear bit 5 of port (A2)
    }

    // s3 = Pin A3 - PF4
    if (x & B00001000) // if bit 3 of counter is high
    {
      PORTF = PORTF | B00010000; // set bit 4 of port (A3)
    } else {
      PORTF = PORTF & B11101111; // clear bit 4 of port (A3)
    }

    // enable mux 1
    PORTC = PORTC & B10111111; // Digital Pin 5 - PortC6

    // wait for capacitors of mux to react
    delayMicroseconds(1);

    // read value
    if (x == 0)
    {
      iYokeButtonPinStates = digitalRead(MUX_SIGNAL_YOKE) << x;
    } else {
      iYokeButtonPinStates |= digitalRead(MUX_SIGNAL_YOKE) << x;
    }

    // disable mux1
    PORTC = PORTC | B01000000; // Digital Pin 5 - PortC6

    // enable mux 2
    PORTD = PORTD & B11101111; // Digital Pin 4 - PortD4

    // wait for capacitors of mux to react
    delayMicroseconds(1);

    // read value
    iSensorPinStates |= digitalRead(MUX_SIGNAL_INPUT) << x;

    if (x == 0)
    {
      iSensorPinStates = digitalRead(MUX_SIGNAL_INPUT) << x;
    } else {
      iSensorPinStates |= digitalRead(MUX_SIGNAL_INPUT) << x;
    }

    // disblae mux 2
    PORTD = PORTD | B00010000; // Digital Pin 4 - PortD4

  }//for

}// ReadMux

*/
/******************************************
   set button states to Joystick library
*******************************************/
/*
void UpdateJoystickButtons() {
  // detect hat switch position
  if ( iYokeButtonPinStates << 12 == 0B0000000000000000)
    Joystick.setHatSwitch(0, -1);   // no direction

  if ( iYokeButtonPinStates << 12 == 0B0100000000000000)
    Joystick.setHatSwitch(0, 0);    // up

  if ( iYokeButtonPinStates << 12 == 0B0101000000000000)
    Joystick.setHatSwitch(0, 45);   // up right

  if ( iYokeButtonPinStates << 12 == 0B0001000000000000)
    Joystick.setHatSwitch(0, 90);   // right

  if ( iYokeButtonPinStates << 12 == 0B0011000000000000)
    Joystick.setHatSwitch(0, 135);  // down right

  if ( iYokeButtonPinStates << 12 == 0B0010000000000000)
    Joystick.setHatSwitch(0, 180);  // down

  if ( iYokeButtonPinStates << 12 == 0B1010000000000000)
    Joystick.setHatSwitch(0, 225);   // down left

  if ( iYokeButtonPinStates << 12 == 0B1000000000000000)
    Joystick.setHatSwitch(0, 270);  // left

  if ( iYokeButtonPinStates << 12 == 0B1100000000000000)
    Joystick.setHatSwitch(0, 315);  // up left

  // read button states from multiplexer
  for (byte channel = 4; channel < 16; channel++)
  {
    Joystick.setButton(channel - 4, (iYokeButtonPinStates & (1 << channel)) >> channel);
  }
} //updateJoystickButtons
*/
