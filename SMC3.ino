/*
 *  3DOF Motion Platform PID Control Loop
 *  Joshua Martin (skizzles@gmail.com)
 *
 *  This application provides support for any compatible clone of the Adafruid Motorshield v2
 *  to be used either as is, or by having outputs being ran through a low
 *  pass filter with externally supplied 10vDC for handling isolated -/+10v
 *  industrial motor controllers among other control systems.  A PCB motor shield with
 *  this 10v regulation and filtering is available at http://github.com/skiz/mcu
 *
 *  This library is heavily influenced by the SMC3 PID driver, including the
 *  command set, general logic, and algorithms.  However, the code has been
 *  extensively rewritten to work seamlessly with with V2 motor shields and
 *  to remove excessive code duplication.
 *
 *  Note: The Adafruit Motorshield V2 library is required for compilation.
 *
 *  TODO: 
 *  - Need to set up loop to properly time PID
 *  - Validate PID calculations work as expected
 *  - Figure out if we need eeprom storage and autoloading
 *  
 */

//    COMMAND SET:
//    []                  Drive all motors to their predefined stop positions (park)
//    [Axx],[Bxx],[Cxx]   Send position updates for motor 1,2,3 where xx is the binary position limitted to range 0-1024
//    [Dxx],[Exx],[Fxx]   Send the Kp parameter for motor 1,2,3 where xx is the Kp value multiplied by 100
//    [Gxx],[Hxx],[Ixx]   Send the Ki parameter for motor 1,2,3 where xx is the Ki value multiplied by 100
//    [Jxx],[Kxx],[Lxx]   Send the Kd parameter for motor 1,2,3 where xx is the Kd value multiplied by 100
//    [Mxx],[Nxx],[Oxx]   Send the Ks parameter for motor 1,2,3 where xx is the Ks value
//    [Pxy],[Qxy],[Rxy]   Send the speedMin and speedMax values where x is the speedMin and y is speedMax each being in range 0-255
//    [Sxy],[Txy],[Uxy]   Send the cutoff and clip minimum values where x is the cutoff, and y is the clip (applied as 1023 - min as max for both)
//    [Vxy],[Wxy],[Xxy]   Send the feedback dead zone (x) and the backup speed (y) for each motor
//    [rdx]               Read a value from the controller where x is the command code (A-Z) for the parameter to read    
//    [rd~]               Read PWM frequency for motors 1 and (2,3)
//    [ena]               Enable all motors
//    [enX]               Enable a specific motor where X is the motor number (1,2,3)
//    [dis]               Disable all motors
//    [diX]               Disable a specific motor where X is the motor number (1,2,3)
//    [moX]               Monitor a specific monitor on the serial line where X is 1,2,3 (or 0 for none)
//    [ver]               Send the software version
//

#include <Wire.h>
#include <Adafruit_MotorShield.h>

#define MCU_ADDRESS 0x40   // I2C Address
#define MCU_FREQUENCY 8000 // PWM Frequency (Hz). Up to 25MHz supported by PWM chip internal clock.
#define VERSION 70         // Firmware Version
#define START_BYTE '['     // First command/response byte
#define END_BYTE ']'       // Last command/response byte
#define ERR_RESP "!!!"     // Error response string

typedef struct {
  Adafruit_DCMotor *motor; // Adafruit DC motor control instance
  int feedback;            // current position of motor feedback device
  int target;              // Target motor position (0-1023)
  int deadzone;            // Feedback deadzone to ignore around center position
  int cutoffMin;           // The position beyond which the motors are disabled
  int cutoffMax;           // The position beyond which the motors are disabled
  int clipMin;             // The position which the motors will stop if reached
  int clipMax;             // The position which the mostor will stop if reached
  int speed;               // The speed which the motor should be running (0-255)
  int speedMin;            // The minimum speed which the motor should run (0-255)
  int speedMax;            // The maximum speed which the motor should limit (0-255)
  int direction;           // intended direction of motor FORWARD or REVERSE
  int backupSpeed;         // The speed which should be used to reverse if we overshoot (0-255)
  int feedbackPin;         // analog input pin for positioning feedback
  bool enabled;            // motor has been authorized to move
  int stopPosition;        // target position for axis parking 
  int lshift;              // axis specific modifier for PID delta position (try 5)
  int rshift;              // axis specific modifier for PID cumulative error (try 6)
  int kp_x100;             // The proportional term of the PID loop. Larger value drives harder  (0-1000)
  int ki_x100;             // The integral term to steady state. set to 0, large values may overshoot and oscillate
  int kd_x100;             // The derivative term reduces overshoot. Slows movement under large steps
  int ks;                  // The smoothing parameter. Limits kd to only update every N runs
} Axis;

Axis a1, a2, a3;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(MCU_ADDRESS);
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
Adafruit_DCMotor *motor3 = AFMS.getMotor(3);

unsigned int rxBuffer[3]={0};  // 5 byte command buffer "XXX\r\n"
unsigned int rxByte={0};       // current command byte
unsigned int bufferEnd={-1};   // index of last buffer byte
unsigned int serialFeedback=0; // axis to give feedback on over com port
byte powerScale = 7;           // used to divide PID result changes when in low power mode

void setup() {
  initAxis(&a1, motor1, A0); // Initialize Axis 1
  initAxis(&a2, motor2, A1); // Initialize Axis 2
  initAxis(&a3, motor3, A2); // Initialize Axis 3
  
  Serial.begin(500000);

  AFMS.begin();
}

void loop() {
    processFeedback(&a1);
    processFeedback(&a2);
    processFeedback(&a3);
    checkSerial();
    sendSerialFeedback();
}

void sendSerialFeedback() {
  if (serialFeedback == 0) {
    return;
  } else if (serialFeedback == 1) {
    sendValues('A', (1023 - a1.feedback)/4, (1023 - a1.target)/4);
    sendValues('A', a1.feedback/4, a1.target/4);
    sendValues('a', 1*16 + !a1.enabled + !a1.enabled * 2 + !a3.enabled * 4, a1.speed);
  } else if (serialFeedback == 2) {
    sendValues('B', (1023 - a2.feedback)/4, (1023 - a2.target)/4);
    sendValues('b', 1*16 + !a1.enabled + !a1.enabled * 2 + !a3.enabled * 4, a2.speed);
  } else if (serialFeedback == 3) {
    sendValues('C', (1023 - a3.feedback)/4, (1023 - a3.target)/4);
    sendValues('c', 1*16 + !a1.enabled + !a1.enabled * 2 + !a3.enabled * 4, a3.speed);
  }
}

// Initialize an axis in the stopped state with default values;
void initAxis(Axis *a, Adafruit_DCMotor *motor, int feedbackPin) {
  a->feedbackPin = feedbackPin;
  pinMode(a->feedbackPin, INPUT);

  a->motor = motor;
  a->target = a->feedback;
  brakeAxis(a);
  
  a->speedMax = 255;
  a->speedMin = 0;
  a->feedback = 512;
  a->deadzone = 4;
  a->cutoffMin = 28;  // fix me
  a->cutoffMax = 1024;  // fix me
  a->clipMin = 100;
  a->clipMax = 923;
  a->backupSpeed = 128;
  a->lshift = 0;
  a->rshift = 0;
  a->kp_x100 = 420;
  a->ki_x100 = 40;
  a->kd_x100 = 40;
  a->ks = 1;
}

// parse serial input command
void parseCommand() {
  switch (rxBuffer[0]) {
    case END_BYTE:
      parkAxis(&a1);
      parkAxis(&a2);
      parkAxis(&a3);
      break;
    case 'A':
      a1.target = cmdValue();
      break;
    case 'B':
      a2.target = cmdValue();
      break;
    case 'C':
      a3.target = cmdValue();
      break;
    case 'D':
      a1.kp_x100 = cmdValue();
      break;
    case 'E':
      a2.kp_x100 = cmdValue();
      break;
    case 'F':
      a3.kp_x100 = cmdValue();
      break;
    case 'G':
      a1.ki_x100 = cmdValue();
      break;
    case 'H':
      a2.ki_x100 = cmdValue();
      break;
    case 'I':
      a3.ki_x100 = cmdValue();
      break;
    case 'J':
      a1.kd_x100 = cmdValue();
      break;
    case 'K':
      a2.kd_x100 = cmdValue();
      break;
    case 'L':
      a3.kd_x100 = cmdValue();
      break;
    case 'M':
      a1.ks = cmdValue();
      break;
    case 'N':
      a2.ks = cmdValue();
      break;
    case 'O':
      a3.ks = cmdValue();
      break;
    case 'P':
      a1.speedMin = rxBuffer[1];
      a1.speedMax = rxBuffer[2];
      break;
    case 'Q':
      a2.speedMin = rxBuffer[1];
      a2.speedMax = rxBuffer[2];
      break;
    case 'R':
      a3.speedMin = rxBuffer[1];
      a3.speedMax = rxBuffer[2];
      break;
    case 'S':
      a1.cutoffMin = rxBuffer[1];
      a1.cutoffMax = 1023 - rxBuffer[1];
      a1.clipMin = rxBuffer[1];
      a1.clipMax = 1023 - rxBuffer[1];
      break;
    case 'T':
      a2.cutoffMin = rxBuffer[1];
      a2.cutoffMax = 1023 - rxBuffer[1];
      a2.clipMin = rxBuffer[1];
      a2.clipMax = 1023 - rxBuffer[1];
      break;
    case 'U':
      a2.cutoffMin = rxBuffer[1];
      a2.cutoffMax = 1023 - rxBuffer[1];
      a2.clipMin = rxBuffer[1];
      a2.clipMax = 1023 - rxBuffer[1];
      break;
    case 'V':
      a1.deadzone = rxBuffer[1];
      a1.backupSpeed = rxBuffer[2];
      break;
    case 'W':
      a2.deadzone = rxBuffer[1];
      a2.backupSpeed = rxBuffer[2];
      break;
    case 'X':
      a3.deadzone = rxBuffer[1];
      a3.backupSpeed = rxBuffer[2];
      break;
    case 'Z':
      // TODO: pid  proceess divider
      break;
    case '~':
      // TODO: set PWM frequency
      break;

    case 'd':
      // disable axes
      if (rxBuffer[1]=='i') {
        switch (rxBuffer[2]) {
          case 's':
            disableAxis(&a1);
            disableAxis(&a2);
            disableAxis(&a3);
            break;
          case '1':
            disableAxis(&a1);
            break;
          case '2':
            disableAxis(&a2);
            break;
          case '3':
            disableAxis(&a3);
            break;
          default:
            sendError();
            break;
        }
        break;
      }
    case 'e':
      // enable axes
      if (rxBuffer[1]=='n') {
        switch (rxBuffer[2]) {
          case 'a':
            enableAxis(&a1);
            enableAxis(&a2);
            enableAxis(&a3);
            break;
          case '1':
            enableAxis(&a1);
            break;
          case '2':
            enableAxis(&a2);
            break;
          case '3':
            enableAxis(&a3);
            break;
          default:
            sendError();
            break;
        }
        break;
      }
    case 'm':
      if (rxBuffer[1] == 'o') {
        if (rxBuffer[2] == '1') {
          serialFeedback = 1;
        } else if (rxBuffer[2] == '2') {
          serialFeedback = 2;
        } else if (rxBuffer[2] == '3') {
          serialFeedback = 3;
        } else if (rxBuffer[2] == '0') {
          serialFeedback = 0;
        }
        break;
      }
    case 'r':
      if (rxBuffer[1] == 'd') {
        switch (rxBuffer[2]) {
          case 'A':
            sendValues('A', a1.feedback/4, a1.target/4);
            break;
          case 'B':
            sendValues('B', a2.feedback/4, a2.target/4);
            break;
          case 'C':
            sendValues('C', a3.feedback/4, a3.target/4);
            break;
          case 'D':
            sendValue('D', a1.kp_x100);
            break;
          case 'E':
            sendValue('E', a2.kp_x100);
            break;
          case 'F':
            sendValue('F', a3.kp_x100);
            break;
          case 'G':
            sendValue('G', a1.ki_x100);
            break;
          case 'H':
            sendValue('H', a2.ki_x100);
            break;
          case 'I':
            sendValue('I', a3.ki_x100);
            break;
          case 'J':
            sendValue('J', a1.kd_x100);
            break;
          case 'K':
            sendValue('K', a2.kd_x100);
            break;
          case 'L':
            sendValue('L', a3.kd_x100);
            break;
          case 'M':
            sendValue('M', a1.ks);
            break;
          case 'N':
            sendValue('N', a2.ks);
            break;
          case 'O':
            sendValue('O', a3.ks);
            break;
          case 'P':
            sendValues('P', a1.speedMin, a1.speedMax);
            break;
          case 'Q':
            sendValues('Q', a2.speedMin, a2.speedMax);
            break;
          case 'R':
            sendValues('R', a3.speedMin, a3.speedMax);
            break;
          case 'S':
            sendValues('S', a1.cutoffMin, a1.clipMin);
            break;
          case 'T':
            sendValues('T', a2.cutoffMin, a2.clipMin);
            break;
          case 'U':
            sendValues('U', a3.cutoffMin, a3.clipMin);
            break;
          case 'V':
            sendValues('V', a1.deadzone, a1.backupSpeed);
            break;
          case 'W':
            sendValues('W', a2.deadzone, a2.backupSpeed);
            break;
          case 'X':
            sendValues('X', a3.deadzone, a3.backupSpeed);
            break;
          case 'Y':
            sendValues('Y', 16 * !a1.enabled + (!a2.enabled*2) + (!a3.enabled * 4), 0); // TODO: Support process divider
            break;
          case 'Z':
            sendValue('Z', 0); // TODO: Was delta loop count
            break;
          case '~':
            sendValues('~', 16, 16); // default PWM of 1.6khz from chip, but doesn't support setting as of yet (stote local value)
          default:
            sendError();
            break;
        }
      } else {
        sendError();
        break;
      }
    case 's': 
      // TODO: save
      break;
    case 'v':
      // version
      if (rxBuffer[1]=='e' && rxBuffer[2] == 'r') {
        sendValue('v', VERSION);
        break;
    }
    default:
      sendError();
  }
}

// send a single word value via serial
void sendValue(int id, int value) {
  int high = value / 256;
  int low = value - (high * 256);
  Serial.write(START_BYTE);
  Serial.write(id);
  Serial.write(high);
  Serial.write(low);
  Serial.write(END_BYTE);
}

// send 2 words via serial
void sendValues(int id, int v1, int v2) {
  Serial.write(START_BYTE);
  Serial.write(id);
  Serial.write(v1);
  Serial.write(v2);
  Serial.write(END_BYTE);  
}

// send a serial error response
void sendError() {
  Serial.write(START_BYTE);
  Serial.write(ERR_RESP);
  Serial.write(END_BYTE);
}

// Check incoming serial data for a command
void checkSerial() {
  while(Serial.available()) {
    rxByte = Serial.read();

    // start parsing new command
    if (bufferEnd == -1) { 
      if (rxByte == START_BYTE) {
        bufferEnd = 0;
      }
    } else {
      // parse rest of commmand
      rxBuffer[bufferEnd] = rxByte;
      bufferEnd++;
      if(bufferEnd > 3) {
        if(rxBuffer[3]==END_BYTE) {
          parseCommand();
        } else {
          sendError();
        }
        bufferEnd=-1;
      }
    }
  }
}

// set the axis target to predefined stop position
void parkAxis(Axis *a) {
  a->target = a->stopPosition;
}

// Stop and disable an axis.
void disableAxis(Axis *a) {
  a->enabled = false;
  brakeAxis(a);
}

// Stop axis, but do not disable
void brakeAxis(Axis *a) {
  a->speed = 0;
  a->motor->setSpeed(a->speed);
  a->direction = RELEASE;
  a->motor->run(a->direction);
}

// Enable an axis
void enableAxis(Axis *a) {
  a->enabled = true;
  a->motor->setSpeed(a->speed);
  a->motor->run(a->direction);
}

// read and convert command value from the buffer
int cmdValue() {
  return (rxBuffer[1] * 256) + rxBuffer[2];
}

// calculates how a given axis needs to move in order to reach target position
void calcMotorPID(Axis *a) {
  static int error=0;                                                     
  static long pTerm_x100=0;
  static long dTerm_x100=0;
  static long iTerm_x100=0;
  static int cumErr=0;
  static int lastPos=0;
  static int deltaPos=0;
  static int KdFilterCount=0;

  error = a->target - a->feedback;
  if (abs(error) <= a->deadzone) {
    cumErr = 0;
  } else {
    cumErr += error;
    cumErr = constrain(cumErr, -1024, 1024);            
  }         

  // error can only be between +/-1023 and Kp1_100 is
  // constrained to 0-1000 so can work with type long
  pTerm_x100 = a->kp_x100 * (long)error;  
  iTerm_x100 = (a->ki_x100 * (long)cumErr) >> a->rshift;
  
  // Apply a level of filtering to Kd parameter to help reduce motor noise
  KdFilterCount++;
  if (KdFilterCount >= a->ks) {
    deltaPos = (a->feedback - lastPos);
    lastPos = a->feedback;
    dTerm_x100 = (a->kd_x100 * (long)deltaPos) << a->lshift;
    KdFilterCount=0;    
  }

  //  The /128 (PowerScale) is an approximation to bring x100 terms back to units. 
  //  Accurate/consistent enough for this function, and the power calculated can be
  //  easily converted in to speed and direction.
  int power = constrain((pTerm_x100 + iTerm_x100 - dTerm_x100) >> powerScale, -255, 255);
  a->speed = constrain(abs(power), a->speedMin, a->speedMax);
  a->direction = (power >= 0) ? FORWARD : BACKWARD;
}

// reads the feedback pin and either brakes or updates motor direction & speed.
void processFeedback(Axis *a) {
  a->feedback = analogRead(a->feedbackPin);
  if (!a->enabled) {
    return;
  }

  // calculate new speed and direction
  calcMotorPID(a);

  a->target = constrain(a->target, a->clipMin, a->clipMax);
  
  if ((a->feedback > a->clipMax) && a->backupSpeed > 0) {
    // we overshot, so back up a bit
    a->speed = a->backupSpeed;
    a->direction = BACKWARD;
    a->motor->setSpeed(a->speed);
    a->motor->run(a->direction);
  } else if ((a->feedback < a->clipMin) && a->backupSpeed > 0) {
    // we undershot, so move forward a bit
    a->speed = a->backupSpeed;
    a->direction = FORWARD;
    a->motor->setSpeed(a->speed);
    a->motor->run(a->direction);
  } else if ((a->speed > 0) && ((a->feedback > a->cutoffMax) || (a->feedback < a->cutoffMin))){
    // we exceeded the cutoff, and as a safety precaution, disable the axis.
    disableAxis(a);
  } else if (a->target > (a->feedback + a->deadzone)) {
    // Target direction is forward
    a->direction = FORWARD;
    a->motor->setSpeed(a->speed);
    a->motor->run(a->direction);
  } else if (a->target < (a->feedback - a->deadzone)) {
    // Target direction is backwards
    a->direction = BACKWARD;
    a->motor->setSpeed(a->speed);
    a->motor->run(a->direction);
  } else {
    // we are where we want to be so brake for now
    brakeAxis(a);
  }
}


