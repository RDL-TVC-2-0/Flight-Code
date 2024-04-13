#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SD.h>

File BenchFile;

Adafruit_BNO055 bno = Adafruit_BNO055(55);
//Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

#define userButtonPin 6
unsigned long userButtonTime = 1000;
const int userButtonDelay = 2000;

const unsigned long SHORT_CODE = 500;
const unsigned long LONG_CODE = 1000;
const unsigned int CODE_TONE = 262;

float orient[6];
unsigned long lastTime;
float* outputang = (float*)malloc(2*sizeof(float));
float lengthInputs[3];

#define buzzerPin 37
#define LEDPin_green 41
#define LEDPin_yellow 40
#define LEDPin_red 39
#define LEDPin_blue 38

#define actuator1Forward  0
#define actuator1Backward 1
#define actuator2Forward  2
#define actuator2Backward 3
#define actuator3Forward  4
#define actuator3Backward 5

// Linear actuator kinematic characteristics
// Assumes constant acceleration
// TODO: Find actual neutral length, max speed, and acceleration time from testing
#define maxSpeed 3 // m/s
#define accel 3 // m/s^2
#define accelTime maxSpeed/accel // s
#define lengthTransient 0.5*accel*pow(accelTime, 2) // m

File dataTable;
struct TableRange {
      int count;
      float min, max, diff;
};
TableRange tablePitchRange, tableYawRange;

struct actuator {
  byte forwardPin, backwardPin;
  float currentLength, displacement, driveTime;
  int pwm;
};

enum FlightStateType {
	BoardInit_State,
	Idle_State,
	LaunchReady_State,
	Boost_State,
	Coast_State,
	Recovery_State,
	ShutDown_State,
} FlightState;
unsigned long FlightStateTimer;

void setup() {
	pinMode(userButtonPin, INPUT);
	
	pinMode(buzzerPin, OUTPUT);
	
	pinMode(LEDPin_green, OUTPUT);
	pinMode(LEDPin_yellow, OUTPUT);
	pinMode(LEDPin_red, OUTPUT);
	pinMode(LEDPin_blue, OUTPUT);

	pinMode(actuator1Forward, OUTPUT);
	pinMode(actuator1Backward, OUTPUT);
	pinMode(actuator2Forward, OUTPUT);
	pinMode(actuator2Backward, OUTPUT);
	pinMode(actuator3Forward, OUTPUT);
	pinMode(actuator3Backward, OUTPUT);

   // Setup SD IO
  if (!SD.begin(BUILTIN_SDCARD)) {
    int code[] = {LONG_CODE, LONG_CODE};
    CodeFailure(code); // Pulled the code failure function from Flight-Code.ino
  }
  dataTable = SD.open("LengthTable.bin", FILE_WRITE);
  readLengthsFromFile("LengthTable.txt", &dataTable);
  // Explicitly reopening the file as "read" as this file should now remain constant
  dataTable.close();
  dataTable = SD.open("LengthTable.bin", FILE_READ);
}

void loop() {
  getLengths(outputang[0], outputang[1], lengthInputs, &dataTable); // inputs for getLengths taken from PIDcontrol.ino
  driveActuators(lengthInputs);

	// State Machine
	switch (FlightState) {
		case BoardInit_State:
			BoardInit();
			break;

		case Idle_State:
			Idle();
			break;

		case LaunchReady_State:
			LaunchReady();
			break;

		case Boost_State:
			Boost();
			break;

		case Coast_State:
			Coast();
			break;

		case Recovery_State:
			Recovery();
			break;

		case ShutDown_State:
			ShutDown();
			break;
	}
}

void SwitchStateTo(FlightStateType state) {
	FlightStateTimer = millis();
	FlightState = state;
}

void CodeFailure(int failureCode[]) {
	digitalWrite(LEDPin_yellow, HIGH);

	for (int i = 0; i <= 2; i++) {
		int beep = failureCode[i];
		digitalWrite(LEDPin_blue, HIGH);
		tone(buzzerPin, CODE_TONE, beep);
		delay(beep);
		digitalWrite(LEDPin_blue, LOW);
		noTone(buzzerPin);
		delay(beep);
	}
	digitalWrite(LEDPin_blue, LOW);
	noTone(buzzerPin);

	digitalWrite(LEDPin_yellow, LOW);
	digitalWrite(LEDPin_red, HIGH);

	while(true);
}

void WaitForUserButton() {
	// Delay until user button input is found
	digitalWrite(LEDPin_blue, HIGH);
	while (!ReadUserButton()) {
		delay(100);
	}
	digitalWrite(LEDPin_blue, LOW);
	delay(userButtonDelay);
}

unsigned long lastDebounceTime;
int buttonState = LOW;
int lastButtonState = LOW;
bool ReadUserButton() {
	int reading = digitalRead(userButtonPin);

	if (reading != lastButtonState) {
		lastDebounceTime = millis();
	}

	if ((millis() - lastDebounceTime) > userButtonDelay) {
		if (reading != buttonState) {
			buttonState = reading;
		}
	}

	lastButtonState = reading;

	if (buttonState == HIGH && reading == HIGH) {
		return true;
	}
	else {
		return false;
	}
}

void BoardInit() {
	WaitForUserButton();
	{		
		SwitchStateTo(Idle_State);
	}
}

#define SERVO_MAX 50
#define SERVO_MID 90
void Idle() {
	// Setup BNO055
	if (!bno.begin()) {
		int code[] = {SHORT_CODE, LONG_CODE};
		CodeFailure(code);
	}
	
	// Setup SD IO
	/*if (!SD.begin(BUILTIN_SDCARD)) {
		int code[] = {LONG_CODE, LONG_CODE};
		CodeFailure(code);
	} else {
    BenchFile = SD.open("arduino.txt", FILE_WRITE);
	  if (BenchFile) {
      BenchFile.print("Pitch\tYaw\tRoll");
    } else {
		int code[] = {LONG_CODE, SHORT_CODE};
		CodeFailure(code);
	}
  	}
	SD.remove("arduino.txt");*/

	BenchFile = SD.open("arduino.txt", FILE_WRITE);
	if (!BenchFile) {
		int code[] = {LONG_CODE, SHORT_CODE};
		CodeFailure(code);
	}

	digitalWrite(LEDPin_green, HIGH);
	
	WaitForUserButton();
	{
		digitalWrite(LEDPin_yellow, HIGH);
		
    // Check Actuator Range of Motion

		// Check Servo Range of Motion
		/*servoA.write(SERVO_MID);
		servoB.write(SERVO_MID);
		delay(5000);
		servoA.write(SERVO_MID + SERVO_MAX);
		servoB.write(SERVO_MID + SERVO_MAX);
		delay(5000);
		servoA.write(SERVO_MID - SERVO_MAX);
		servoB.write(SERVO_MID - SERVO_MAX);
		delay(5000);
		servoA.write(SERVO_MID);
		servoB.write(SERVO_MID);
		delay(5000);*/

		digitalWrite(LEDPin_yellow, LOW);
		
		WaitForUserButton();
		{
			// Calibrate BNO055
      uint8_t system, gyro, accelerometer, mag;
      system = gyro = accelerometer = mag = 0;
			bno.setExtCrystalUse(true);
			bno.getCalibration(&system, &gyro, &accelerometer, &mag);
      while(system != 3);
			SwitchStateTo(LaunchReady_State);
		}
	}
}

void LaunchReady() {
	digitalWrite(LEDPin_red, HIGH);
 
	// Acquire acceleration data
	/*imu::Vector<3> linaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
 
	// If high acceleration in z direction (towards nosecone)
	if (linaccel.z > 1) {
		delay(1000);
		digitalWrite(LEDPin_red, LOW);
		delay(1000);
		SwitchStateTo(Boost_State);
	}*/
}

#define BOOST_TIME 4000
void Boost() {
	// Acquire orientation data
	orientation(orient);
	// PID actuator positions
	PIDcontrol(orient, outputang, (millis()-lastTime));
	float lengthInputs[3];// = getLengths(1,1); // inputs for getLengths taken from PIDcontrol.ino
	// Move Servos
  	driveActuators(lengthInputs);
	// Log data to SD IO
  	String outputString = String(orient[0]) + "\t" + String(orient[1]) + "\t" + String(orient[2]);
  	BenchFile = SD.open("orientation.txt", FILE_WRITE);
	if (BenchFile) {
    		Serial.println("Writing to orientation.txt");
    		BenchFile.println(outputString);
	} else {
    		int code[] = {LONG_CODE, SHORT_CODE};
		CodeFailure(code);
  	}
	
	if (millis() - FlightStateTimer > BOOST_TIME) {
		// Deactivate actuators
		analogWrite(actuator1Forward, 0);
    	analogWrite(actuator1Backward, 0);
    	analogWrite(actuator2Forward, 0);
   		analogWrite(actuator2Backward, 0);
    	analogWrite(actuator3Forward, 0);
    	analogWrite(actuator3Backward, 0);

		SwitchStateTo(Coast_State);
	}
}

#define COAST_TIME 4000
void Coast() {
	// Acquire and log orientation data
	orientation(orient);
	String outputString = String(orient[0]) + "\t" + String(orient[1]) + "\t" + String(orient[2]);
  	BenchFile = SD.open("orientation.txt", FILE_WRITE);
	if (BenchFile) {
    		Serial.println("Writing to orientation.txt");
    		BenchFile.println(outputString);
	} else {
    		int code[] = {LONG_CODE, SHORT_CODE};
		CodeFailure(code);
  	}
	
	if (millis() - FlightStateTimer > COAST_TIME) {
		SwitchStateTo(Recovery_State);
	}
}

void Recovery() {
	// Stop BNO055
	bno.enterSuspendMode();
	
	// Close SD IO
	BenchFile.close();
	
	// Cycle Lights and Buzzer
	digitalWrite(LEDPin_green, HIGH);
	digitalWrite(LEDPin_yellow, HIGH);
	digitalWrite(LEDPin_red, LOW);
	digitalWrite(LEDPin_blue, LOW);
	tone(buzzerPin, 1000);
	delay(3000);
	digitalWrite(LEDPin_green, LOW);
	digitalWrite(LEDPin_yellow, LOW);
	digitalWrite(LEDPin_red, HIGH);
	digitalWrite(LEDPin_blue, HIGH);
	noTone(buzzerPin);
	
	WaitForUserButton();
	{
		SwitchStateTo(ShutDown_State);
	}
}

void ShutDown() {
	exit(0);
}

void orientation(float orient[6]) {
	// Create quaternion object and get quaternion data from BNO055
	imu::Quaternion quat = bno.getQuat();

	// Create 3-axis vector for angular velocity
	imu::Vector<3> angvel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

	// Assign conjugate quaternion values to variables
	float qw = quat.w();
	float qx = quat.x();
	float qy = quat.y();
	float qz = quat.z();

	// Get angular rate data from BNO055
	float pitch_rate = angvel.x();
	float yaw_rate = angvel.y();
	float roll_rate = angvel.z();

	// Convert quaternion data to Euler angles
	float pitch, yaw, roll;
	quaternionToEuler(qx,qy,qz,qw,&pitch,&yaw,&roll);

	// Assigning Pitch, Yaw, and Roll to elements of orient[] array
	orient[0] = pitch;
	orient[1] = yaw;
	orient[2] = roll;

	// Assigning Pitch rate, Yaw rate, and Roll rate to elements of orient[] array
	orient[3] = pitch_rate;
	orient[4] = yaw_rate;
	orient[5] = roll_rate;

	// Plotting Pitch, Yaw, and Roll
	plotOrientation(pitch,yaw,roll);
}

void quaternionToEuler(float qx, float qy, float qz, float qw, float* pitch, float* yaw, float* roll) {
	// Convert quaternion to Euler angles (pitch, roll, and yaw) in degrees
	*pitch = atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy)) * 180.0 / M_PI;
	*yaw = asin(2.0 * (qw * qy - qz * qx)) * 180.0 / M_PI; 
	*roll = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)) * 180.0 / M_PI;
}

void plotOrientation(float pitch, float yaw, float roll) {
	// Display angles in the Serial Plotter
	Serial.print("\nPitch: ");
	Serial.print(pitch);
	Serial.print("\tYaw: ");
	Serial.println(yaw);
	Serial.print("\tRoll: ");
	Serial.println(roll);
}

// PID Control Parameters for Pitch
#define kp_pitch 0.0
#define ki_pitch 0.0
#define kd_pitch 0.0

// PID Control Parameters for Yaw
#define kp_yaw 0.0
#define ki_yaw 0.0
#define kd_yaw 0.0

// Target Pitch and Yaw Angles
float target_pitch = 0.0;
float target_yaw = 0.0;

// Initializing Integral Error
float int_pitch = 0.0; 
float int_yaw = 0.0;

void PIDcontrol(float orient[6], float* outputang, unsigned long dt) {
	// Assign orientation values to pitch, yaw, pitch_rate, and yaw_rate variables
	float pitch = orient[0];
	float yaw = orient[1];
	float pitch_rate = orient[3];
	float yaw_rate = orient[4];

	// Error calculation
	float pitch_error = target_pitch - pitch;
	float yaw_error = target_yaw - yaw;

	// Proportional error
	float prop_pitch = kp_pitch*pitch_error;
	float prop_yaw = kp_yaw*yaw_error;

	// Integral error
	int_pitch = int_pitch + (ki_pitch*pitch_error*dt);
	int_yaw = int_yaw + (ki_yaw*yaw_error*dt);

	// Derivative error
	float der_pitch = kd_pitch*pitch_rate;
	float der_yaw = kd_yaw*yaw_rate;

	// PID Output
	float output_pitch = prop_pitch + int_pitch - der_pitch;
	float output_yaw = prop_yaw + int_yaw - der_yaw;

	// Updating outputang[] array with PID processed correction angles
	outputang[0] = output_pitch;
	outputang[1] = output_yaw; 
}

void getLengths(float inputPitch, float inputYaw, float* lengths, File* dataFile) {
  // Using the input pitch and yaw, and the known table ranges, we calculate the position in the file
  // for which the desired lengths should be located
  // Clamp the pitch and yaw between the max and min
  if (inputPitch > tablePitchRange.max) inputPitch = tablePitchRange.max;
  if (inputPitch < tablePitchRange.min) inputPitch = tablePitchRange.min;
  if (inputYaw > tableYawRange.max) inputYaw = tableYawRange.max;
  if (inputYaw < tableYawRange.min) inputYaw = tableYawRange.min;
  // Find the bounding pitch "indices" from the desired pitch input
  int pitchIdx = (int)((inputPitch - tablePitchRange.min) / tablePitchRange.diff);
  // Find the bounding yaw "indices" from the desired yaw input
  int yawIdx = (int)((inputYaw - tableYawRange.min) / tableYawRange.diff);
  // Combine the two and take into account the 3 length values per angle to find the actual file locations
  byte tableData[12];
  byte* dataPtr = tableData;
  dataFile->seek((pitchIdx*tableYawRange.count + yawIdx) * 3);
  dataFile->read(dataPtr, 6);
  dataFile->seek(((pitchIdx+1)*tableYawRange.count + yawIdx) * 3);
  dataFile->read(dataPtr+6, 6);
  // Interpolate between the two to get the resulting length output
  // Since the length is a function of both pitch and yaw, we use bilinear interpolation
  // Yaw was arbitrarily chosen to go first, but order shouldn't matter
  float minPitch = pitchIdx*tablePitchRange.diff + tablePitchRange.min;
  float maxPitch = (pitchIdx+1)*tablePitchRange.diff + tablePitchRange.min;
  float minYaw = yawIdx*tableYawRange.diff + tableYawRange.min;
  float maxYaw = (yawIdx+1)*tableYawRange.diff + tableYawRange.min;
  int idx;
  float interp[6];
  for (idx = 0; idx < 3; idx++) {
    interp[idx] = (float)tableData[idx]*(maxYaw-inputYaw)/(tableYawRange.diff) + (float)tableData[idx+3]*(inputYaw-minYaw)/(tableYawRange.diff);
    interp[idx+3] = (float)tableData[idx+6]*(maxYaw-inputYaw)/(tableYawRange.diff) + (float)tableData[idx+9]*(inputYaw-minYaw)/(tableYawRange.diff);
  }
  for (idx = 0; idx < 3; idx++) {
    lengths[idx] = interp[idx]*(maxPitch-inputPitch)/(tablePitchRange.diff) + interp[idx+3]*(inputPitch-minPitch)/(tablePitchRange.diff);
  }
}

/*int orderDriveTimes(const actuator* actuator1, const actuator* actuator2) {
  return actuator1->driveTime - actuator2->driveTime;
}*/

byte actuatorDirection(float displacement, byte forwardPin, byte backwardPin) {
  if (displacement >= 0)
    return forwardPin;
  else
    return backwardPin;
}

void driveActuators(float lengthInputs[3]) {
	/*  Assume constants are acceleration and max speed, must be experimentally found
	 *  Limitations: error propagates over time since there's no way to check position past initial conditions */

  static int maxDriveTime = 0;
  static actuator actuators[3] = { { 0, 1, 1, 0, 0 },
                                   { 2, 3, 1, 0, 0 },
                                   { 4, 5, 1, 0, 0 } };

  for (int i = 0; i <= 2; ++i) {
    actuators[i].displacement = lengthInputs[i] - actuators[i].currentLength;
    
    // Calculating drive times for actuators at max power (function of desired length and current length)
    if (lengthInputs[i] >= lengthTransient) {
      // Drive time if it's less than the time it takes to accelerate to full speed
      actuators[i].driveTime = abs((lengthInputs[i] - actuators[i].currentLength))*accelTime/lengthTransient*1000; // ms
    } else {
      // Drive time if it's greater than the time it takes to accelerate to full speed
      actuators[i].driveTime = (abs((lengthInputs[i] - actuators[i].currentLength - lengthTransient))/maxSpeed + accelTime)*1000; // ms
    }
  }
  // Assign drive timings and associated actuators from shortest to longest
  // qsort(actuators, 3, sizeof(actuator), orderDriveTimes);
  maxDriveTime = max(actuators[0].driveTime, max(actuators[1].driveTime, actuators[2].driveTime));

  // Slowing down the other two actuators to have the same drive time as the slowest one
  for (int i = 0; i <= 2; ++i) {
    actuators[i].pwm = actuators[i].driveTime/maxDriveTime*255;
    actuators[i].driveTime = actuators[i].driveTime / actuators[i].pwm * 255; // ms
  }

  // Driving actuators
  for (int i = 0; i <= 2; ++i) {
    analogWrite(actuatorDirection(actuators[i].displacement, actuators[i].forwardPin, actuators[i].backwardPin), actuators[i].pwm);
  }
  delay(maxDriveTime);
  for (int i = 0; i <= 2; ++i) {
    analogWrite(actuatorDirection(actuators[i].displacement, actuators[i].forwardPin, actuators[i].backwardPin), 0);
  }

  /* More accurate but also more computationally demanding method of driving motors
  // Sort by drive time again for stopping the actuators at the right time
  qsort(actuators, 3, sizeof(actuator), orderDriveTimes);

  // Driving actuators
  for (int i = 0; i <= 2; ++i) {
    analogWrite(actuatorDirection(actuators[i].displacement, actuators[i].forwardPin, actuators[i].backwardPin), actuators[i].pwm);
  }
  for (int i = 0; i <= 2; ++i) {
    delay(actuators[i].driveTime);
    analogWrite(actuatorDirection(actuators[i].displacement, actuators[i].forwardPin, actuators[i].backwardPin), 0);
  }
  */

  // Set the new lengths as initial conditions for next iteration
  actuators[0].currentLength = lengthInputs[0]; // m
  actuators[1].currentLength = lengthInputs[1]; // m
  actuators[2].currentLength = lengthInputs[2]; // m
}

void readLengthsFromFile(const char* sourceFileName, File* targetFile) {
  /* This function converts the human-readable MATLAB-generated length table file and converts it into a more
   * compressed machine-readable (but therefore not very human-readable) file, intended to maximize read speed.
   * The human-readable file will have a header, in which the final line will contain a line of equals signs
   * Due to the file generation, simply checking for a line with an equals sign is enough to detect the end of
   * the header
   * The data format of the file is as follows:
   * <sign><pitchAngle %2.3f> <sign><yawAngle %2.3f> <length1 %2d> <length2 %2d> <length3 %2d>
   * To be more efficient with memory usage, these values will be packed by the following format:
   *   angles -> uint16_t's where the decimal place occurs after the second digit (ex: 7.24 -> 07240)
   *   lengths -> bytes, only utilizing 0-50mm
   * This, as a note, this is a tradeoff between relatively low RAM usage and read speed/precision
   * To streamline usage, the function "getLengths" takes this packed format and converts the length into a double
   * This packing format also specifies that angles MUST be less than 65.535 degrees in either direction with 0.001
   * degree precision, and that lengths MUST be less than 256mm with only millimeter precision
  */
  // Open the file
  File sourceFile = SD.open(sourceFileName, FILE_READ);
  if (!sourceFile) {
    int code[] = {LONG_CODE, SHORT_CODE};
    CodeFailure(code);
  }
  // Skip the header
  int rlen = sourceFile.available();
  bool foundEquals = false;
  char ch = ' ';
  for (int count = 0; count < rlen && !(foundEquals && ch == '\n'); count++, ch = sourceFile.read()) {
    if (ch == '=') foundEquals = true;
  }
  // Read in the file statistics to get the data ranges
  String buffer = "";
  rlen = sourceFile.available();
  int currentVar = 0;
  for (int count = 0; count < rlen && currentVar < 6; count++, ch = sourceFile.read()) {
    if (ch == ' ' || ch == '\n') {
      switch (currentVar) {
      case 0:
        tablePitchRange.count = buffer.toInt();
        break;
      case 1:
        tablePitchRange.min = buffer.toFloat();
        break;
      case 2:
        tablePitchRange.max = buffer.toFloat();
        break;
      case 3:
        tableYawRange.count = buffer.toInt();
        break;
      case 4:
        tableYawRange.min = buffer.toFloat();
        break;
      case 5:
        tableYawRange.max = buffer.toFloat();
        break;
      }
      buffer = "";
      currentVar++;
    }
    else
      buffer += ch;
  }
  tablePitchRange.diff = (tablePitchRange.max - tablePitchRange.min) / (tablePitchRange.count - 1);
  tableYawRange.diff = (tableYawRange.max - tableYawRange.min) / (tableYawRange.count - 1);
  // Transfer data to the target file
  buffer = "";
  rlen = sourceFile.available();
  currentVar = 0;
  bool emptyBuffer = true;
  for (int count = 0; count < rlen; count++, ch = sourceFile.read()) {
    if ((ch == ' ' || ch == '\n') && !emptyBuffer) {
      if (!(currentVar == 0 || currentVar == 1)) {
        // Length, no conversion is needed
        // We ignore angle, as it should come in correctly from the MATLAB
        targetFile->write((byte)buffer.toInt());
      }
      buffer = "";
      emptyBuffer = true;
      currentVar = (currentVar + 1) % 6;
    }
    else {
      emptyBuffer = false;
      buffer += ch;
    }
  }
  // Close the file
  sourceFile.close();
}
