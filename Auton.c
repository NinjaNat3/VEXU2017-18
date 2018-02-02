#pragma config(Sensor, in1,    shoulderMeasure, sensorPotentiometer)
#pragma config(Sensor, dgtl1,  driveRight,     sensorQuadEncoder, reversed)
#pragma config(Sensor, dgtl3,  driveLeft,      sensorQuadEncoder, reversed)
#pragma config(Motor,  port1,           claw,          tmotorVex393_HBridge, openLoop, encoderPort, None)
#pragma config(Motor,  port2,           driveR1,       tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           driveR2,       tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           driveL1,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           driveL2,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           shoulderL1,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           shoulderL2,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           shoulderR1,    tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           shoulderR2,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          lift,          tmotorVex393_HBridge, openLoop)

float wheelD = 67/16; //in
float wheelC = wheelD * PI; //in

float speedRight;
float speedLeft;
float maxShoulderSpeed = 63;
float minShoulderSpeed = 45;
float speedShoulder;
float maxClawSpeed = 63;
float minClawSpeed = 25;
float speedClaw;
float maxLiftSpeed = 63;
float speedLift;

float previousSpeedRight = 0;
float previousSpeedLeft = 0;
float previousShoulder = 0;
float previousClaw = 0;
float previousLift = 0;

float errorCoeff = 4/5;

float errorSpeedRight;
float errorSpeedLeft;
float errorShoulder;
float errorClaw;
float errorLift;

float maxValue = 127;
float minValue = -127;

float speedError = 95/100;

float currentPos = 0;

bool tog = false;

float disBetweenWheel = 17;
float turnSpeed = 127;

void driveForward(float speed)
{
		//Probability factor
		errorSpeedRight 	= errorCoeff * (speed - previousSpeedRight);
		errorSpeedLeft = errorCoeff * (speed - previousSpeedLeft);

		//Swerve Drive with P-Loop
		motor[driveR1] 	= speed - errorSpeedRight;
		motor[driveR2] 	= speed - errorSpeedRight;
		motor[driveL1] 	= speed - errorSpeedLeft;
		motor[driveL2] 	= speed - errorSpeedLeft;

		//New previous Speed
		previousSpeedRight = speed - errorSpeedRight;
		previousSpeedLeft 	= speed - errorSpeedLeft;
}
void driveBackward(float speed)
{
		//Probability factor
		errorSpeedRight 	= errorCoeff * (-speed - previousSpeedRight);
		errorSpeedLeft = errorCoeff * (-speed - previousSpeedLeft);

		//Swerve Drive with P-Loop
		motor[driveR1] 	= -speed - errorSpeedRight;
		motor[driveR2] 	= -speed - errorSpeedRight;
		motor[driveL1] 	= -speed - errorSpeedLeft;
		motor[driveL2] 	= -speed - errorSpeedLeft;

		//New previous Speed
		previousSpeedRight = -speed - errorSpeedRight;
		previousSpeedLeft 	= -speed - errorSpeedLeft;
}
void turnLeft(float speed)
{
		//Probability factor
		errorSpeedRight 	= errorCoeff * (speed - previousSpeedRight);
		errorSpeedLeft = errorCoeff * (-speed - previousSpeedLeft);

		//Swerve Drive with P-Loop
		motor[driveR1] 	= speed - errorSpeedRight;
		motor[driveR2] 	= speed - errorSpeedRight;
		motor[driveL1] 	= -speed - errorSpeedLeft;
		motor[driveL2] 	= -speed - errorSpeedLeft;

		//New previous Speed
		previousSpeedRight = speed - errorSpeedRight;
		previousSpeedLeft 	= -speed - errorSpeedLeft;
}
void turnRight(float speed)
{
		//Probability factor
		errorSpeedRight 	= errorCoeff * (speed - previousSpeedRight);
		errorSpeedLeft = errorCoeff * (-speed - previousSpeedLeft);

		//Swerve Drive with P-Loop
		motor[driveR1] 	= speed - errorSpeedRight;
		motor[driveR2] 	= speed - errorSpeedRight;
		motor[driveL1] 	= -speed - errorSpeedLeft;
		motor[driveL2] 	= -speed - errorSpeedLeft;

		//New previous Speed
		previousSpeedRight = speed - errorSpeedRight;
		previousSpeedLeft 	= -speed - errorSpeedLeft;
}

void stopRobot()
{
//Probability factor
		errorSpeedRight 	= errorCoeff * (previousSpeedRight);
		errorSpeedLeft	= errorCoeff * (previousSpeedLeft);

		//Swerve Drive with P-Loop
		motor[driveR1] 	= errorSpeedRight;
		motor[driveR2] 	= errorSpeedRight;
		motor[driveL1] 	= errorSpeedLeft;
		motor[driveL2] 	= errorSpeedLeft;

		//New previous Speed
		previousSpeedRight = errorSpeedRight;
		previousSpeedLeft	= errorSpeedLeft;
}

void holdShoulder()
{

	if ((currentPos > 500) && (currentPos < 3000))
	{
		if(tog)
		{
			if(currentPos - 100 < SensorValue[shoulderMeasure])
			{
				speedShoulder = 15;
				motor[shoulderR1]	= speedShoulder;
				motor[shoulderR2]	= speedShoulder;
				motor[shoulderL1]	= speedShoulder;
				motor[shoulderL2]	= speedShoulder;
				previousShoulder = 15;
			}
			else
			{
				tog = false;
				speedShoulder = -15;
				motor[shoulderR1]	= speedShoulder;
				motor[shoulderR2]	= speedShoulder;
				motor[shoulderL1]	= speedShoulder;
				motor[shoulderL2]	= speedShoulder;
				previousShoulder = -15;
			}
		}
		else
		{
			if(currentPos + 100 > SensorValue[shoulderMeasure])
			{
				speedShoulder = -15;
				motor[shoulderR1]	= speedShoulder;
				motor[shoulderR2]	= speedShoulder;
				motor[shoulderL1]	= speedShoulder;
				motor[shoulderL2]	= speedShoulder;
				previousShoulder = -15;
			}
			else
			{
				tog = true;
				speedShoulder = 15;
				motor[shoulderR1]	= speedShoulder;
				motor[shoulderR2]	= speedShoulder;
				motor[shoulderL1]	= speedShoulder;
				motor[shoulderL2]	= speedShoulder;
				previousShoulder = 15;
			}
		}

	}
	else
	{
		errorShoulder	= errorCoeff * previousShoulder;

		speedShoulder 	= errorShoulder;

		motor[shoulderR1]	= speedShoulder;
		motor[shoulderR2]	= speedShoulder;
		motor[shoulderL1]	= speedShoulder;
		motor[shoulderL2]	= speedShoulder;

		previousShoulder 	= speedShoulder;
		currentPos = SensorValue[shoulderMeasure];
	}
}

//Encoder Stuff Auton
void resetValues()
{
	SensorValue[driveRight] = 0;
	SensorValue[driveLeft] = 0;
}

void encoderForwards(float speed, float dist){

	resetValues();

	int ticks = dist/wheelC * 360;
	while(((abs(SensorValue[driveRight])) < ticks) &&
		((abs(SensorValue[driveLeft])) < ticks))
	{
	/*
		int rDiff = abs(nMotorEncoder[frontRight]) - abs(nMotorEncoder[frontLeft]);
		int rMod = sgn(rDiff)*speed*.1;
		motor[backLeft] = speed;
		motor[backRight] = speed;
		motor[frontLeft] = speed+rMod;
		motor[frontRight] = speed+rMod;
		*/
		holdShoulder();
		float speedChangeFactor = speed * speedError * (float)abs(SensorValue(driveRight))/ticks;
		driveForward(speed - speedChangeFactor);
		wait1Msec(50);
	}
}

void encoderBackwards(float speed, float dist){

	resetValues();

	float ticks = dist/wheelC * 360;
	while(((abs(SensorValue(driveRight))) > -ticks) &&
		((abs(SensorValue(driveLeft))) > -ticks))
	{
	/*
		int rDiff = abs(nMotorEncoder[frontRight]) - abs(nMotorEncoder[frontLeft]);
		int rMod = sgn(rDiff)*speed*.1;
		motor[backLeft] = speed;
		motor[backRight] = speed;
		motor[frontLeft] = speed+rMod;
		motor[frontRight] = speed+rMod;
		*/
		holdShoulder();
		float speedChangeFactor = speed * speedError * abs(SensorValue(driveRight))/ ticks;
		driveBackward(speed - speedChangeFactor);
		wait1Msec(50);
	}
}

void encoderTurnRight(float speed, float dist){

	resetValues();

	float ticks = dist/wheelC * 360;
	while(((abs(SensorValue(driveRight))) > -ticks) &&
		((abs(SensorValue(driveLeft))) < ticks))
	{
	/*
		int rDiff = abs(nMotorEncoder[frontRight]) - abs(nMotorEncoder[frontLeft]);
		int rMod = sgn(rDiff)*speed*.1;
		motor[backLeft] = speed;
		motor[backRight] = speed;
		motor[frontLeft] = speed+rMod;
		motor[frontRight] = speed+rMod;
		*/
		holdShoulder();
		float speedChangeFactor = speed * speedError * abs(SensorValue(driveRight))/ ticks;
		turnRight(speed - speedChangeFactor);
		wait1Msec(50);
	}
}

void encoderTurnLeft(float speed, float dist){

	resetValues();

	float ticks = dist/wheelC * 360;
	while(((abs(SensorValue(driveRight))) < ticks) &&
		((abs(SensorValue(driveLeft))) > -ticks))
	{
	/*
		int rDiff = abs(nMotorEncoder[frontRight]) - abs(nMotorEncoder[frontLeft]);
		int rMod = sgn(rDiff)*speed*.1;
		motor[backLeft] = speed;
		motor[backRight] = speed;
		motor[frontLeft] = speed+rMod;
		motor[frontRight] = speed+rMod;
		*/
		holdShoulder();
		float speedChangeFactor = speed * speedError * abs(SensorValue(driveRight))/ ticks;
		turnLeft(speed - speedChangeFactor);
		wait1Msec(50);
	}
}

void encoderStop()
{
	while (abs(previousSpeedRight) > 3)
	{
		holdShoulder();
		stopRobot();
		delay(50);
	}
	motor[driveR1] 	= 0;
	motor[driveR2] 	= 0;
	motor[driveL1] 	= 0;
	motor[driveL2] 	= 0;
	previousSpeedRight = 0;
	previousSpeedLeft = 0;
	holdShoulder();
}

void closeClaw()
{
	int t = 0;
	while(t<500)
	{
		errorClaw	= errorCoeff * (-minClawSpeed - previousClaw);

		speedClaw 	= -minClawSpeed - errorClaw;

		motor[claw]	= speedClaw;

		previousClaw 	= speedClaw;
		t+=50;
		wait1Msec(50);
	}
}

void openClaw()
{
	int t = 0;
	while(t<1000)
	{
		errorClaw	= errorCoeff * (maxClawSpeed - previousClaw);

		speedClaw 	= maxClawSpeed - errorClaw;

		motor[claw]	= speedClaw;

		previousClaw 	= speedClaw;
		t+=50;
		wait1Msec(50);
	}
	t = 0;
	while(t<300)
	{
		errorClaw = errorCoeff * previousClaw;
		speedClaw = errorClaw;
		motor[claw] = speedClaw;
		previousClaw = speedClaw;
		t+=50;
		wait1Msec(50);
	}
	motor[claw] = 0;
}

void shoulderUp()
{
	int t = 0;
	while(t<3000)
	{
		errorShoulder	= errorCoeff * (maxShoulderSpeed - previousShoulder);

		speedShoulder 	= maxShoulderSpeed - errorShoulder;

		motor[shoulderR1]	= speedShoulder;
		motor[shoulderR2]	= speedShoulder;
		motor[shoulderL1]	= speedShoulder;
		motor[shoulderL2]	= speedShoulder;

		previousShoulder 	= speedShoulder;

		t+=50;
		wait1Msec(50);
		tog = true;
		currentPos = SensorValue[shoulderMeasure];
	}

	t = 0;

	while(t<500)
	{
		errorShoulder = errorCoeff * previousShoulder;

		speedShoulder = errorShoulder;
		motor[shoulderR1]	= speedShoulder;
		motor[shoulderR2]	= speedShoulder;
		motor[shoulderL1]	= speedShoulder;
		motor[shoulderL2]	= speedShoulder;

		previousShoulder 	= speedShoulder;

		t+=50;
		wait1Msec(50);
		currentPos = SensorValue[shoulderMeasure];
	}
	motor[shoulderR1]	= 0;
	motor[shoulderR2]	= 0;
	motor[shoulderL1]	= 0;
	motor[shoulderL2]	= 0;
	currentPos = SensorValue[shoulderMeasure];
}

void shoulderDown()
{
	int t = 0;
	while(t<3000)
	{
		errorShoulder	= errorCoeff * (-maxShoulderSpeed - previousShoulder);

		speedShoulder 	= -maxShoulderSpeed - errorShoulder;

		motor[shoulderR1]	= speedShoulder;
		motor[shoulderR2]	= speedShoulder;
		motor[shoulderL1]	= speedShoulder;
		motor[shoulderL2]	= speedShoulder;

		previousShoulder 	= speedShoulder;

		t+=50;
		wait1Msec(50);
		tog = false;
		currentPos = SensorValue[shoulderMeasure];
	}

	t = 0;

	while(t<500)
	{
		errorShoulder = errorCoeff * previousShoulder;

		speedShoulder = errorShoulder;
		motor[shoulderR1]	= speedShoulder;
		motor[shoulderR2]	= speedShoulder;
		motor[shoulderL1]	= speedShoulder;
		motor[shoulderL2]	= speedShoulder;

		previousShoulder 	= speedShoulder;

		t+=50;
		wait1Msec(50);
		currentPos = SensorValue[shoulderMeasure];
	}
	motor[shoulderR1]	= 0;
	motor[shoulderR2]	= 0;
	motor[shoulderL1]	= 0;
	motor[shoulderL2]	= 0;
	currentPos = SensorValue[shoulderMeasure];
}

void goalUp()
{
	int t = 0;
	while(t<2000)
	{
		errorLift	= errorCoeff * (maxLiftSpeed - previousLift);
		speedLift 	= maxLiftSpeed - errorLift;
		motor[lift]	= speedLift;
		previousLift 	= speedLift;
		holdShoulder();

		t+=50;
		wait1Msec(50);
	}
	while(t<500)
	{
		errorLift	= errorCoeff * previousLift;
		speedLift 	= errorLift;
		motor[lift]	= speedLift;
		previousLift 	= speedLift;
		holdShoulder();

		t+=50;
		wait1Msec(50);
	}
	motor[lift]	= 0;
}

void goalDown()
{
		int t = 0;
	while(t<2000)
	{
		errorLift	= errorCoeff * (-maxLiftSpeed - previousLift);
		speedLift 	= -maxLiftSpeed - errorLift;
		motor[lift]	= speedLift;
		previousLift 	= speedLift;
		holdShoulder();

		t+=50;
		wait1Msec(50);
	}
	while(t<500)
	{
		errorLift	= errorCoeff * previousLift;
		speedLift 	= errorLift;
		motor[lift]	= speedLift;
		previousLift 	= speedLift;
		holdShoulder();

		t+=50;
		wait1Msec(50);
	}
	motor[lift]	= 0;
}

void shoulderMiddle()
{
	if(currentPos > 2000)
	{
		while (currentPos > 2000)
		{
			errorShoulder	= errorCoeff * (minShoulderSpeed - previousShoulder);

			speedShoulder 	= minShoulderSpeed - errorShoulder;

			motor[shoulderR1]	= speedShoulder;
			motor[shoulderR2]	= speedShoulder;
			motor[shoulderL1]	= speedShoulder;
			motor[shoulderL2]	= speedShoulder;

			previousShoulder 	= speedShoulder;
			wait1Msec(50);
			tog = true;
			currentPos = SensorValue[shoulderMeasure];
		}
	}
	else if(currentPos < 1900)
	{
		while (currentPos < 1900)
		{
			errorShoulder	= errorCoeff * (-minShoulderSpeed - previousShoulder);

			speedShoulder 	= -minShoulderSpeed - errorShoulder;

			motor[shoulderR1]	= speedShoulder;
			motor[shoulderR2]	= speedShoulder;
			motor[shoulderL1]	= speedShoulder;
			motor[shoulderL2]	= speedShoulder;

			previousShoulder 	= speedShoulder;
			wait1Msec(50);
			tog = false;
			currentPos = SensorValue[shoulderMeasure];
		}
	}
}

float distTurn()
{
	float turndist = disBetweenWheel * PI / 4;
	return turndist;
}

void turnExactRight()
{
	encoderTurnRight(turnSpeed, distTurn());
	encoderStop();
}

void turnExactLeft()
{
	encoderTurnLeft(turnSpeed, distTurn());
	encoderStop();
}

task main()
{

//I added a comment
	closeClaw();
	shoulderMiddle();
	encoderForwards(127,36);
	goalUp();
	shoulderUp();
	turnExactLeft();
	turnExactLeft();
	encoderForwards(127,40);
	openClaw();
	shoulderMiddle();
	encoderBackwards(127,36);
	encoderForwards(127,10);
	encoderStop();
}
