#pragma config(Sensor, dgtl2,  ,               sensorQuadEncoder)
#pragma config(Sensor, dgtl4,  ,               sensorQuadEncoder)
#pragma config(Motor,  port1,           lift,          tmotorVex393_HBridge, openLoop, encoderPort, None)
#pragma config(Motor,  port2,           leftDrive1,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           leftDrive2,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           rightDrive1,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           rightDrive2,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           shoulderR1,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           shoulderR2,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           shoulderL1,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           shoulderL2,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          claw,          tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//this is a test comment

void drive(){
	motor[rightDrive1] = -vexRT[Ch3];
	motor[rightDrive2] = -vexRT[Ch3];
	motor[leftDrive1] = vexRT[Ch2];
	motor[leftDrive2] = vexRT[Ch2];
}

void liftShoulder(){
	if(vexRT[Btn7D]==1){
		motor[shoulderR1] = -100;
		motor[shoulderL1] = 100;
		motor[shoulderR2] = -100;
		motor[shoulderL2] = -100;
	} else if (vexRT[Btn7L] == 1){
		motor[shoulderR1] = 100;
		motor[shoulderL1] = -100;
		motor[shoulderR2] = 100;
		motor[shoulderL2] = 100;
	} else{
		motor[shoulderR1] = 0;
		motor[shoulderL1] = 0;
		motor[shoulderR2] = 0;
		motor[shoulderL2] = 0;
	}
}

void liftClaw() {
	if (vexRT[Btn7U] == 1) {
		motor[claw] = 100;
	} else if (vexRT[Btn7R] == 1) {
		motor[claw] = -100;
	} else {
		motor[claw] = 0;
	}
}

void liftGoal() {
	if (vexRT[Btn8U] == 1) {
		motor[lift] = 80;
	} else if (vexRT[Btn8R] == 1){
		motor[lift] = -80;
	} else{
		motor[lift] = 0;
	}
}

task main(){
	while(true){
		drive();
		liftShoulder();
		liftClaw();
		liftGoal();
	}
}
