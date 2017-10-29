#pragma config(Sensor, dgtl2,  ,               sensorQuadEncoder)
#pragma config(Sensor, dgtl4,  ,               sensorQuadEncoder)
#pragma config(Motor,  port1,           claw,          tmotorVex393_HBridge, openLoop, encoderPort, None)
#pragma config(Motor,  port2,           shoulderR,     tmotorServoContinuousRotation, openLoop, encoderPort, dgtl3)
#pragma config(Motor,  port3,           shoulderL,     tmotorServoContinuousRotation, openLoop, encoderPort, dgtl4)
#pragma config(Motor,  port4,           elbowR,        tmotorServoContinuousRotation, openLoop)
#pragma config(Motor,  port5,           elbowL,        tmotorServoContinuousRotation, openLoop)
#pragma config(Motor,  port6,           rightDrive,    tmotorServoContinuousRotation, PIDControl, encoderPort, dgtl1)
#pragma config(Motor,  port7,           leftDrive,     tmotorServoContinuousRotation, openLoop, reversed)
#pragma config(Motor,  port8,           liftR,         tmotorServoContinuousRotation, openLoop)
#pragma config(Motor,  port9,           liftL,         tmotorServoContinuousRotation, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

task main(){
	while(true){
		drive();
	}
}

void drive(){
	motor[right] = vexRT[Ch2];
	motor[left] = vexRT[Ch4];
}

void liftShoulder(){
	if(vexRT[Btn7D]==1){
		motor[shoulderR] = 40;
		motor[shoulderL] = 40;
	} else if (vexRT[Btn7L] == 1]){
		motor[shoulderR] = -40;
		motor[shoulderL] = -40;
	} else{
		motor[shoulderR] = 0;
		motor[shoulderL] = 0;
	}
}
