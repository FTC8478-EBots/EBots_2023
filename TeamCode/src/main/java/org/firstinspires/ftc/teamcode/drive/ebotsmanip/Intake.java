package org.firstinspires.ftc.teamcode.drive.ebotsmanip;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    Servo intakeServoLeft;
    Servo intakeServoRight;
    public Intake(HardwareMap hardwareMap) {
        intakeServoLeft = hardwareMap.get(Servo.class,"intakeServoLeft");
        intakeServoLeft.setDirection(Servo.Direction.FORWARD);
        intakeServoRight = hardwareMap.get(Servo.class,"intakeServoRight");
        intakeServoRight.setDirection(Servo.Direction.REVERSE);

    }
    public void powerServos(boolean on) {
        if (on) {
            intakeServoLeft.setPosition(0);
            intakeServoRight.setPosition(0);
        } else {
            intakeServoLeft.setPosition(0.5);
            intakeServoRight.setPosition(0.5);
        }
    }
}