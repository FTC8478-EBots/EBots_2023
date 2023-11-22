package org.firstinspires.ftc.teamcode.drive.ebotsmanip;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    Servo intakeServoLeft;
    Servo intakeServoRight;
    Servo intakeAssistRear;
    Servo intakeAssistFront;
    public Intake(HardwareMap hardwareMap) {
        intakeServoLeft = hardwareMap.get(Servo.class,"intakeServoLeft");
        intakeServoLeft.setDirection(Servo.Direction.FORWARD);
        intakeServoRight = hardwareMap.get(Servo.class,"intakeServoRight");
        intakeServoRight.setDirection(Servo.Direction.REVERSE);
        intakeAssistFront = hardwareMap.get(Servo.class,"intakeAssistFront");
        intakeAssistFront.setDirection(Servo.Direction.FORWARD);
        intakeAssistRear = hardwareMap.get(Servo.class,"intakeAssistRear");
        intakeAssistRear.setDirection(Servo.Direction.REVERSE);
    }
    public void powerServos(boolean on) {
        if (on) {
            intakeServoLeft.setPosition(0);
            intakeServoRight.setPosition(0);
            intakeAssistFront.setPosition(0);
            intakeAssistRear.setPosition(0);
        } else {
            intakeServoLeft.setPosition(0.5);
            intakeServoRight.setPosition(0.5);
            intakeAssistFront.setPosition(0.5);
            intakeAssistRear.setPosition(0.5);
        }
    }
    public void ejectPixel() {
        intakeServoLeft.setPosition(1);
        intakeServoRight.setPosition(1);

        intakeAssistFront.setPosition(1);
        intakeAssistRear.setPosition(1);

    }

}