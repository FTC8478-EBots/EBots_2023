package org.firstinspires.ftc.teamcode.drive.ebotsmanip;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DroneLauncher {
    Servo droneServo;

    public DroneLauncher(HardwareMap hardwareMap) {
        droneServo  = hardwareMap.get(Servo.class,"droneServo");
        droneServo.setPosition(0);
    }

    public void launchDrone() {
        droneServo.setPosition(1);
    }
    public void resetDrone() {
        droneServo.setPosition(0);
    }
}
