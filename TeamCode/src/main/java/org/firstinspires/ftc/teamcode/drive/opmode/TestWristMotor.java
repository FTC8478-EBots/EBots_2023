package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.ebotsmanip.DataStorage;
import org.firstinspires.ftc.teamcode.drive.ebotsmanip.Lift;

@TeleOp(group = "drive")
@Config
public class TestWristMotor extends LinearOpMode {
    public static double handPosition = 0;
    public static double wristPosition = 0;

    public static double WRIST_ANGLE = 11;

    //0=> back locked in front open down
    //.14 => front locked
    //.08 movable
    //.22 Both Open
    //.03 1 out

    //Wrist
    //.89 +90
    //.18 stowed
    //.61 straight



    @Override

    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        Lift lift = new Lift(hardwareMap, null);
        lift.init(this);
        lift.wristEnableStability(true);
        DcMotorEx wristMotor = hardwareMap.get(DcMotorEx.class,"wristMotor");

       // wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristMotor.setTargetPosition(0);
        //wristMotor.setMotorEnable();

        waitForStart();


        while (!isStopRequested()) {
            //lift.setwristFieldCentricAngle(WRIST_ANGLE);
            //lift.stabilizeWrist();
            //lift.setWristAngle(WRIST_ANGLE);

            telemetry.addData("Wrist Position",wristMotor.getCurrentPosition());
            telemetry.addData("BaseArm Angle",lift.getLowerArmAngleDegrees());
            telemetry.addData("UpperArm Angle",lift.getUpperArmAngleDegrees());
            telemetry.update();

            //wristMotor.setPower(0.5);
            //wristMotor.setTargetPosition(200);
            //linearOpMode.sleep(500);
            //wristMotor.setTargetPosition(0);
            //linearOpMode.sleep(500);
        }

        }
    }
