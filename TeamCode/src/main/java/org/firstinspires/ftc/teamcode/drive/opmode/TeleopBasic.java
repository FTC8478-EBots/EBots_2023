package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.CenterStageConstants.bluePixelX;
import static org.firstinspires.ftc.teamcode.drive.CenterStageConstants.bluePixelY;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.CenterStageRobotConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.ebotsmanip.Camera;
import org.firstinspires.ftc.teamcode.drive.ebotsmanip.DataStorage;
import org.firstinspires.ftc.teamcode.drive.ebotsmanip.DroneLauncher;
import org.firstinspires.ftc.teamcode.drive.ebotsmanip.Intake;
import org.firstinspires.ftc.teamcode.drive.ebotsmanip.Lift;

@TeleOp(group = "drive")
@Config
public class TeleopBasic extends LinearOpMode {
    public static double handPosition = 0;
    public static double wristPosition = 0;

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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(DataStorage.currentPose);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        Lift pixelArm = new Lift(hardwareMap, telemetry);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DataStorage.alreadyInitialized = true;
        Servo wristServo = hardwareMap.get(Servo.class, "wristServo");
        Servo handServo = hardwareMap.get(Servo.class,"handServo");
        waitForStart();


        while (!isStopRequested()) {
            Pose2d poseEstimate = drive.getPoseEstimate();

// Create a vector from the gamepad x/y inputs
// Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

// Pass in the rotated input + right stick value for rotation
// Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );
wristServo.setPosition(wristPosition);
handServo.setPosition(handPosition);
                drive.update();

                poseEstimate = drive.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
                telemetry.addData("arm position", pixelArm.getArmPosition());
                telemetry.update();
            }
        }
    }
