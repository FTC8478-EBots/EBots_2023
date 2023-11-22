package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.CenterStageConstants.bluePixelX;
import static org.firstinspires.ftc.teamcode.drive.CenterStageConstants.bluePixelY;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
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
public class TeleopTest2 extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(DataStorage.currentPose);
        Intake intake = new Intake(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        Camera camera = new Camera(hardwareMap, telemetry);
        camera.init();
        camera.enable();
        DroneLauncher droneLauncher = new DroneLauncher(hardwareMap);
        Lift pixelArm = new Lift(hardwareMap, telemetry);
        if (!DataStorage.alreadyInitialized) {
            pixelArm.init(this);
        }
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //DataStorage.alreadyInitialized = true;
        waitForStart();
        intake.powerServos(true);
        boolean flagup = false;
        boolean flagdown = false;
        boolean flagright = false;
        boolean flagleft = false;
        int column = 1;
        int row = 1;
        boolean pixelMode = false;
        boolean pixelFlag = false;
        boolean intakeOn = false;
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
            pixelArm.stabilizeWrist();
            telemetry.addData("Joystick 1 leftX", gamepad1.left_stick_x);

            if (gamepad2.square) {
                pixelArm.loadPixels();
                telemetry.addData("opening pixel grabber, 2square", "");
            }
            if (gamepad1.triangle) {
                droneLauncher.launchDrone();
                telemetry.addData("launching airplane, 2triangle", "");
            }
            if (gamepad2.cross) {
                telemetry.addLine("stowArm");
                pixelArm.stowArm();
            }
            if (gamepad2.triangle) {
                telemetry.addLine("PullArm");
                pixelArm.pullArm();
            }
            if (gamepad2.dpad_up) {
                if (!flagup) {
                    if (1 <= (row + 1) & (row + 1) <= 11) {
                        row += 1;
                    }
                }
                flagup = true;
                //Lift.setHeight(300);
                telemetry.addData("pixel arm up, 2up", "");
            }
            if (!gamepad2.dpad_up) {
                flagup = false;
            }
            if (gamepad2.dpad_down) {
                if (!flagdown) {
                    if (1 <= row - 1 & row - 1 <= 11) {
                        row -= 1;
                    }
                }
                flagdown = true;
                //Lift.setHeight(300);
                telemetry.addData("pixel arm down, 2down", "");
            }
            if (!gamepad2.dpad_down) {
                flagdown = false;
            }
            if (gamepad2.dpad_right) {
                if (!flagright) {
                    if (1 <= (column + 1)) {
                        column += 1;
                    }
                }
                flagright = true;
                //Lift.setHeight(300);
                telemetry.addData("moving right, 2right", "");
            }
            if (!gamepad2.dpad_right) {
                flagright = false;
            }
            if (gamepad2.dpad_left) {
                if (!flagleft) {
                    if (1 <= (column - 1)) {
                        column -= 1;
                    }
                }
                flagleft = true;
                //Lift.setHeight(300);
                telemetry.addData("moving left, 2left", "");
            }
            if (!gamepad2.dpad_left) {
                flagleft = false;
            }
            telemetry.addData("column", column);
            telemetry.addData("row", row);

            if (gamepad1.dpad_up) {
                telemetry.addData("Prepare to Hang arm, 1up", "");
                pixelArm.prepareToHang();
                //PixelArm.lower
            }
            if (gamepad1.dpad_down) {
                telemetry.addData("hang, 1down", "");
                pixelArm.hang();
                //PixelArm.lower
            }


            /*if (gamepad1.circle) {
                //HangArm.hang
                telemetry.addData("hanging, 1circle","");
            }*/
            if (gamepad1.square) {
                intake.powerServos(true);

                telemetry.addData("Intake on", "square1");
            } else if (gamepad1.circle) {
                intake.ejectPixel();
                telemetry.addData("Intake off", "circle1");
            } else {
                intake.powerServos(false);
            }
            /*if (pixelMode) {
                if (gamepad2.left_bumper) {
                    drive.pixelArm.setArmPos(row);
                }
            }*/

            if (gamepad2.left_bumper) {
                //pixelArm.pixelSetArmPos(row);
                //TeleopTest2.prepareColumn(column);
                telemetry.addData("Preparing row", row);
                //double[] angles = pixelArm.getAngleSettingsDegrees(row, 48 - drive.getPoseEstimate().getX(), CenterStageRobotConstants.baseDistanceOffGround);
                //telemetry.addData("Go to base position", angles[0]);
                //telemetry.addData("Go to arm position", angles[1]);
                telemetry.addData("Preparing column", column);
                //ask camera for position
                Pose2d distanceToTarget = camera.getDistanceToColumnOne();
                if (distanceToTarget != null) {
                    telemetry.addData("Preparing column", drive.getPoseEstimate());
                    telemetry.addData("Estimated Distance: ", distanceToTarget.getY());
                    telemetry.addData("Estimated Offset: ", distanceToTarget.getX());
                    telemetry.addData("Estimated skew", distanceToTarget.getHeading());


                    if (!(drive.getPoseEstimate().getX() == bluePixelX & drive.getPoseEstimate().getY() == bluePixelY[column - 1])) {
                        //Trajectory move = drive.trajectoryBuilder(drive.getPoseEstimate())
                        //        .lineToLinearHeading(new Pose2d(bluePixelX, bluePixelY[column - 1], Math.toRadians(0)))
                        //        .build();
                        //drive.followTrajectory(move);
                    }
                } else {
                    telemetry.addLine("Target not found!!");
                }
            }

                if (Math.abs(gamepad2.left_stick_y) > .1 || Math.abs(gamepad2.left_stick_x) > .1) {
                    //
                    pixelArm.setManualControl();
                    pixelArm.setBaseMotorVelocity(gamepad2.left_stick_y / 2 - gamepad2.left_stick_x);
                    pixelArm.setArmMotorVelocity(gamepad2.left_stick_y + gamepad2.left_stick_x);
                    //pixelArm.setBaseMotorVelocity(Math.cos(pixelArm.getLowerArmAngle()));

                    telemetry.addData("manual override", "");
                } else {
                    pixelArm.setPositionControl();
                }
                if (Math.abs(gamepad2.right_stick_y) > .1 && Math.abs(gamepad2.right_stick_x) > .1) {
                    pixelArm.setwristFieldCentricAngle(Math.atan(gamepad2.right_stick_x/gamepad2.right_stick_y));
                }
                if (gamepad1.cross) {
                    droneLauncher.resetDrone();
                }
           /* if (Math.abs(gamepad2.left_stick_x) > .1) {
                //
                drive.pixelArm.setBaseMotorVelocity(gamepad2.left_stick_x);
                drive.pixelArm.setArmMotorVelocity(gamepad2.left_stick_x);
                telemetry.addData("triangle is pressed on 1", "");
            }
            /*if (gamepad1.triangle) {
                //
                drive.pixelArm.setBaseMotorVelocity(.3);
                drive.pixelArm.setArmMotorVelocity(.3);
                telemetry.addData("triangle is pressed on 1","");
            }
            else if (gamepad1.cross) {
                //
                drive.pixelArm.setBaseMotorVelocity(-.3);
                drive.pixelArm.setArmMotorVelocity(-.3);
                telemetry.addData("x is pressed on 1","");
            }
            else {
                drive.pixelArm.setBaseMotorVelocity(0.0);
                drive.pixelArm.setArmMotorVelocity(0.0);
            }*/
                if (gamepad1.left_bumper & gamepad1.right_bumper & gamepad1.left_trigger > 0.5 & gamepad1.right_trigger > 0.5) {
                    telemetry.addData("Reset location", "oo");
                    Pose2d currentPosition = drive.getPoseEstimate();
                    drive.setPoseEstimate(new Pose2d(currentPosition.getX(),currentPosition.getY(),0));
                }
                telemetry.addData("Base angle", pixelArm.getLowerArmAngleDegrees());
                telemetry.addData("Arm angle", pixelArm.getUpperArmAngleDegrees());
                telemetry.addData("Upper arm XY", pixelArm.getUpperArmXY());
                double[] angles = pixelArm.getAngleSettingsDegrees(row, 48 - drive.getPoseEstimate().getX(), CenterStageRobotConstants.baseDistanceOffGround);
                telemetry.addData("Go to base position", angles[0]);
                telemetry.addData("Go to arm position", angles[1]);
            /*if gamepad2.Circle pressed:
            Close grabber
            if gamepad2.Square pressed:
            Open grabber
            if gamepad2.Triangle pressed:
            Launch airplane

            PixelArm.setWeightedDrivePower(
                    new Pose2d(-gamepad2.left_stick_y, -gamepad2.left_stick_x, -gamepad2.right_stick_x)
            );
            */
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
