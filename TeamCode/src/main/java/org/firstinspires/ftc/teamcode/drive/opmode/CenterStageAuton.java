package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.ebotsmanip.AutonConfig;
import org.firstinspires.ftc.teamcode.drive.ebotsmanip.AutonConfig1127;
import org.firstinspires.ftc.teamcode.drive.ebotsmanip.Camera;
import org.firstinspires.ftc.teamcode.drive.ebotsmanip.DataStorage;
import org.firstinspires.ftc.teamcode.drive.ebotsmanip.Intake;
import org.firstinspires.ftc.teamcode.drive.ebotsmanip.Lift;
import org.firstinspires.ftc.teamcode.drive.ebotsmanip.TeamElementDetector;

import java.util.List;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class CenterStageAuton extends LinearOpMode {
    AutonConfig1127 config;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        Lift pixelArm = new Lift(hardwareMap,telemetry);
        Intake intake = new Intake(hardwareMap);
        pixelArm.init(this);

        DataStorage.alreadyInitialized = true;

        config = new AutonConfig1127(telemetry, gamepad1);
        //Camera camera = new Camera(hardwareMap, telemetry);
        while (!config.processUpdates() && !opModeIsActive() &&!isStopRequested()) ;
        TeamElementDetector detector = new TeamElementDetector(hardwareMap, telemetry);
        detector.init();
        while (!isStopRequested() && !opModeIsActive()) {
            int position = detector.detect();
            telemetry.addData("Team element position", position);
            telemetry.update();
        }

        List<Trajectory> trajectories = config.generateAutonTrajectories(drive,intake,pixelArm);
  //      waitForStart();
        if (isStopRequested()) return;
        /*telemetry.addLine("robot is ready to start moving");
        telemetry.update();
        telemetry.addLine("picking up pixel");
        telemetry.update();*/

        //camera.getSpikePosition();
        for (Trajectory traj : trajectories) {
            drive.followTrajectory(traj);
        }



        sleep(1000);
        DataStorage.currentPose = drive.getPoseEstimate();


    }
}
