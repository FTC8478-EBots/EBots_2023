package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.ebotsmanip.AutonConfig;
import org.firstinspires.ftc.teamcode.drive.ebotsmanip.DataStorage;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
@Disabled

public class AutoTest2 extends LinearOpMode {
    AutonConfig config;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //drive.pixelArm.init(telemetry);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        config = new AutonConfig(telemetry, gamepad1);
        while (!config.processUpdates()) ;

        Pose2d startPose = new Pose2d(-36, -66, Math.toRadians(90));
        Pose2d startPose2 = new Pose2d(48, -36, 0);
        drive.setPoseEstimate(startPose);
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                //.splineTo(new Vector2d(-36,-36),0)
                .splineTo(new Vector2d(-36, 0),0)
                .splineTo(new Vector2d(24,0),0)
                .splineTo(new Vector2d(24,-36),0)
                .splineTo(new Vector2d(48,-36),0)
                .splineTo(new Vector2d(24, -36),0)
                .splineTo(new Vector2d(24, 0),0)
                .splineTo(new Vector2d(-36, 0),0)
                .splineTo(new Vector2d(-36, -66),0)
                .build();

        Trajectory traj1rev = drive.trajectoryBuilder(startPose2)
                .splineTo(new Vector2d(24, -36),Math.toRadians(180))
                //.splineTo(new Vector2d(24,0),Math.toRadians(180))
                //.splineTo(new Vector2d(-36,0),Math.toRadians(180))
                //.splineTo(new Vector2d(-36, -66), Math.toRadians(180))
                .build();
        waitForStart();
        if (isStopRequested()) return;
        telemetry.addLine("robot is ready to start moving");
        telemetry.update();
        drive.followTrajectory(traj1);
        telemetry.addLine("picking up pixel");
        telemetry.update();


        sleep(1000);
        DataStorage.currentPose = drive.getPoseEstimate();
        //telemetry.addData("robot has gone back","yes");
        //drive.followTrajectory(traj3);
        //sleep(2000);

       /* drive.followTrajectory(
                drive.trajectoryBuilder(traj.end())
                .lineToSplineHeading(new Pose2d(0, -24, Math.toRadians(180)))
                        .build()

        );
*/

    }
}
