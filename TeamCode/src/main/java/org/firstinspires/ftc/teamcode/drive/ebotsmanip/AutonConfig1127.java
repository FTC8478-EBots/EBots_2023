package org.firstinspires.ftc.teamcode.drive.ebotsmanip;

import static org.firstinspires.ftc.teamcode.drive.ebotsmanip.AutonConfigBackup.startingPosition;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;
import java.util.List;

@Config
public class AutonConfig1127 {




    Telemetry telemetry;
    Gamepad gamepad;
    int elementPos = 2;
    //int parkingPos = 2;
    boolean red = true;
    //final int cMult;


    //int frontStage = 1;
    int throughDoor = 0;
    public static int teamColor = 0; //0 == RED
    public static int frontStage = 1; //0 == backstage by the boards
    public static int startingDelay = 0;
    private boolean previousLeftBumper = false;
    private boolean previousRightBumper = false;
    private boolean previousUp = false;
    private boolean previousDown = false;

    public AutonConfig1127(Telemetry telemetry, Gamepad gamepad) {
        this.telemetry = telemetry;
        this.gamepad = gamepad;

    }
    public boolean processUpdates() {
        telemetry.addLine("Left Bumper to change team.");
        telemetry.addLine("Right Bumper to change position.");
        telemetry.addLine("Up/Down to change delay.");
        telemetry.addLine("Circle to finalize");
        telemetry.addData("Team Color", red ? "RED":"BLUE");
        telemetry.addData("Team Position", frontStage == 1 ? "Frontstage":"Backstage");
        telemetry.addData("Starting Delay",startingDelay);
        if (gamepad.left_bumper  && !previousLeftBumper) {
            red = !red;
        }
        previousLeftBumper = gamepad.left_bumper;

        if (gamepad.right_bumper  && !previousRightBumper) {
            frontStage = 1-frontStage;
        }
        previousRightBumper = gamepad.right_bumper;
        if (gamepad.dpad_up && !previousUp) {
            if (0 <= startingDelay && startingDelay <= 9) {
                startingDelay += 1;
            }
        }

        previousUp = gamepad.dpad_up;
        if (gamepad.dpad_down && !previousDown) {
            if (1 <= startingDelay && startingDelay <= 10) {
                startingDelay -= 1;
            }
        }
        previousDown = gamepad.dpad_down;
        telemetry.update();
        if (gamepad.circle)
        {
            telemetry.addLine("Ready To Run.");
            telemetry.addData("Team Color", teamColor == 0 ? "RED":"BLUE");
            telemetry.addData("Team Position", startingPosition == 0 ? "Backstage":"Frontstage");
            telemetry.addData("Starting Delay",startingDelay);
            telemetry.update();

            return true;
        }
        return false;

    }
    //returns a list of trajectories to be executed in order.
    public List<Trajectory> generateAutonTrajectories(SampleMecanumDrive drive,Intake intake, Lift pixelArm) {
        List<Trajectory> returnValue = new ArrayList<>();
        final int cMult = red ? 1 : -1;

        Pose2d startPose = new Pose2d(-36, cMult*(-66), cMult*Math.toRadians(90));
        Trajectory startToSpikeL = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-36, cMult*(-35 + Math.abs(elementPos - 2) * 7), cMult*Math.toRadians(270 - elementPos * 90)))
                .lineToLinearHeading(new Pose2d(-36, cMult*(-58),cMult*Math.toRadians(0)))
                //.addDisplacementMarker(()->intake.ejectPixel())
                .build();
        Trajectory spikeToPixelL = drive.trajectoryBuilder(startToSpikeL.end())
                .lineToLinearHeading(new Pose2d(47, cMult*(-58), Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(47, cMult*(-24 - elementPos * 6), Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(48, cMult*(-24 - elementPos * 6), Math.toRadians(0)))
                .build();
        Trajectory pixelToParkL = drive.trajectoryBuilder(spikeToPixelL.end())
                .lineToLinearHeading(new Pose2d(48, cMult*(-56)/* + parkingPos * 24*/, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(56, cMult*(-56)/*+parkingPos * 24*/, Math.toRadians(0)))
                .build();
        //Trajectory startAwayFromTheWall = null;

        //drive.trajectorySequenceBuilder(startPose)
                //startToPixel
                //.lineToLinearHeading(new Pose2d(-36, cMult*(-36), Math.toRadians(0)))

                //.addDisplacementMarker(()->intake.ejectPixel())
                //turnAfterSpike
                //.lineToLinearHeading(new Pose2d(-42,-36,Math.toRadians(0)))
                //pixelToBackdrop

                //.splineTo(new Vector2d(36,-12),Math.toRadians(0))
                //backdropToParking

        drive.setPoseEstimate(startPose);
        returnValue.add(startToSpikeL);
        returnValue.add(spikeToPixelL);
        returnValue.add(pixelToParkL);
        return returnValue;
    }
}
