package org.firstinspires.ftc.teamcode.drive.ebotsmanip;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.CenterStageConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;
import java.util.List;

public class AutonConfig {
    Telemetry telemetry;
    Gamepad gamepad;

    public int teamColor = 0; //0 == RED
    public int startingPosition = 1; //0 == backstage by the boards
    public int startingDelay = 0;
    private boolean previousLeftBumper = false;
    private boolean previousRightBumper = false;
    private boolean previousUp = false;
    private boolean previousDown = false;
    public AutonConfig(Telemetry telemetry, Gamepad gamepad) {
        this.telemetry = telemetry;
        this.gamepad = gamepad;

    }
    public boolean processUpdates() {
        telemetry.addLine("Left Bumper to change team.");
        telemetry.addLine("Right Bumper to change position.");
        telemetry.addLine("Up/Down to change delay.");
        telemetry.addLine("Circle to finalize");
        telemetry.addData("Team Color", teamColor == 0 ? "RED":"BLUE");
        telemetry.addData("Team Position", startingPosition == 0 ? "Backstage":"Frontstage");
        telemetry.addData("Starting Delay",startingDelay);
        if (gamepad.left_bumper  && !previousLeftBumper) {
             teamColor = 1-teamColor;
        }
        previousLeftBumper = gamepad.left_bumper;

        if (gamepad.right_bumper  && !previousRightBumper) {
            startingPosition = 1-startingPosition;
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
    public List<Trajectory> generateAutonTrajectories(SampleMecanumDrive drive, int spikePosition) {
        List<Trajectory> returnValue = new ArrayList<>();
        //Place a pixel
        Pose2d startPose  = null;
        if (teamColor == 0 && startingPosition == 0) {
            startPose = CenterStageConstants.startPoseRedBackStage;
        }
        if (teamColor == 0 && startingPosition == 1) {
             startPose = CenterStageConstants.startPoseRedFrontStage;
        }
        if (teamColor == 1 && startingPosition == 0) {
             startPose = CenterStageConstants.startPoseBlueBackStage;
        }
        if (teamColor == 1 && startingPosition == 1) {
             startPose = CenterStageConstants.startPoseBlueFrontStage;
        }
        drive.setPoseEstimate(startPose);

        Trajectory startToPixel = null;
        Trajectory pixelToBackdrop = null;
        Trajectory backdropToParking = null;
        Trajectory turnAfterSpike = null;

        if (teamColor == 0 && startingPosition == 0) {
            throw new RuntimeException("Forgot Something");
         }
        if (teamColor == 0 && startingPosition == 1) {
            //red frontstage
            startToPixel = drive.trajectoryBuilder(startPose)
                    .splineTo(new Vector2d(-36, -42),Math.toRadians(90))
                    .build();
            turnAfterSpike = drive.trajectoryBuilder(startToPixel.end())
                    .lineToLinearHeading(new Pose2d(-42,-42,Math.toRadians(0)))
                    .build();
            pixelToBackdrop = drive.trajectoryBuilder(turnAfterSpike.end())
                    .lineToLinearHeading(new Pose2d(48, -42,Math.toRadians(0)))
                    //.splineTo(new Vector2d(36,-12),Math.toRadians(0))
                    //.splineTo(new Vector2d())
                    .build();
            backdropToParking = drive.trajectoryBuilder(pixelToBackdrop.end())
                    .splineTo(new Vector2d(-48, -42),Math.toRadians(0))
                    //.splineTo(new Vector2d(36,-12),Math.toRadians(0))
                    //.splineTo(new Vector2d())
                    .build();
        }
        if (teamColor == 1 && startingPosition == 0) {
            throw new RuntimeException("Forgot Something");

        }
        if (teamColor == 1 && startingPosition == 1) {
            throw new RuntimeException("Forgot Something");

        }
        //startToPixel add magical drop pixel Code here:
        returnValue.add(startToPixel);
        returnValue.add(turnAfterSpike);
        returnValue.add(pixelToBackdrop);
        returnValue.add(backdropToParking);


        //go to boards
        //go to start
        //go to boards;
        return returnValue;
    }





}
