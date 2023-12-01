package org.firstinspires.ftc.teamcode.drive.ebotsmanip;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.CenterStageConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;
import java.util.List;

@Config
public class AutonConfigBackup {
    Telemetry telemetry;
    Gamepad gamepad;
    public CameraRedBlue.TEAM_START_POSITION teamStartPosition = null;
    public Pose2d startPose = null;
    public static int teamColor = 0; //0 == RED
    public static int startingPosition = 1; //0 == backstage by the boards
    public static int startingDelay = 0;
    private boolean previousLeftBumper = false;
    private boolean previousRightBumper = false;
    private boolean previousUp = false;
    private boolean previousDown = false;
    private boolean configComplete = false;
    public AutonConfigBackup(Telemetry telemetry, Gamepad gamepad) {
        this.telemetry = telemetry;
        this.gamepad = gamepad;

    }
    public boolean processUpdates() {
        if(configComplete) {
            telemetry.addLine("Ready To Run.");
            telemetry.addData("Team Color", teamColor == 0 ? "RED":"BLUE");
            telemetry.addData("Team Position", startingPosition == 0 ? "Backstage":"Frontstage");
            telemetry.addData("Starting Delay",startingDelay);
            return configComplete;
        }
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
        if (gamepad.circle)
        {

            configComplete = true;

        }
        if(teamColor == 0) {
            if (startingPosition == 0) {
                teamStartPosition = CameraRedBlue.TEAM_START_POSITION.RedLeft;
            } else {
                teamStartPosition = CameraRedBlue.TEAM_START_POSITION.RedRight;
            }
        } else {
            if (startingPosition == 0) {
                teamStartPosition = CameraRedBlue.TEAM_START_POSITION.BlueLeft;
            } else {
                teamStartPosition = CameraRedBlue.TEAM_START_POSITION.BlueRight;
            }
        }
        return configComplete;

    }

    //returns a list of trajectories to be executed in order.
    public List<Trajectory> generateAutonTrajectories(SampleMecanumDrive drive,Intake intake, Lift pixelArm) {
        List<Trajectory> returnValue = new ArrayList<>();


        //Pose2d startPose  = null;
        Trajectory startToPark = null;
        Trajectory startAwayFromTheWall = null;
        if (teamColor == 0 && startingPosition == 0) {
            startPose = CenterStageConstants.startPoseRedBackStage;
            startAwayFromTheWall = drive.trajectoryBuilder(startPose)
                    //.splineTo(new Vector2d(-36, 42),Math.toRadians(90))
                    .forward(4)
                    .build();
            startToPark = drive.trajectoryBuilder(startAwayFromTheWall.end())
                    //.splineTo(new Vector2d(-36, 42),Math.toRadians(90))
                    .strafeRight(48)
                    .addDisplacementMarker(()->intake.ejectPixel())
                    .build();


        }
        if (teamColor == 0 && startingPosition == 1) {
            startPose = CenterStageConstants.startPoseRedFrontStage;
            startAwayFromTheWall = drive.trajectoryBuilder(startPose)
                    //.splineTo(new Vector2d(-36, 42),Math.toRadians(90))
                    .forward(4)
                    .build();
            startToPark = drive.trajectoryBuilder(startAwayFromTheWall.end())
                    .strafeRight(96)
                    .addDisplacementMarker(()->intake.ejectPixel())
                    .build();
        }
        if (teamColor == 1 && startingPosition == 0) {
            startPose = CenterStageConstants.startPoseBlueBackStage;
            startAwayFromTheWall = drive.trajectoryBuilder(startPose)
                    //.splineTo(new Vector2d(-36, 42),Math.toRadians(90))
                    .forward(4)
                    .build();
            startToPark = drive.trajectoryBuilder(startAwayFromTheWall.end())
                    //.splineTo(new Vector2d(-36, -42),Math.toRadians(270))
                    .strafeLeft(48)
                    .addDisplacementMarker(()->intake.ejectPixel())
                    .build();
        }
        if (teamColor == 1 && startingPosition == 1) {
            startPose = CenterStageConstants.startPoseBlueFrontStage;
            startAwayFromTheWall = drive.trajectoryBuilder(startPose)
                    //.splineTo(new Vector2d(-36, 42),Math.toRadians(90))
                    .forward(4)
                    .build();
            startToPark = drive.trajectoryBuilder(startAwayFromTheWall.end())
                    .strafeLeft(96)
                    .addDisplacementMarker(()->intake.ejectPixel())
                    .build();
        }
        drive.setPoseEstimate(startPose);
        returnValue.add(startAwayFromTheWall);
        returnValue.add(startToPark);



        return returnValue;





        //Place a pixel
        /*Pose2d startPose  = null;
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
       Trajectory park = null;

        if (teamColor == 0 && startingPosition == 0) {
            //Red Backstage
            startToPixel = drive.trajectoryBuilder(startPose)
                    .splineTo(new Vector2d(-36,-66),Math.toRadians(90))
                    .addDisplacementMarker(()->intake.ejectPixel())
                    .build();
            //pixelToBackdrop =
            backdropToParking = drive.trajectoryBuilder(startToPixel.end())
                    .splineTo(new Vector2d(-36,-35),Math.toRadians(90))
                    .splineTo(new Vector2d(-36,-36),Math.toRadians(90))
                            .build();
            returnValue.add(startToPixel);
            //returnValue.add(pixelToBackdrop);
            returnValue.add(backdropToParking);
         }
        if (teamColor == 0 && startingPosition == 1) {
            //red frontstage
            startToPixel = drive.trajectoryBuilder(startPose)
                    .splineTo(new Vector2d(-36, -42),Math.toRadians(90))
                    .addDisplacementMarker(()->intake.ejectPixel())
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
                    .splineTo(new Vector2d(48, -56),Math.toRadians(0))
                    .splineTo(new Vector2d(56, -56),Math.toRadians(0))

                    //.splineTo(new Vector2d(36,-12),Math.toRadians(0))
                    //.splineTo(new Vector2d())
                    .build();
            returnValue.add(startToPixel);
            returnValue.add(turnAfterSpike);
            returnValue.add(pixelToBackdrop);
            returnValue.add(backdropToParking);
        }
        if (teamColor == 1 && startingPosition == 0) {
            //Backstage Blue
            startToPixel = drive.trajectoryBuilder(startPose)
                    .splineTo(new Vector2d(36, 42),Math.toRadians(90))
                    .addDisplacementMarker(()->intake.ejectPixel())
                    .build();
            turnAfterSpike = drive.trajectoryBuilder(startToPixel.end())
                    .lineToLinearHeading(new Pose2d(42,42,Math.toRadians(0)))
                    .build();
            pixelToBackdrop = drive.trajectoryBuilder(turnAfterSpike.end())
                    .lineToLinearHeading(new Pose2d(48, 42,Math.toRadians(0)))
                    .build();
            backdropToParking = drive.trajectoryBuilder(pixelToBackdrop.end())
                    .splineTo(new Vector2d(36,12),Math.toRadians(90))
                    .splineTo(new Vector2d(60,12),Math.toRadians(90))
                    .build();
            returnValue.add(startToPixel);
            returnValue.add(turnAfterSpike);
            returnValue.add(pixelToBackdrop);
            returnValue.add(backdropToParking);

            park = drive.trajectoryBuilder(startPose)
                    .strafeLeft(48)
                    .build();

        }
        if (teamColor == 1 && startingPosition == 1) {
            //Frontstage Blue
            startToPixel = drive.trajectoryBuilder(startPose)
                    .splineTo(new Vector2d(-36, 42),Math.toRadians(90))
                    .addDisplacementMarker(()->intake.ejectPixel())
                    .build();
            turnAfterSpike = drive.trajectoryBuilder(startToPixel.end())
                    .lineToLinearHeading(new Pose2d(42,42,Math.toRadians(0)))
                    .build();
            pixelToBackdrop = drive.trajectoryBuilder(turnAfterSpike.end())
                    .lineToLinearHeading(new Pose2d(48, 42,Math.toRadians(0)))
                    //.splineTo(new Vector2d(36,-12),Math.toRadians(0))
                    //.splineTo(new Vector2d())
                    .build();
            backdropToParking = drive.trajectoryBuilder(pixelToBackdrop.end())
                    .splineTo(new Vector2d(36,12),Math.toRadians(0))
                    .splineTo(new Vector2d(54,12),Math.toRadians(90))
                    .build();
            returnValue.add(park);
            /*returnValue.add(startToPixel);
            returnValue.add(turnAfterSpike);
            returnValue.add(pixelToBackdrop);
            returnValue.add(backdropToParking);*/
      //  }
        //startToPixel add magical drop pixel Code here:




        //go to boards
        //go to start
        //go to boards;
        //return returnValue;
    }








}
