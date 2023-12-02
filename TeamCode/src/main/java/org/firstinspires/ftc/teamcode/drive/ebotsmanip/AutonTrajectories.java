package org.firstinspires.ftc.teamcode.drive.ebotsmanip;

import static org.firstinspires.ftc.teamcode.drive.ebotsmanip.AutonConfigBackup.startingPosition;
import static org.firstinspires.ftc.teamcode.drive.ebotsmanip.CameraRedBlue.TEAM_START_POSITION.BlueLeft;
import static org.firstinspires.ftc.teamcode.drive.ebotsmanip.CameraRedBlue.TEAM_START_POSITION.RedRight;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.CenterStageConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;
import java.util.List;

@Config
public class AutonTrajectories {





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

    public AutonTrajectories(Telemetry telemetry, Gamepad gamepad) {
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
    public List<Trajectory> getAutonTrajectories(SampleMecanumDrive drive,Intake intake, Lift pixelArm, CameraRedBlue.TEAM_START_POSITION teamStartPosition, CameraRedBlue.TEAM_ELEMENT_POSITION elementPosition) {
        List<Trajectory> returnValue = new ArrayList<>();
        final int cMult = red ? 1 : -1;
        Pose2d startPose = null;
        switch(teamStartPosition) {
            case RedLeft:
                startPose = CenterStageConstants.startPoseRedBackStage;
                break;
            case RedRight:
                startPose = CenterStageConstants.startPoseRedFrontStage;
                break;
            case BlueLeft:
                startPose = CenterStageConstants.startPoseBlueFrontStage;
                break;
            case BlueRight:
                startPose = CenterStageConstants.startPoseBlueBackStage;
        }
        int spikeOffset = 0;
        if (elementPosition == CameraRedBlue.TEAM_ELEMENT_POSITION.LeftSpike) spikeOffset = -1;
        if (elementPosition == CameraRedBlue.TEAM_ELEMENT_POSITION.RightSpike) spikeOffset = 1;
            //Do Offset math here
        Trajectory startToSpike = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(startPose.getX(), cMult*(-37 + Math.abs(2 - 2) * 7), cMult*Math.toRadians(90/*270*/ - 2 * 90)))
                .addDisplacementMarker(()->{intake.ejectPixel();})
                .addTemporalMarker(2,()->{intake.powerServos(false);})
                .build();
        returnValue.add(startToSpike);
        Trajectory spikeToPixel = drive.trajectoryBuilder(startToSpike.end())
                .splineTo(new Vector2d(startPose.getX()+6,cMult*(-58)),Math.toRadians(0))
                .splineTo(new Vector2d(24,cMult*(-58)),Math.toRadians(0))
                .splineTo(new Vector2d(48,cMult*(-36)+-1*spikeOffset*CenterStageConstants.DistanceBetweenQrCodes),Math.toRadians(0))
                .addDisplacementMarker(()-> {pixelArm.pullArm(0);})
                .addTemporalMarker(7,()->{pixelArm.loadPixels();})
                .build();
        returnValue.add(spikeToPixel);
        if (teamStartPosition == RedRight || teamStartPosition == BlueLeft) {
            Trajectory pixelToPark = drive.trajectoryBuilder(spikeToPixel.end())
                    .strafeTo(new Vector2d(spikeToPixel.end().getX()-3,spikeToPixel.end().getY()))
                   // .strafeTo(new Vector2d(48,cMult*-12))
                    //.strafeTo(new Vector2d(58,cMult*-12))
                    .build();
                    returnValue.add(pixelToPark);
        }


//        Pose2d startPose = new Pose2d(-36, cMult*(-66), cMult*Math.toRadians(90));
//        //left
//        Trajectory startToSpikeL = drive.trajectoryBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(-36, cMult*(-37 + Math.abs(1 - 2) * 7), cMult*Math.toRadians(90/*270*/ - 1 * 90)))
//                //.lineToLinearHeading(new Pose2d(-36, cMult*(-58),cMult*Math.toRadians(0)))
//                .addDisplacementMarker(()->intake.ejectPixel())
//                .build();
//        Trajectory spikeToPixelL = drive.trajectoryBuilder(startToSpikeL.end())
//                .lineToLinearHeading(new Pose2d(47, cMult*(-58), Math.toRadians(0)))
//                //.lineToLinearHeading(new Pose2d(47, cMult*(-24 - 1 * 6), Math.toRadians(0)))
//                //.lineToLinearHeading(new Pose2d(48, cMult*(-24 - 1 * 6), Math.toRadians(0)))
//                //.addDisplacementMarker(()->pixelArm.placePixelRow1)
//
//                .build();
//        Trajectory pixelToParkL = drive.trajectoryBuilder(spikeToPixelL.end())
//                .lineToLinearHeading(new Pose2d(48, cMult*(-56)/* + parkingPos * 24*/, Math.toRadians(0)))
//                //.lineToLinearHeading(new Pose2d(56, cMult*(-56)/*+parkingPos * 24*/, Math.toRadians(0)))
//                .build();
//        //center
//        Trajectory startToSpikeC = drive.trajectoryBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(-36, cMult*(-37 + Math.abs(2 - 2) * 7), cMult*Math.toRadians(90/*270*/ - 2 * 90)))
//                .addDisplacementMarker(()->{intake.ejectPixel();})
//                .addTemporalMarker(2,()->{intake.powerServos(false)})
//                //.lineToLinearHeading(new Pose2d(47, cMult*(-35 + Math.abs(2-2) * 7),cMult*Math.toRadians(270-2*90)))
//                .build();
//        Trajectory spikeToPixelC = drive.trajectoryBuilder(startToSpikeC.end())
//                .splineTo(new Vector2d(-30,cMult*(-58)),Math.toRadians(0))
//                .splineTo(new Vector2d(24,cMult*(-58)),Math.toRadians(0))
//                .splineTo(new Vector2d(48,cMult*(-36)),Math.toRadians(0))
//                //.lineToLinearHeading(new Pose2d(-36, cMult*(-61), cMult*Math.toRadians(90/*270*/ - 2 * 90)))
//                //.lineToLinearHeading(new Pose2d(47, cMult*(-61), Math.toRadians(0)))
//
//                //.lineToLinearHeading(new Pose2d(47, cMult*(-24 - 2 * 6), Math.toRadians(0)))
//                //.lineToLinearHeading(new Pose2d(48, cMult*(-24 - 2 * 6), Math.toRadians(0)))
//                //.addDisplacementMarker(()->pixelArm.placePixelRow1)
//                .build();
//        Trajectory pixelToParkC = drive.trajectoryBuilder(spikeToPixelC.end())
//                .lineToLinearHeading(new Pose2d(48, cMult*(-56)/* + parkingPos * 24*/, Math.toRadians(0)))
//                //.lineToLinearHeading(new Pose2d(56, cMult*(-56)/*+parkingPos * 24*/, Math.toRadians(0)))
//                .build();
//        //right
//        Trajectory startToSpikeR = drive.trajectoryBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(-36, cMult*(-37 + Math.abs(3 - 2) * 7), cMult*Math.toRadians(90/*270*/ - 3 * 90)))
//                .addDisplacementMarker(()->intake.ejectPixel())
//                //.lineToLinearHeading(new Pose2d(-36, cMult*(-58),cMult*Math.toRadians(0)))
//                .build();
//        Trajectory spikeToPixelR = drive.trajectoryBuilder(startToSpikeR.end())
//                .lineToLinearHeading(new Pose2d(47, cMult*(-58), Math.toRadians(0)))
//                //.lineToLinearHeading(new Pose2d(47, cMult*(-24 - 3 * 6), Math.toRadians(0)))
//                //.lineToLinearHeading(new Pose2d(48, cMult*(-24 - 3 * 6), Math.toRadians(0)))
//                //.addDisplacementMarker(()->pixelArm.placePixelRow1)
//
//                .build();
//        Trajectory pixelToParkR = drive.trajectoryBuilder(spikeToPixelR.end())
//                .lineToLinearHeading(new Pose2d(48, cMult*(-56)/* + parkingPos * 24*/, Math.toRadians(0)))
//                //.lineToLinearHeading(new Pose2d(56, cMult*(-56)/*+parkingPos * 24*/, Math.toRadians(0)))
//                .build();
//        //Trajectory startAwayFromTheWall = null;
//
//        //drive.trajectorySequenceBuilder(startPose)
//                //startToPixel
//                //.lineToLinearHeading(new Pose2d(-36, cMult*(-36), Math.toRadians(0)))
//
//                //.addDisplacementMarker(()->intake.ejectPixel())
//                //turnAfterSpike
//                //.lineToLinearHeading(new Pose2d(-42,-36,Math.toRadians(0)))
//                //pixelToBackdrop
//
//                //.splineTo(new Vector2d(36,-12),Math.toRadians(0))
//                //backdropToParking
//
//        drive.setPoseEstimate(startPose);
//        if (elementPosition == CameraRedBlue.TEAM_ELEMENT_POSITION.LeftSpike) {
//            returnValue.add(startToSpikeL);
//            //returnValue.add(spikeToPixelL);
//            //returnValue.add(pixelToParkL);
//        }
//        if (elementPosition == CameraRedBlue.TEAM_ELEMENT_POSITION.MiddleSpike) {
//            returnValue.add(startToSpikeC);
//            returnValue.add(spikeToPixelC);
//            //returnValue.add(pixelToParkC);
//        }
//        if (elementPosition == CameraRedBlue.TEAM_ELEMENT_POSITION.RightSpike) {
//            returnValue.add(startToSpikeR);
//            //returnValue.add(spikeToPixelR);
//            //returnValue.add(pixelToParkR);
//        }
        return returnValue;
    }
}
