package org.firstinspires.ftc.teamcode.drive.ebotsmanip;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.drive.ebotsmanip.CameraRedBlue.TEAM_START_POSITION.BlueLeft;
import static org.firstinspires.ftc.teamcode.drive.ebotsmanip.CameraRedBlue.TEAM_START_POSITION.BlueRight;
import static org.firstinspires.ftc.teamcode.drive.ebotsmanip.CameraRedBlue.TEAM_START_POSITION.RedLeft;
import static org.firstinspires.ftc.teamcode.drive.ebotsmanip.CameraRedBlue.TEAM_START_POSITION.RedRight;
import static org.firstinspires.ftc.teamcode.drive.opmode.CenterStageAutonFlint.teamElementPosition;

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
public class AutonConfig {
    Telemetry telemetry;
    Gamepad gamepad;
    AutonTrajectories autonTrajectories;
    public CameraRedBlue.TEAM_START_POSITION teamStartPosition = null;
    public Pose2d startPose = null;
    public static int teamColor = 0; //0 == RED
    public static int startingPosition = 1; //0 == backstage by the boards
    public static int startingDelay = 0;
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
        if(teamColor == 0) {
            if (startingPosition == 0) {
                teamStartPosition = CameraRedBlue.TEAM_START_POSITION.RedLeft;
            } else {
                teamStartPosition = RedRight;
            }
        } else {
            if (startingPosition == 0) {
                teamStartPosition = BlueLeft;
            } else {
                teamStartPosition = CameraRedBlue.TEAM_START_POSITION.BlueRight;
            }
        }
        return false;

    }

    //returns a list of trajectories to be executed in order.
    public List<Trajectory> generateAutonTrajectories(SampleMecanumDrive drive,Intake intake, Lift pixelArm) {
        List<Trajectory> returnValue = new ArrayList<>();
        CameraRedBlue.TEAM_START_POSITION startPosition = null;


        //Pose2d startPose  = null;
        Trajectory startToPark = null;
        Trajectory startAwayFromTheWall = null;
        if (teamColor == 0 && startingPosition == 0) {
            startPose = CenterStageConstants.startPoseRedBackStage;
            startPosition = RedLeft;

        }
        if (teamColor == 0 && startingPosition == 1) {
            startPose = CenterStageConstants.startPoseRedFrontStage;
            startPosition = RedRight;
        }
        if (teamColor == 1 && startingPosition == 0) {
            startPose = CenterStageConstants.startPoseBlueBackStage;
            startPosition = BlueRight;
        }
        if (teamColor == 1 && startingPosition == 1) {
            startPose = CenterStageConstants.startPoseBlueFrontStage;
            startPosition = BlueLeft;
        }
        drive.setPoseEstimate(startPose);
        //returnValue.add(startAwayFromTheWall);
        //returnValue.add(startToPark);
        autonTrajectories = new AutonTrajectories(telemetry,gamepad1);
        for (Trajectory traj : autonTrajectories.getAutonTrajectories(drive, intake, pixelArm, startPosition, teamElementPosition)) {
            returnValue.add(traj);
        }


        return returnValue;
    }
}
