package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.ebotsmanip.AutonConfigBackup;
import org.firstinspires.ftc.teamcode.drive.ebotsmanip.CameraRedBlue;
import org.firstinspires.ftc.teamcode.drive.ebotsmanip.DataStorage;
import org.firstinspires.ftc.teamcode.drive.ebotsmanip.Intake;
import org.firstinspires.ftc.teamcode.drive.ebotsmanip.Lift;
import org.firstinspires.ftc.teamcode.drive.ebotsmanip.TeamElementDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class CenterStageAutonFlint extends LinearOpMode {
    AutonConfigBackup config;
    SampleMecanumDrive drive;
    Intake intake;
    Telemetry telemetry;
    Lift pixelArm;
    public static CameraRedBlue.TEAM_ELEMENT_POSITION teamElementPosition = CameraRedBlue.TEAM_ELEMENT_POSITION.MiddleSpike;
    public static CameraRedBlue.TEAM_START_POSITION startPosition = CameraRedBlue.TEAM_START_POSITION.RedLeft;
    OpenCvWebcam webcam;
    CameraRedBlue.CenterStagePipeline pipeline;

@Override
    public void runOpMode() throws InterruptedException {
        /*Create your Subsystem Objects*/

    drive = new SampleMecanumDrive(hardwareMap);
    telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
    pixelArm = new Lift(hardwareMap,telemetry);
    intake = new Intake(hardwareMap);
    pixelArm.init(this);

    DataStorage.alreadyInitialized = true;

    config = new AutonConfigBackup(telemetry, gamepad1);

// Initiate Camera on Init.
        int cameraMonitorViewId =
                hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                        hardwareMap.appContext.getPackageName());
        //If you didn’t name the camera Webcam 1, change the name below
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
                "Webcam 1"), cameraMonitorViewId);
        pipeline = new CameraRedBlue.CenterStagePipeline();
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(5000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            //Camera stream is downsampled to 320x240 resolution regardless of the webcam resolution
            @Override
            public void onOpened() {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
            }
        });
        while (!isStopRequested() && !opModeIsActive()) {
            config.processUpdates();
            startPosition = config.teamStartPosition;
          //  telemetry.clearAll();
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Selected Starting Position", startPosition);
            telemetry.addData("Vision identified Team Element", teamElementPosition);
            telemetry.update();
        }
//Game Play Button is pressed
        if (opModeIsActive() && !isStopRequested()) {
//Build trajectory based on last detected target by vision
            config.generateAutonTrajectories(drive, intake,pixelArm);
            drive.getLocalizer().setPoseEstimate(config.startPose);
            //buildParking();
//run Autonomous trajectory
            //runAutoAndParking();
        }
    sleep(500);
    DataStorage.currentPose = drive.getPoseEstimate();

}
}
