package org.firstinspires.ftc.teamcode.drive.ebotsmanip;

import static org.firstinspires.ftc.teamcode.drive.opmode.CenterStageAutonFlint.startPosition;
import static org.firstinspires.ftc.teamcode.drive.opmode.CenterStageAutonFlint.teamElementPosition;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class CameraRedBlue {
    public enum TEAM_ELEMENT_POSITION{
        LeftSpike,
        MiddleSpike,
        RightSpike
    }

    public enum TEAM_START_POSITION {
        RedLeft,
        RedRight,
        BlueLeft,
        BlueRight
    }
    TEAM_START_POSITION teamStartPosition = null;

//Set all position based on selected staring location and Build Autonomous Trajectory
//Use this field layout for X and Y

    // -X BlueR BlueL +X
//|-------------------------------------------| +Y
//| |
//| 3 1 3 1 | 1
//| 2 2 | 2 Backdrop
//| | 3
//| |
//| 0,0 |
//| |
//| |
//| | 1
//| 2 2 | 2 Backdrop
//| 1 3 1 3 | 3
//| |
//|-------------------------------------------| -Y
// RedL RedR
//
//Use this compass for angles to match field above
// 90
// 180 0
// 270
//
    public static class CenterStagePipeline extends OpenCvPipeline
    {
        public enum TeamElementPosition {
            LeftSpike,
            CenterSpike,
            RightSpike
        }
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

       // TODO: Use this section to move the 3 regions of interest top left corners to where the team
        //elements are from your cameras’ perspective. The boxes are currently set at 20 x 20 pixels
        //which you can adjust below. Ideally the region of interest position should see the gray mat when
        //there is no element or the element color when the element is there.
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(20,98);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(181,68);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(300,98);
        static final int REGION_WIDTH = 20;
        static final int REGION_HEIGHT = 20;
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Mat region1_Cb, region2_Cb, region3_Cb; //For detecting blue (and red)
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1, avg2, avg3;
        private volatile TeamElementPosition position = TeamElementPosition.LeftSpike;
        void inputToCb(Mat input)
        {
          /*  This converts over from RGB to YCrCb which give you a channel Cr (Red) or Cb (Blue). Below,
                even through I am putting it into Cb the index of 1 or 2 will pick either Red for Index 1, or
            Blue for Index 2. The original example I started from only looked for Blue… YCrCb is supposed
            to be much less sensitive to the lighting at the event.
*/
                Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            switch (startPosition) {
                case RedLeft:
                    Core.extractChannel(YCrCb, Cb, 1) //Look for strongest Red
                    ;
                    break;
                case RedRight:
                    Core.extractChannel(YCrCb, Cb, 1) //Look for strongest Red
                    ;
                    break;
                case BlueLeft:
                    Core.extractChannel(YCrCb, Cb, 2) //Look for strongest Blue
                    ;
                    break;
                case BlueRight:
                    Core.extractChannel(YCrCb, Cb, 2) //Look for strongest Blue
                    ;
                    break;
            }
        }
        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);
            region1_Cb = Cb.submat(new Rect(region1_pointA,region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA,region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA,region3_pointB));
        }
       // This next draws a box on the image at the driver station for the 3 regions of interest so you
        //know where it is looking for troubleshooting. From the 3 vertical dots select ‘Camera Stream’
        //to see them
        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);
            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];
//Draw some rectangles
            Imgproc.rectangle(
                    input, //Buffer to draw on
                    region1_pointA,
                    region1_pointB,
                    BLUE,
                    2);
            Imgproc.rectangle(
                    input, //Buffer to draw on
                    region2_pointA,
                    region2_pointB,
                    BLUE,
                    2);
            Imgproc.rectangle(
                    input, //Buffer to draw on
                    region3_pointA,
                    region3_pointB,
                    BLUE,
                    2);
//Find the max of the 3 areas
          /*  This final section finds the maximum of the 3, then sees which of the 3 the maximum is and
            colors in the region of interest on the Driver Station GREEN so you can see what OpenCV has
            selected. We haven’t cleaned up the final code, I don’t think you need both teamElementPosition
            and position.
            */
            int maxOneTwo = Math.max(avg1, avg2);
            int max = Math.max(maxOneTwo, avg3);
//Find out which region has the max
            if(max == avg1)
            {
                teamElementPosition = TEAM_ELEMENT_POSITION.LeftSpike;
                position = TeamElementPosition.LeftSpike;

                Imgproc.rectangle(
                        input, region1_pointA, region1_pointB, GREEN, -1);
            }
            if(max == avg2)
            {
                teamElementPosition = TEAM_ELEMENT_POSITION.MiddleSpike;
                position = TeamElementPosition.CenterSpike;
                Imgproc.rectangle(
                        input, region2_pointA, region2_pointB, GREEN, -1);
            }
            if(max == avg3)
            {
                teamElementPosition = TEAM_ELEMENT_POSITION.RightSpike;
                position = TeamElementPosition.RightSpike;
                Imgproc.rectangle(
                        input, region3_pointA, region3_pointB, GREEN, -1);
            }
            return input;
        }
        public TeamElementPosition getAnalysis()
        {
            return position;
        }
    }
}
