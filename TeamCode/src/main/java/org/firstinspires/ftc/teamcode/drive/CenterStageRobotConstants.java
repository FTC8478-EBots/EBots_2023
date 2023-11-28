package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class CenterStageRobotConstants {

    public boolean USE_GOBILDA_DEADWHEELS = true;

    public static final double lowerArmTicksToDegrees = -29.3394547479;

    public static final double upperArmTicksToDegrees = 29.3394547479;
    //14.669727374

    public static final int lowerArmLengthInches = 12;

    public static final int upperArmLengthInches = 14;

    public static final int lowerArmAngleOffset = 15;
    public static final double backboardAngle = 66;

    public static final double wristFCAngleStart =-4;
    public static final double wristFCAngleStow = 0;

    public static final int upperArmAngleOffset = 0;

    public static final double BASEMOTORPREPAREHANGINGPOSITION = 46 * lowerArmTicksToDegrees;

    public static final double baseDistanceOffGround = 23/4;
    public static final double ARMMOTORPREPAREHANGINGPOSITION = 109 * upperArmTicksToDegrees;

    public static final int BASEMOTORHANGINGPOSITION = 000;


    public static final int ARMMOTORHANGINGPOSITION = 000;

    public static final int[] pixelRowsLower = {100,200,300,400,500,600,700,800,900,1000,1100};
    public static final int[] pixelRowsUpper = {200,400,600,800,1000,1200,1400,1600,1800,2000,2200};


    public static final double[] rowHeightsIn = {7.25, 9.5, 11.75, 14, 16.25, 18.5, 20.75, 23, 25.25, 27.5, 29.75};


    public static int SPIKE_LEFT_CUTOFF = 200;
    public static int SPIKE_RIGHT_CUTOFF = 400;


}
