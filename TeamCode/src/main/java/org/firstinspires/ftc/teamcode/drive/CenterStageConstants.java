package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class CenterStageConstants {
    public static final Pose2d startPoseRedFrontStage = new Pose2d(-36, -66, Math.toRadians(90));
    public static final Pose2d startPoseRedBackStage = new Pose2d(12, -66, Math.toRadians(90));
    public static final Pose2d startPoseBlueFrontStage = new Pose2d(-36, 66, Math.toRadians(270));
    public static final Pose2d startPoseBlueBackStage = new Pose2d(12, 66, Math.toRadians(270));
    public static final Pose2d redTape = new Pose2d(-36,-42,Math.toRadians(90));
    public static final Pose2d blueTape = new Pose2d(-36,42,Math.toRadians(270));

    //public static final Pose2d tag1;
    public static final Pose2d tag2 = new Pose2d(48,42);
    //public static final Pose2d tag3;
    //public static final Pose2d tag4;
    public static final Pose2d tag5 = new Pose2d(48, -42);
    //public static final Pose2d tag6 = new Pose2d;
    public static final Pose2d SpikeLRedFront = new Pose2d(-42, -36, 0);
    public static final Pose2d SpikeCRedFront = new Pose2d(-36, -30, 0);

    public static final Pose2d SpikeRRedFront = new Pose2d(-42, -24, 0);

    //public static final Pose2d BlueTagLeft = new Pose2d()

    public static final int bluePixelX = 48;

    public static final int[] bluePixelY = {46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34};
}
