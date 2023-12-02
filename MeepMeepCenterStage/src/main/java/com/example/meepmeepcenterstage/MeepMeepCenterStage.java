package com.example.meepmeepcenterstage;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepCenterStage {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        int elementPos = 2;
        boolean red = true;
        final int cMult;
        int cMult1 = 0;
        if (red) cMult1 = 1;
        if (!red) cMult1 = -1;
        cMult = cMult1;
        int frontStage = 1;
        int throughDoor = 0;
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(13, 20)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(44.84, 28, Math.toRadians(224.02104812767092), Math.toRadians(184.02607784577722), 11.3)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(/*starting position*/ new Pose2d(-36, cMult*(-66), cMult*Math.toRadians(90)))
                                //startToPixel
                                .lineToLinearHeading(new Pose2d(-36, cMult*(-35 + Math.abs(elementPos - 2) * 7), cMult*Math.toRadians(270 - elementPos * 90)))
                                .lineToLinearHeading(new Pose2d(-36, cMult*(-58+48*throughDoor),Math.toRadians(0)))
                                //center: replace above with .lineToLinearHeading(new Pose2d(47, cMult*(-35 + Math.abs(elementPos-2) * 7),cMult*Math.toRadians(270-elementPos*90)))
                                //.lineToLinearHeading(new Pose2d(-36, cMult*(-36), Math.toRadians(0)))

                                //.addDisplacementMarker(()->intake.ejectPixel())
                                //turnAfterSpike
                                //.lineToLinearHeading(new Pose2d(-42,-36,Math.toRadians(0)))
                                //pixelToBackdrop
                                .lineToLinearHeading(new Pose2d(47, cMult*(-58+48*throughDoor), Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(48, cMult*(-24 - elementPos * 6), Math.toRadians(0)))
                                //.splineTo(new Vector2d(36,-12),Math.toRadians(0))
                                //backdropToParking
                                .lineToLinearHeading(new Pose2d(48, cMult*(-56)/* + parkingPos * 24*/, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(56, cMult*(-56)/*+parkingPos * 24*/, Math.toRadians(0)))
                                .build()
                );


        RoadRunnerBotEntity myOtherBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setDimensions(13, 20)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(44.84, 28, Math.toRadians(224.02104812767092), Math.toRadians(184.02607784577722), 11.3)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(/*starting position*/ new Pose2d(-36, cMult*(-66), cMult*Math.toRadians(90)))
                                //startToPixel
                                .lineToLinearHeading(new Pose2d(-36, cMult*(-35 + Math.abs(elementPos - 2) * 7), cMult*Math.toRadians(90 - elementPos * 90)))
                                .splineTo(new Vector2d(-30,cMult*(-58+48*throughDoor)),Math.toRadians(0))
                                .splineTo(new Vector2d(24,cMult*(-58+48*throughDoor)),Math.toRadians(0))
                                .splineTo(new Vector2d(48,cMult*(-36)),Math.toRadians(0))

                                .splineToConstantHeading(new Vector2d(48,cMult*-12),Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(58,cMult*-12),Math.toRadians(0))
                                //.lineToLinearHeading(new Pose2d(-36, cMult*(-58+48*throughDoor),Math.toRadians(0)))
                                //center: replace above with .lineToLinearHeading(new Pose2d(47, cMult*(-35 + Math.abs(elementPos-2) * 7),cMult*Math.toRadians(270-elementPos*90)))
                                //.lineToLinearHeading(new Pose2d(-36, cMult*(-36), Math.toRadians(0)))

                                //.addDisplacementMarker(()->intake.ejectPixel())
                                //turnAfterSpike
                                //.lineToLinearHeading(new Pose2d(-42,-36,Math.toRadians(0)))
                                //pixelToBackdrop
                                //.lineToLinearHeading(new Pose2d(47, cMult*(-58+48*throughDoor), Math.toRadians(0)))
                                //.lineToLinearHeading(new Pose2d(48, cMult*(-24 - elementPos * 6), Math.toRadians(0)))
                                //.splineTo(new Vector2d(36,-12),Math.toRadians(0))
                                //backdropToParking
                                //.lineToLinearHeading(new Pose2d(48, cMult*(-56)/* + parkingPos * 24*/, Math.toRadians(0)))
                                //.lineToLinearHeading(new Pose2d(56, cMult*(-56)/*+parkingPos * 24*/, Math.toRadians(0)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(myOtherBot)
                .start();
    }
}
