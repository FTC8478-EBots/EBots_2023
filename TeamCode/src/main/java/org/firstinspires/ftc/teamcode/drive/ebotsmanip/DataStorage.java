package org.firstinspires.ftc.teamcode.drive.ebotsmanip;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class DataStorage {
        // See this static keyword? That's what lets us share the data between opmodes.
        public static Pose2d currentPose = new Pose2d();
        public static boolean alreadyInitialized = false;
}