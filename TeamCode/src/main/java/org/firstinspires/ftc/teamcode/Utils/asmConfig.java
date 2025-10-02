package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class asmConfig {
    public static int pattern = 0;
    public static boolean isRed = false;

    public static double motorVelocityClose = -1200;
    public static double motorOffsetClose = -20;

    public static double motorVelocityLong = -1500;
    public static double motorOffsetLong = -40;

    public static Pose scorePose = new Pose(34.801,4.545,-2.35);

    public static Pose startPose = new Pose(14.238,-18.086,-2.333);

    public static Pose closeScorePose = new Pose(26.461,-4.759,-2.339);
    public static Pose longScore = new Pose(54.5,27.5,-2.3);

    public static Pose take1PosStart = new Pose(39.472,2.1406,-1.59);
    public static Pose take1PosEnd = new Pose(39.472,-26.2,-1.59);

    public static Pose take2PosStart = new Pose(64.5,2.1406,-1.59);
    public static Pose take2PosEnd = new Pose(64.5,-26.2,-1.59);

    public static Pose take3PosStart = new Pose(86.763,2.1406,-1.59);
    public static Pose take3PosEnd = new Pose(86.763,-26.2,-1.59);

    public static Pose parkingPose = new Pose(101.744,67,-1.558);

    // ==================================RED==============================

    public static Pose scorePoseRed = new Pose(34.801,4.545,-2.35);

    public static Pose startPoseRed = new Pose(14.238,-18.086,-2.333);

    public static  Pose closeScorePoseRed = new Pose(26.461,-4.759,-2.339);
    public static Pose longScoreRed = new Pose(54.5,27.5,-2.3);

    public static Pose take1PosStartRed = new Pose(39.472,2.1406,-1.59);
    public static Pose take1PosEndRed = new Pose(39.472,-26.2,-1.59);

    public static Pose take2PosStartRed = new Pose(64.5,2.1406,-1.59);
    public static Pose take2PosEndRed = new Pose(64.5,-26.2,-1.59);

    public static Pose take3PosStartRed = new Pose(86.763,2.1406,-1.59);
    public static Pose take3PosEndRed = new Pose(86.763,-26.2,-1.59);

    public static Pose parkingPoseRed = new Pose(101.744,67,-1.558);
    // 0 - GPP
    // 1 - PGP
    // 2 - PPG
}
