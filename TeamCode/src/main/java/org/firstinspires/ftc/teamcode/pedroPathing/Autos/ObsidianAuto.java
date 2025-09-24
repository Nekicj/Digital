package org.firstinspires.ftc.teamcode.pedroPathing.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "Obsidian Auto",group = "Competition Auto")
public class ObsidianAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer,acitionTimer,opModeTimer;
    private ElapsedTime niggTimer;

    private int pathState = 0;

    private final Pose startPose = new Pose(0,0,0);
    private final Pose scorePose = new Pose(0,0,0);
    private final Pose take1PosStart = new Pose(-2.37,13.33,0.74607);
    private final Pose take1PosEnd = new Pose(19.635,34,0.74607);
    private final Pose take2PosStart = new Pose(18.73,31.098,0.74607);
    private final Pose take2PosEnd = new Pose(1.3,34,0.746);

    public Path take1Path;
    public Path take2Path;

    public PathChain startToTake1;
    public PathChain take1toScore;
    public PathChain scoreToTake2;
    public PathChain take2ToScore;
    public void buildPaths(){
        startToTake1 = follower.pathBuilder()
                .addPath(new BezierCurve(startPose,take1PosStart))
                .setLinearHeadingInterpolation(startPose.getHeading(),take1PosStart.getHeading(),1)
                .build();


        take1Path = new Path(new BezierLine(take1PosStart,take1PosEnd));
        take1Path.setConstantHeadingInterpolation(take1PosStart.getHeading());
        //take1Path.setLinearHeadingInterpolation(take1PosStart.getHeading(),take1PosEnd.getHeading());

        take1toScore = follower.pathBuilder()
                .addPath(new BezierCurve(take1PosEnd,scorePose))
                .setLinearHeadingInterpolation(take1PosEnd.getHeading(),scorePose.getHeading(),1)
                .build();

        scoreToTake2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose,take2PosStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(),take2PosStart.getHeading(),1)
                .build();

        take2Path = new Path(new BezierLine(take2PosStart,take2PosEnd));
        take2Path.setConstantHeadingInterpolation(take2PosStart.getHeading());

        take2ToScore = follower.pathBuilder()
                .addPath(new BezierCurve(take2PosEnd,scorePose))
                .setLinearHeadingInterpolation(take2PosEnd.getHeading(),scorePose.getHeading(),1)
                .build();



    }

    @Override
    public void init(){
        pathTimer = new Timer();
        pathTimer.resetTimer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();

        niggTimer = new ElapsedTime();
        niggTimer.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();
    }

    public void pathUpdate(){
        switch (pathState){
            case 0:
                if(niggTimer.milliseconds() > 1000){
                    niggTimer.reset();
                    pathState = 1;
                }
                break;
            case 1:
                follower.followPath(startToTake1);
                pathState = 2;
                break;
            case 2:
                if(!follower.isBusy()){
                    follower.followPath(take1Path,false);
                    pathState = 3;
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    follower.followPath(take1toScore);
                    pathState = 4;
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    pathState = 5;
                    follower.setPose(scorePose);
                    niggTimer.reset();
                }
                break;
            case 5:
                if(niggTimer.milliseconds() > 1000){
                    niggTimer.reset();
                    follower.followPath(scoreToTake2);
                    pathState = 6;
                }
                break;
            case 6:
                if(!follower.isBusy()){
                    follower.followPath(take2Path);
                    pathState = 7;
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    follower.followPath(take2ToScore);
                    pathState = 8;
                }
                break;
            case 8:
                if(!follower.isBusy()){
                    follower.setPose(scorePose);

                }

        }
    }
    @Override
    public void loop(){
        follower.update();
        pathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("closest heading",follower.getClosestPointHeadingGoal());
        telemetry.addData("closestPosHeading",follower.getClosestPose().getPose().getHeading());
        telemetry.addData("closest X",follower.getClosestPose().getPose().getX());
        telemetry.addData("closest Y",follower.getClosestPose().getPose().getY());




        telemetry.update();
    }
}
