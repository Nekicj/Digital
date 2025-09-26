package org.firstinspires.ftc.teamcode.pedroPathing.Autos;

import com.acmerobotics.dashboard.config.Config;
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

import org.firstinspires.ftc.teamcode.Controllers.ActionsController;
import org.firstinspires.ftc.teamcode.Controllers.ShooterController;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@Autonomous(name = "12+0 with human",group = "Competition Auto")
public class ObsidianNHumanPlayerAuto extends OpMode {
    private ActionsController actionsController;
    private Follower follower;
    private Timer pathTimer,acitionTimer,opModeTimer;
    private ElapsedTime niggTimer;

    public static boolean isLongScore = false;

    private int pathState = 0;

    private int ballsCount = 0;


    private Pose scorePose = new Pose(34.801,4.545,-2.35);

    private final Pose startPose = new Pose(14.238,-18.086,-2.333);

    private final Pose closeScorePose = new Pose(26.461,-4.759,-2.339);
    private final Pose longScore = new Pose(54.5,27.5,-2.3);

    private final Pose take1PosStart = new Pose(39.472,2.1406,-1.59);
    private final Pose take1PosEnd = new Pose(39.472,-26.2,-1.59);

    private final Pose take2PosStart = new Pose(64.5,2.1406,-1.59);
    private final Pose take2PosEnd = new Pose(64.5,-26.2,-1.59);

    // -1820 -1600 long score

    // -1420 -1200-1220 close score

    private double targetVelocityToCheck = -1820;
    private double offset = -270;

    public Path take1Path;
    public Path take2Path;

    public PathChain startToScore;
    public PathChain scoreToTake1;
    public PathChain take1toScore;
    public PathChain scoreToTake2;
    public PathChain take2ToScore;
    public void buildPaths(){

        startToScore = follower.pathBuilder()
                .addPath(new BezierCurve(startPose,scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(),scorePose.getHeading(),1)

                .build();

        scoreToTake1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose,take1PosStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(),take1PosStart.getHeading(),1)
                .build();


        take1Path = new Path(new BezierLine(take1PosStart,take1PosEnd));
        take1Path.setConstantHeadingInterpolation(take1PosStart.getHeading());
//        take1Path.setLinearHeadingInterpolation(scorePose.getHeading(),take1PosEnd.getHeading());

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
//        take2Path.setLinearHeadingInterpolation(scorePose.getHeading(),take2PosEnd.getHeading());

        take2ToScore = follower.pathBuilder()
                .addPath(new BezierCurve(take2PosEnd,scorePose))
                .setLinearHeadingInterpolation(take2PosEnd.getHeading(),scorePose.getHeading(),1)
                .build();



    }

    @Override
    public void init(){
        actionsController = new ActionsController(hardwareMap);

        if(!isLongScore){
            scorePose = closeScorePose;
            targetVelocityToCheck = -1320;
            offset = -150;
            actionsController.setShooterVelocity(targetVelocityToCheck);
            actionsController.setDirectionPos(ShooterController.ServosPos.DIRECTION_UP.getPos());
        }else{
            scorePose = longScore;
            targetVelocityToCheck = -1750;
            actionsController.setShooterVelocity(targetVelocityToCheck);
            actionsController.setDirectionPos(ShooterController.ServosPos.DIRECTION_DOWN.getPos());
            offset = -200;
        }

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
            case 0: // GO TO SCORE THE PRELOAD
                if(!follower.isBusy()){
                    follower.followPath(startToScore);
                    actionsController.intakeEpt(1);
                    actionsController.toShoot(true);
                    pathState = 1;
                }
                break;
            case 1: // SCORING
                if(!follower.isBusy()){
                    follower.breakFollowing();
                    follower.deactivateAllPIDFs();
                    pathState = 2;
                }
                break;
            case 2: // WAIT FOR SHOOTER AND GO TO TAKE 1
                if(actionsController.checkShooterVelocity(targetVelocityToCheck,offset) && niggTimer.milliseconds() > 650){
                    if(ballsCount <=3){
                        actionsController.shootBall();
                        niggTimer.reset();
                        ballsCount+=1;
                    }else{
                        actionsController.toShoot(false);
                        follower.activateAllPIDFs();
                        ballsCount = 0;
                        pathState = 3;
                    }
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    follower.followPath(scoreToTake1);
                    pathState = 4;
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    follower.followPath(take1Path);
                    pathState = 5;
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    follower.followPath(take1toScore);
                    actionsController.toShoot(true);
                    pathState = 6;
                }
                break;
            case 6:
                if(!follower.isBusy()){
                    follower.breakFollowing();
                    follower.deactivateAllPIDFs();
                    pathState = 7;
                }
                break;
            case 7:
                if(actionsController.checkShooterVelocity(targetVelocityToCheck,offset) && niggTimer.milliseconds() > 650){
                    if(ballsCount <=3){
                        actionsController.shootBall();
                        niggTimer.reset();
                        ballsCount+=1;
                    }else{
                        actionsController.toShoot(false);
                        follower.activateAllPIDFs();
                        ballsCount = 0;
                        pathState = 8;
                    }
                }
                break;
            case 8:
                if(!follower.isBusy()){
                    follower.followPath(scoreToTake2);
                    pathState = 9;
                }
                break;
            case 9:
                if(!follower.isBusy()){
                    follower.followPath(take2Path);
                    pathState = 10;
                }
                break;
            case 10:
                if(!follower.isBusy()){
                    follower.followPath(take2ToScore);
                    actionsController.toShoot(true);
                    pathState = 11;
                }
                break;
            case 11:
                if(!follower.isBusy()){
                    follower.breakFollowing();
                    follower.deactivateAllPIDFs();
                    pathState = 12;
                }
                break;
            case 12:
                if(actionsController.checkShooterVelocity(targetVelocityToCheck,offset) && niggTimer.milliseconds() > 650){
                    if(ballsCount <=3){
                        actionsController.shootBall();
                        niggTimer.reset();
                        ballsCount+=1;
                    }else{
                        actionsController.toShoot(false);
                        actionsController.intakeEpt(1);
                        follower.activateAllPIDFs();
                        ballsCount = 0;
                        pathState = 13;
                    }
                }
                break;

        }
    }
    @Override
    public void loop(){
        follower.update();
        pathUpdate();
        actionsController.update(false);

        telemetry.addData("path state", pathState);
        telemetry.addData("balls count",ballsCount);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("target - offset", targetVelocityToCheck - offset);
        actionsController.showShooterTelemetry(telemetry);
        telemetry.addData("true false:",actionsController.checkShooterVelocity(targetVelocityToCheck,offset));




        telemetry.update();
    }
}
