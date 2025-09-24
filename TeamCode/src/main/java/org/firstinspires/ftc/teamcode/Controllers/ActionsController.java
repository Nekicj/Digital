package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class ActionsController {
    private double intakeDirection = 0;
    private IntakeController intakeController;
    private ShooterController shooterController;
    private CommandScheduler intakeScheduler;

    private double intakeStage = -2;



    public ActionsController(HardwareMap hardwareMap){
        intakeController = new IntakeController();
        shooterController = new ShooterController();
        intakeScheduler = new CommandScheduler();
        intakeController.initialize(hardwareMap,"intake_1","intake_2");
        shooterController.initialize(hardwareMap,"shooter_l","shooter_r","l_angle","r_angle",ShooterController.ServosPos.DIRECTION_UP.getPos());
    }

    private boolean isShooting = false;

    public static double shooterSpeed = -1;

    public void update(boolean isBack){
        intakeScheduler.update();
    }

    // -2 stop intake
    // -1 bluet
    // 0 toUp
    // 1 piet

    public void intakeEpt(double intakeState){

        if(intakeStage == -2 && intakeState == -1){
            intakeStage = intakeState;
        }else if( intakeStage == -1 && intakeState == -1){
            intakeStage = -2;
        }
        else if(intakeStage == -2 && intakeState == 0){
            intakeStage = intakeState;
        }else if( intakeStage == 0 && intakeState == 0){
            intakeStage = -2;
        }
        else if(intakeStage == -2 && intakeState == 1){
            intakeStage = intakeState;
        }else if( intakeStage == 1 && intakeState == 1){
            intakeStage = -2;
        }else{
            intakeStage = intakeState;
        }



        if(intakeStage == -1){
            intakeController.setIntakePower(-1);
            intakeController.setSecIntakeMotor(1);

        }else if(intakeStage == 0){
            intakeController.setIntakePower(1);
            intakeController.setSecIntakeMotor(-1);
        }else if(intakeStage == 1){
            intakeController.setIntakePower(1);
            intakeController.setSecIntakeMotor(1);
        }else{
            intakeController.setIntakePower(0);
            intakeController.setSecIntakeMotor(0);
        }
    }

    public void setDirectionPos(double pos){
        shooterController.setDirectionPos(pos);
    }

    public void toShoot(){
        isShooting = !isShooting;
        if(isShooting){
            shooterController.setShooterPower(shooterSpeed);
        }else{
            shooterController.setShooterPower(0);
        }
    }



//    public void toTakeSpecimen(){
//
//        outtakeScheduler.clearQueue();
//        outtakeScheduler.setAutoReset(false);
//
//        outtakeScheduler.scheduleCommand(()->  liftController.setTargetPosition(LiftController.Position.SPECIMEN_TAKE.getPos()));
//        outtakeScheduler.scheduleCommand(outtakeController::setOuttakeToTake);
//        outtakeScheduler.scheduleCommand(outtakeController::setClawRotateToTake);
//        outtakeScheduler.scheduleCommand(outtakeController::setClawOpen);
//
//        outtakeScheduler.start();
//
//    }
}
