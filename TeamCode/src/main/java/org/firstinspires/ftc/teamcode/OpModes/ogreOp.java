package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.ActionsController;
import org.firstinspires.ftc.teamcode.Controllers.BaseController;
import org.firstinspires.ftc.teamcode.Controllers.ShooterController;
import org.firstinspires.ftc.teamcode.Utils.asmConfig;
import org.firstinspires.ftc.teamcode.Utils.asmGamepadEx;


@Config
@TeleOp(name = "Solo TelePopus",group = "Competition")
public class ogreOp extends LinearOpMode {
    private ActionsController actionsController;
    private BaseController baseController;
    private asmGamepadEx driver1;

    private double targetVelocityToCheck = asmConfig.motorVelocityClose;
    private double offset = asmConfig.motorOffsetClose;

    @Override
    public void runOpMode() {

        driver1 = new asmGamepadEx(gamepad1);
        actionsController = new ActionsController(hardwareMap);
        baseController = new BaseController();
        baseController.initialize(hardwareMap);


        telemetry.addData("Status, ","Initialized");
        waitForStart();

        while (opModeIsActive()){
            driver1.update();



            baseController.update(gamepad1.left_stick_x,-gamepad1.left_stick_y,gamepad1.right_stick_x,1,gamepad1.left_trigger > 0);

            if(driver1.isXPressed()){
                actionsController.toShoot();
            }

            if(driver1.isRightBumperPressed()){
                actionsController.intakeEpt(1);
            }
            if(driver1.isLeftBumperPressed()){
                actionsController.intakeEpt(-1);
            }
            if(driver1.isDpadUpPressed()){
                actionsController.intakeEpt(0);
            }
            if(driver1.isRightTriggerPressed(0.2)){
                actionsController.shootBall();
            }


            if(driver1.isRightStickButtonPressed()){
                actionsController.setDirectionPos(ShooterController.ServosPos.DIRECTION_UP.getPos());
                targetVelocityToCheck = asmConfig.motorVelocityClose;
                offset = asmConfig.motorOffsetClose;
                actionsController.setShooterVelocity(targetVelocityToCheck);
            }else if(driver1.isLeftStickButtonPressed()){
                targetVelocityToCheck = asmConfig.motorVelocityLong;
                offset = asmConfig.motorOffsetLong;
                actionsController.setShooterVelocity(targetVelocityToCheck);
                actionsController.setDirectionPos(ShooterController.ServosPos.DIRECTION_DOWN.getPos());
            }

            if(driver1.isBackPressed()){
                baseController.resetHeading();
            }

            if(driver1.isBPressed()){
                baseController.setTargetHeading();
            }

            if(actionsController.checkShooterVelocity(targetVelocityToCheck,offset)){
                gamepad1.rumble(0.1,0.1,50);
            }

//            actionsController.ravaPiet(gamepad1.right_bumper);
//
//            actionsController.ravaBluet(gamepad1.left_bumper);
//
//            actionsController.toUp(gamepad1.dpad_up);



            actionsController.update(gamepad2.back);

            telemetry.addData("Status", "Running");
            actionsController.showShooterTelemetry(telemetry);

            baseController.viewTelemetry(telemetry);
            telemetry.update();

        }
    }

}
