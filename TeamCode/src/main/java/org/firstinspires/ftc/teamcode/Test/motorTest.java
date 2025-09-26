package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@Disabled
@TeleOp(name = "motorTest", group = "Test")
public class motorTest extends LinearOpMode {

    private DcMotor motor = null;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.x){
                motor.setPower(1);
            }else if(gamepad1.y){
                motor.setPower(-1);
            }else{
                motor.setPower(0);
            }
            telemetry.addData("Motor Power", motor.getPower());
            telemetry.update();
        }
    }
}