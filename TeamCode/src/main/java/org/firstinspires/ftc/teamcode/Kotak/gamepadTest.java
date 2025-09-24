package org.firstinspires.ftc.teamcode.Kotak;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utils.asmGamepadEx;


@Config
@TeleOp(name = "GamepadTest",group = "Test")
public class gamepadTest extends LinearOpMode {
    private asmGamepadEx driver1;

    private double count = 0;

    ElapsedTime elapsedTimer = new ElapsedTime();


    @Override
    public void runOpMode() {

        driver1 = new asmGamepadEx(gamepad1);


        telemetry.addData("Status, ","Initialized");
        elapsedTimer.reset();
        waitForStart();

        while (opModeIsActive()){
            driver1.update();
            if(driver1.isAPressed()){
                count +=1;
            }
            telemetry.addData("Status", "Running");
            telemetry.addData("count",count);
            telemetry.update();

        }
    }

}
