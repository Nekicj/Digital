package org.firstinspires.ftc.teamcode.Kotak;

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

    private DcMotorEx motor = null;
    private double CPR = 383.6;
    private double lastPosition = 0;
    private double lastTime = 0;
    private double rpm = 0;

    @Override
    public void runOpMode() {
         motor = hardwareMap.get(DcMotorEx.class, "motor");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        lastPosition = motor.getCurrentPosition();
        lastTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            if (gamepad1.x) {
                motor.setPower(1);
            } else if (gamepad1.y) {
                motor.setPower(-1);
            } else {
                motor.setPower(0);
            }

            double currentPosition = motor.getCurrentPosition();
            double currentTime = System.currentTimeMillis();
            double deltaTime = (currentTime - lastTime) / 1000.0;
            double deltaPosition = currentPosition - lastPosition;

            if (deltaTime > 0) {
                double velocityTicksPerSecond = deltaPosition / deltaTime;
                rpm = (velocityTicksPerSecond / CPR) * 60;
            }

            lastPosition = currentPosition;
            lastTime = currentTime;

            telemetry.addData("Motor Power", motor.getPower());
            telemetry.addData("Encoder Position", currentPosition);
            telemetry.addData("RPM", rpm);
            telemetry.update();
        }
    }
}