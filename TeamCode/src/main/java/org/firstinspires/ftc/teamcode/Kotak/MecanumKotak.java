package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name="DualShock Mecanum + Intake/Shooter + Mirrored Servos", group="Kotak")
public class MecanumKotak extends LinearOpMode {

    // База
    private DcMotor rfd, rbd, lfd, lbd;
    // Интейк
    private DcMotor intake1, intake2;
    // Шутер
    private DcMotor shooterR, shooterL;
    // Серво угла (зеркальные установки)
    private Servo rAngle, lAngle;

    // Параметры управления
    private static final double DRIVE_SLOW_TURN = 1.0;
    private static final double DRIVE_SLOW_MOVE = 1.0;

    // Плавное управление сервоприводами
    private static final double SERVO_MIN = 0.05;
    private static final double SERVO_MAX = 0.95;
    private static final double SERVO_MAX_RATE = 0.8;   // ед./сек (0..1)
    private static final double SERVO_DEADZONE = 0.05;

    private double servoPos = 0.5; // базовая «средняя» позиция
    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // ===== Имена устройств (как в вашем конфиге) =====
        rfd = hardwareMap.get(DcMotor.class, "rfd");
        rbd = hardwareMap.get(DcMotor.class, "rbd");
        lfd = hardwareMap.get(DcMotor.class, "lfd");
        lbd = hardwareMap.get(DcMotor.class, "lbd");

        intake1 = hardwareMap.get(DcMotor.class, "intake_1");
        intake2 = hardwareMap.get(DcMotor.class, "intake_2");

        shooterR = hardwareMap.get(DcMotor.class, "shooter_r");
        shooterL = hardwareMap.get(DcMotor.class, "shooter_l");

        rAngle = hardwareMap.get(Servo.class, "r_angle");
        lAngle = hardwareMap.get(Servo.class, "l_angle");

        // ===== Направления моторов =====
        // Механум: одинаковые знаки едут вперёд
        lfd.setDirection(DcMotorSimple.Direction.REVERSE);
        lbd.setDirection(DcMotorSimple.Direction.FORWARD);
        rfd.setDirection(DcMotorSimple.Direction.FORWARD);
        rbd.setDirection(DcMotorSimple.Direction.REVERSE);

        // Интейк: моторы стоят «друг напротив друга» -> противоположные направления
        // Выберите такую комбинацию, чтобы положительная мощность затягивала внутрь.
        intake1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Шутер: также «друг напротив друга» -> противоположные направления
        // Выберите комбинацию, чтобы положительная мощность выпускала мяч вперёд.
        shooterR.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterL.setDirection(DcMotorSimple.Direction.REVERSE);

        // ===== Поведение при нуле =====
        for (DcMotor m : new DcMotor[]{lfd, lbd, rfd, rbd}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Стартовые позиции серв (зеркальные!)
        setAngleServosMirrored(servoPos);

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();
        loopTimer.reset();

        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();

            // ===== МЕХАНУМ (gamepad1) =====
            double y  = -gamepad1.left_stick_y * DRIVE_SLOW_MOVE; // вперёд(+)
            double x  =  gamepad1.left_stick_x * DRIVE_SLOW_MOVE; // вправо(+)
            double rx =  gamepad1.right_stick_x * DRIVE_SLOW_TURN; // поворот вправо(+)

            // Квадратичное масштабирование для точности
            y  = Math.copySign(y * y, y);

            x  = Math.copySign(x * x, x);
            rx = Math.copySign(rx * rx, rx);

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double fl = (y + x + rx) / denominator;
            double bl = (y - x + rx) / denominator;
            double fr = (y - x - rx) / denominator;
            double br = (y + x - rx) / denominator;

            lfd.setPower(fl);
            lbd.setPower(bl);
            rfd.setPower(fr);
            rbd.setPower(br);

            // ===== ИНТЕЙК (gamepad1): RT - LT =====
            double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
            intake1.setPower(intakePower);
            intake2.setPower(intakePower);

            // ===== ШУТЕР (gamepad2): RT - LT =====
            double shooterPower = gamepad2.right_trigger - gamepad2.left_trigger;
            shooterR.setPower(shooterPower);
            shooterL.setPower(shooterPower);

            // ===== СЕРВО УГЛА (gamepad2 левый стик Y) с плавностью =====
            double stickY = -gamepad2.left_stick_y; // вверх — больше угол
            if (Math.abs(stickY) < SERVO_DEADZONE) stickY = 0.0;

            double delta = stickY * SERVO_MAX_RATE * dt;
            servoPos = clip(servoPos + delta, SERVO_MIN, SERVO_MAX);
            setAngleServosMirrored(servoPos); // зеркалим левое

            // ===== Телеметрия =====
            telemetry.addData("Drive", "FL: %.2f  FR: %.2f  BL: %.2f  BR: %.2f", fl, fr, bl, br);
            telemetry.addData("Intake (RT-LT)", "%.2f", intakePower);
            telemetry.addData("Shooter (RT-LT)", "%.2f", shooterPower);
            telemetry.addData("ServoPos (R=pos, L=1-pos)", "%.2f", servoPos);
            telemetry.update();
        }
    }

    private void setAngleServosMirrored(double pos) {
        // Правое серво = pos, левое серво = 1 - pos (зеркально)
        rAngle.setPosition(pos);
        lAngle.setPosition(1.0 - pos);
    }

    private static double clip(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}