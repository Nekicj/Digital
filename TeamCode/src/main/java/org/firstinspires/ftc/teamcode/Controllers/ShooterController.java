package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class ShooterController {
    private DcMotorEx shooterMotorLeft = null;
    private DcMotorEx shooterMotorRight = null;

    public static double kP = 15;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 15;

    private PIDFCoefficients pidfShooterCoeffs = new PIDFCoefficients(kP,kI,kD,kF);

    private Servo servol = null;
    private Servo servor = null;

    public static double shooter_up = 1;
    public static double shooter_down = 0;

    private double directionPos = 0.7;



    public static enum ServosPos{
        DIRECTION_DOWN(0.8),
        DIRECTION_UP(0.4);


        private final double position;
        ServosPos(double pos) {this.position = pos;}


        public double getPos() {return position;}

    }

    public void initialize(HardwareMap hardwareMap, String shooterMotorLeftName, String shooterMotorRightName, String servoAngleLeftName, String servoAngleRightName,double pos){
        shooterMotorLeft = hardwareMap.get(DcMotorEx.class,shooterMotorLeftName);
        shooterMotorRight = hardwareMap.get(DcMotorEx.class,shooterMotorRightName);

        shooterMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooterMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterMotorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfShooterCoeffs);
        shooterMotorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfShooterCoeffs);


        shooterMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        servol = hardwareMap.get(Servo.class,servoAngleLeftName);
        servor = hardwareMap.get(Servo.class,servoAngleRightName);
        servor.setDirection(Servo.Direction.REVERSE);

        servol.setPosition(pos);
        servor.setPosition(pos);

    }

    public void setShooterPower(double power){
        shooterMotorLeft.setVelocity(power);
        shooterMotorRight.setVelocity(power);
    }

    public void setDirectionPos(double setPos){
        directionPos = setPos;
        powDirectionPos();
    }

    public void powDirectionPos(){
        servol.setPosition(directionPos);
        servor.setPosition(directionPos);
    }
    public void showTelemetry(Telemetry telemetry,double targetVelocity){
        telemetry.addData("shooter L Velocity",shooterMotorLeft.getVelocity());
        telemetry.addData("shooter R Velocity",shooterMotorRight.getVelocity());

        telemetry.addData("TargetVelocity",targetVelocity);
    }

    public boolean checkVelocity(double targetVelocity,double offset){
        if(Math.abs(shooterMotorLeft.getVelocity()) >= Math.abs(targetVelocity -offset) ){
            return true;
        }else{
            return false;
        }
    }
}
