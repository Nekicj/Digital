package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ShooterController {
    private DcMotor shooterMotorLeft = null;
    private DcMotor shooterMotorRight = null;

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
        shooterMotorLeft = hardwareMap.get(DcMotor.class,shooterMotorLeftName);
        shooterMotorRight = hardwareMap.get(DcMotor.class,shooterMotorRightName);
        shooterMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        servol = hardwareMap.get(Servo.class,servoAngleLeftName);
        servor = hardwareMap.get(Servo.class,servoAngleRightName);
        servor.setDirection(Servo.Direction.REVERSE);

        servol.setPosition(pos);
        servor.setPosition(pos);

    }

    public void setShooterPower(double power){
        shooterMotorLeft.setPower(power);
        shooterMotorRight.setPower(power);
    }

    public void setDirectionPos(double setPos){
        directionPos = setPos;
        powDirectionPos();
    }

    public void powDirectionPos(){
        servol.setPosition(directionPos);
        servor.setPosition(directionPos);
    }
}
