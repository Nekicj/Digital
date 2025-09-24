package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utils.asmPIDController;

@Config
public class BaseController {
    public static double targetHeading = 0;
    private asmPIDController headingPID;

    public static double Kp = 0.05;
    public static double Ki = 0;
    public static double Kd = 0.01;



    DcMotor Lfront = null;
    DcMotor Rfront = null;
    DcMotor Rback = null;
    DcMotor Lback = null;

    GoBildaPinpointDriver pinpoint = null;



    public void initialize(HardwareMap hardwareMap){
        headingPID = new asmPIDController(Kp, Ki, Kd);
        headingPID.setTarget(targetHeading);
        headingPID.setTolerance(2);

        Lfront = hardwareMap.get(DcMotor.class,"lfd");
        Rfront = hardwareMap.get(DcMotor.class,"rfd");
        Lback = hardwareMap.get(DcMotor.class,"lbd");
        Rback = hardwareMap.get(DcMotor.class,"rbd");

        Lfront.setDirection(DcMotorSimple.Direction.REVERSE);
        Rfront.setDirection(DcMotorSimple.Direction.FORWARD);
        Lback.setDirection(DcMotorSimple.Direction.FORWARD);
        Rback.setDirection(DcMotorSimple.Direction.REVERSE);

//        Lfront.setRunMode(Motor.RunMode.VelocityControl);
//        Rfront.setRunMode(Motor.RunMode.VelocityControl);
//        Lback.setRunMode(Motor.RunMode.VelocityControl);
//        Rback.setRunMode(Motor.RunMode.VelocityControl);
//
//        Lfront.setFeedforwardCoefficients(kS,kV,kA);
//        Rfront.setFeedforwardCoefficients(kS,kV,kA);
//        Lback.setFeedforwardCoefficients(LBackkS,LBackkV,LBackkA);
//        Rback.setFeedforwardCoefficients(kS,kV,kA);
//
//        Lfront.setVeloCoefficients(KP, KI, KD);
//        Rfront.setVeloCoefficients(KP, KI, KD);
//        Lback.setVeloCoefficients(LBackKP, LBackKI, LBackKD);
//        Rback.setVeloCoefficients(KP, KI, KD);

        Lfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();
    }

    public void update(double leftX,double leftY,double rightX, double turnCoeff,boolean headingToTarget){
        pinpoint.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);

        driveFieldCentric(
                leftX,
                leftY,
                rightX /turnCoeff,
                pinpoint.getHeading(AngleUnit.RADIANS),
                headingToTarget
        );
    }



    public void driveFieldCentric(double strafeSpeed, double forwardSpeed,
                                  double turnSpeed, double gyroAngle,boolean headingLockEnabled) {

        strafeSpeed = clipRange(strafeSpeed);
        forwardSpeed = clipRange(forwardSpeed);
        turnSpeed = clipRange(turnSpeed);

        // --------------------------HEADINGLOCK----------------------
        if (headingLockEnabled) {
            double correction = headingPID.calculate(pinpoint.getHeading(AngleUnit.RADIANS));

            turnSpeed = correction;
        }

        double rotatedStrafe = strafeSpeed * Math.cos(-gyroAngle) - forwardSpeed * Math.sin(-gyroAngle);
        double rotatedForward = strafeSpeed * Math.sin(-gyroAngle) + forwardSpeed * Math.cos(-gyroAngle);

        double theta = Math.atan2(rotatedForward, rotatedStrafe);
        double magnitude = Math.sqrt(rotatedStrafe * rotatedStrafe + rotatedForward * rotatedForward);

        double[] wheelSpeeds = new double[4];
        wheelSpeeds[0] = Math.sin(theta + Math.PI / 4);
        wheelSpeeds[1] = Math.sin(theta - Math.PI / 4);
        wheelSpeeds[2] = Math.sin(theta - Math.PI / 4);
        wheelSpeeds[3] = Math.sin(theta + Math.PI / 4);

        normalizeWithMagnitude(wheelSpeeds, magnitude);

        wheelSpeeds[0] += turnSpeed;
        wheelSpeeds[1] -= turnSpeed;
        wheelSpeeds[2] += turnSpeed;
        wheelSpeeds[3] -= turnSpeed;

        normalize(wheelSpeeds);

        driveWithMotorPowers(wheelSpeeds[0], wheelSpeeds[1], wheelSpeeds[2], wheelSpeeds[3]);
    }

    private double clipRange(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }

    private void normalizeWithMagnitude(double[] wheelSpeeds, double magnitude) {
        double maxMagnitude = 0;
        for (double speed : wheelSpeeds) {
            maxMagnitude = Math.max(maxMagnitude, Math.abs(speed));
        }

        if (maxMagnitude > 0) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude) * magnitude;
            }
        }
    }

    private void normalize(double[] wheelSpeeds) {
        double maxMagnitude = 0;
        for (double speed : wheelSpeeds) {
            maxMagnitude = Math.max(maxMagnitude, Math.abs(speed));
        }

        if (maxMagnitude > 1.0) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] /= maxMagnitude;
            }
        }
    }
    public void setTargetHeading(){
        targetHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        headingPID.setTarget(targetHeading);
    }
    public void resetHeading(){
        pinpoint.setHeading(0,AngleUnit.RADIANS);
        pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();

    }

    private void driveWithMotorPowers(double frontLeft, double frontRight,
                                      double backLeft, double backRight) {
        Lfront.setPower(frontLeft);
        Rfront.setPower(frontRight);
        Lback.setPower(backLeft);
        Rback.setPower(backRight);
    }

    public void viewTelemetry(Telemetry telemetry){

        telemetry.addData("Target Heading",targetHeading);
        telemetry.addData("Angle in radian",pinpoint.getHeading(AngleUnit.RADIANS));
        telemetry.addData("Angle in degrees",pinpoint.getHeading(AngleUnit.DEGREES));

        telemetry.addData("x",pinpoint.getPosX(DistanceUnit.CM));
        telemetry.addData("y",pinpoint.getPosY(DistanceUnit.CM));
    }
}
