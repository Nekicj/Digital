package org.firstinspires.ftc.teamcode.Kotak;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name="logitech april tag",group = "Competition")
@Config
public class LogitechAprilTagKotak extends LinearOpMode {

    private FtcDashboard ftcDashboard;


    @Override
    public void runOpMode(){
        ftcDashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry,ftcDashboard.getTelemetry());

        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class,"Webcam 1"))
                .setCameraResolution(new Size(640,480))
                .setAutoStopLiveView(true)
                .enableLiveView(true)
                .build();

        ftcDashboard.startCameraStream(visionPortal,30);

        waitForStart();
        while(opModeIsActive()){
            if(aprilTagProcessor.getDetections().size() > 0){
                AprilTagDetection tag = aprilTagProcessor.getDetections().get(0);
                telemetry.addData("P",tag.ftcPose.pitch);
                telemetry.addData("Y",tag.ftcPose.yaw);
                telemetry.addData("R",tag.ftcPose.roll  );


                telemetry.addData("X",tag.ftcPose.x);
                telemetry.addData("Y",tag.ftcPose.y);
                telemetry.addData("Z",tag.ftcPose.z);
                telemetry.update();

            }
        }
    }
}
