package org.firstinspires.ftc.teamcode.Kotak;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.ftccommon.SoundPlayer;
import android.media.MediaPlayer;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;
import java.io.IOException;

@Config
@TeleOp(name="Face HuskyLens")
public class HuskyFaceDetectKotak extends LinearOpMode {

    private HuskyLens huskyLens = null;
    private double targetTagId = -1;





    @Override
    public void runOpMode(){
        String pidorasPath = "/sdcard/FIRST/blocks/sounds/pidoras.mp3";
        MediaPlayer mediaPlayer = new MediaPlayer();

        try {
            mediaPlayer.setDataSource(pidorasPath);
            mediaPlayer.prepare();
        } catch (IOException e) {
            telemetry.addData("Error", "Failed to load sound file: " + e.getMessage());
            telemetry.update();
            sleep(2000);
            return;
        }

        huskyLens = hardwareMap.get(HuskyLens.class,"husky");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.FACE_RECOGNITION);

        telemetry.addData("Init","aa");
        telemetry.update();


        waitForStart();
        while(opModeIsActive()){

            HuskyLens.Block[] blocks = huskyLens.blocks();

            if (blocks.length > 0) {
                targetTagId = blocks[0].id;
                telemetry.addData("Tag Id", targetTagId);

                telemetry.update();
            }
            mediaPlayer.start();

        }
    }
}
