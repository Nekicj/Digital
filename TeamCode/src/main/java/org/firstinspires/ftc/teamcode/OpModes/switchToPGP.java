package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils.asmConfig;

@TeleOp(name="GPP switch",group = "Switchers")
public class switchToPGP extends LinearOpMode {

    @Override
    public void runOpMode(){
        asmConfig.pattern = 1;
    }
}
