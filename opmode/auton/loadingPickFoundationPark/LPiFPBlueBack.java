package org.firstinspires.ftc.teamcode.opmode.auton.loadingPickFoundationPark;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue-Loading-Pick-Drop-Foundation-Back-Park")

public class LPiFPBlueBack extends LPiFP {

    @Override
    public void runOpMode() throws InterruptedException {
        super.init("blue");
        waitForStart();
        super.runOpMode();
    }
}
