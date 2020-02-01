package org.firstinspires.ftc.teamcode.opmode.auton.loadingPickFoundationPark;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red-Loading-Pick-Drop-Foundation-Back-Park")

public class LPiFPRedBack extends LPiFP {
    @Override
    public void runOpMode() throws InterruptedException {
        super.init("red");
        waitForStart();
        super.runOpMode();
    }
}
