package org.firstinspires.ftc.teamcode.opmode.auton.buildingFoundationPark;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red-Building-Foundation-FrontPark")
public class BFPRedFront extends BFP {

    @Override
    public void runOpMode() throws InterruptedException {
        super.init("red", "front");
        waitForStart();
        super.runOpMode();
    }
}
