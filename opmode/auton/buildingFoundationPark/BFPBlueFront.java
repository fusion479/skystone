package org.firstinspires.ftc.teamcode.opmode.auton.buildingFoundationPark;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue-Building-Foundation-FrontPark")
public class BFPBlueFront extends BFP {

    @Override
    public void runOpMode() throws InterruptedException {
        super.init("blue", "front");
        waitForStart();
        super.runOpMode();
    }
}
