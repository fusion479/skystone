package org.firstinspires.ftc.teamcode.opmode.auton.buildingFoundationPark;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red-Building-Foundation-BackPark")
public class BFPRedBack extends BFP {

    @Override
    public void runOpMode() throws InterruptedException {
        super.init("red", "back");
        waitForStart();
        super.runOpMode();
    }
}
