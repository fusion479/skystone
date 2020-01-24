package org.firstinspires.ftc.teamcode.opmode.auton.buildingFoundationPark;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue-Building-Foundation-BackPark")
public class BFPBlueBack extends BFP {
    @Override
    public void runOpMode() throws InterruptedException {
        super.init("blue", "back");
        waitForStart();
        super.runOpMode();
    }
}

