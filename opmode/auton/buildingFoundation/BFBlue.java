package org.firstinspires.ftc.teamcode.opmode.auton.buildingFoundation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue-Building-Foundation")
public class BFBlue extends BF {

    @Override
    public void runOpMode() throws InterruptedException {
        super.init("blue");
        waitForStart();
        super.runOpMode();
    }
}
