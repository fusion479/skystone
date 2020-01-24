package org.firstinspires.ftc.teamcode.opmode.auton.buildingFoundation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red-Building-Foundation")
public class BFRed extends BF {

    @Override
    public void runOpMode() throws InterruptedException {
        super.init("red");
        waitForStart();
        super.runOpMode();
    }
}
