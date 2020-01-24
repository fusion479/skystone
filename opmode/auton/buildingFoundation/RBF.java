package org.firstinspires.ftc.teamcode.opmode.auton.buildingFoundation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="RedBuildingFoundation")
public class RBF extends BuildingFoundation {

    @Override
    public void runOpMode() throws InterruptedException {
        super.init("red");
        waitForStart();
        super.runOpMode();
    }
}
