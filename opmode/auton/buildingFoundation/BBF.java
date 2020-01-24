package org.firstinspires.ftc.teamcode.opmode.auton.buildingFoundation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BlueBuildingFoundation")
public class BBF extends BuildingFoundation {

    @Override
    public void runOpMode() throws InterruptedException {
        super.init("blue");
        waitForStart();
        super.runOpMode();
    }
}
