package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Picker;

@Autonomous(name="auton mechanism")
public class AutonClawTest extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        Picker picker = new Picker(this);
        Claw claw = new Claw(this);
        claw.init(hardwareMap);
        picker.init(hardwareMap);
        waitForStart();
//        picker.retract();
//        sleep(2000);

    }
}