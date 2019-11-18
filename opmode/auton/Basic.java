package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;

@Autonomous(name = "Basic")
public class Basic extends LinearOpMode {
    Drivetrain drivetrain = new Drivetrain(this);

    public void runOpMode() {
        drivetrain.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        drivetrain.driveToPos(12, 5);
        drivetrain.turn(90, 5);
        drivetrain.driveToPos(24, 5);
        drivetrain.turn(-90, 5  );
        drivetrain.driveToPos(12 , 5);
    }
}
