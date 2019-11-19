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

        while (opModeIsActive()){

            drivetrain.driveToPos(12, 0.7);
            drivetrain.turn(90, 0.7);
            drivetrain.driveToPos(24, 0.7);
            drivetrain.turn(-90, 0.7  );
            drivetrain.driveToPos(12 , 0.7);

            break;

        }

    }
}
