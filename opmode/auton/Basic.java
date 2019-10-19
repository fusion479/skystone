package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;

@Autonomous(name = "Basic")
public class Basic extends LinearOpMode {
    Drivetrain drivetrain = new Drivetrain(this);

    public void runOpMode() {
        drivetrain.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
//        drivetrain.driveToPos(5, 0.5);
        drivetrain.setPower(0.5,-0.5,0.5,-0.5);
        sleep(10000);
        drivetrain.setPower(0,0,0,0);
//        drivetrain.turn(90,1);

        while(true) {
            telemetry.addData("imu status", drivetrain.imu.getCalibrationStatus().toString());
            telemetry.update();
            sleep(5000);
        }
    }
}
