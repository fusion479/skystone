package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;

@Autonomous(name = "Basic")
public class Basic extends LinearOpMode {
    private ElapsedTime     runtime = new ElapsedTime();
    Drivetrain drivetrain = new Drivetrain(this);

    public void runOpMode() {

//        drivetrain.init(hardwareMap);
//        telemetry.addData("Status", "Initialized");
//        telemetry.addData("imu status", drivetrain.imu.getCalibrationStatus().toString());
//        drivetrain.getAngle();
//        telemetry.update();
//        waitForStart();
//        runtime.reset();  // Start counting run time from now.
//        drivetrain.resetAngle();
//        drivetrain.driveToPos(5, 0.5);
//        drivetrain.setPower(0.5,-0.5,0.5,-0.5);
//        sleep(10000);
//        drivetrain.setPower(0,0,0,0);
//
//        while(opModeIsActive()) {
//            drivetrain.turn(-120,0.7);
//            sleep(1000);
//            drivetrain.turn(120,0.7);
//            sleep(1000);
//            drivetrain.driveToPos(5,0.75);
//            telemetry.addData("1 imu heading", drivetrain.getHeading());
//            telemetry.addData("2 global heading", drivetrain.getHeading());
//            telemetry.update();
//            drivetrain.getAngle();
//            break;
//        }

//        drivetrain.turn(90,0.5);

//    }
    }
}
