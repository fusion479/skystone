package org.firstinspires.ftc.teamcode.opmode.auton.frontPark;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;

@Autonomous(name="Red-Building-Blue-Loading-FrontPark")
public class RBBLfP extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Drivetrain drive = new Drivetrain(this);

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        runtime.reset();

        //facing parallel to bridge using strafe
        drive.driveToPos(23,0.75);
        drive.strafeLeft(1);
        sleep(500);
        drive.setPower(0,0,0,0);

        //facing towards bridge using strafe
        drive.strafeRight(1);
        sleep(500);
        drive.setPower(0,0,0,0);
        drive.driveToPos(25, 0.75);

        //facing parallel to bridge using turn
        drive.driveToPos(23, 0.75);
        drive.turn(90, 1);
        sleep(500);
        drive.driveToPos(25, 0.75);
    }
}
