package org.firstinspires.ftc.teamcode.opmode.auton.backPark;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;

@Autonomous(name="BackPark")
public class AAbP extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Drivetrain drive = new Drivetrain(this);

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        //facing towards bridge
        drive.driveToPos(25, 0.75);
    }
}
