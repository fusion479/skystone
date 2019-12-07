package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;

@Autonomous(name = "AutonDriveTest")
public class AutonDriveTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Drivetrain drivetrain = new Drivetrain(this);

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        runtime.reset();

        while(!isStopRequested()) {
            drivetrain.resetAngle();
            //drivetrain.autonDrive(5,0.75,60,-1);
            break;
        }
    }
}
