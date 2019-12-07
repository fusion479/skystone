package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Claw;

@Autonomous(name = "AutonDriveTest")
public class AutonDriveTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Drivetrain drivetrain = new Drivetrain(this);
    private Claw claw = new Claw(this);
    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        claw.open();

        waitForStart();

        runtime.reset();
        drivetrain.resetAngle();
        drivetrain.driveToPos(24, 0.75);
        drivetrain.strafeLeft();
        claw.close();
        sleep(100);
        drivetrain.strafeRight();
        sleep(2000);
        drivetrain.strafeLeft();

    }
}
