package org.firstinspires.ftc.teamcode.opmode.auton.redBuildingFoundationPark;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Hook;

@Autonomous(name="Red-Building-Foundation-FrontPark")
public class RBFfP extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Drivetrain drive = new Drivetrain(this);
    private Hook hook = new Hook(this);

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        hook.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // facing parallel to bridge
        hook.unhook();
        sleep(1000);

        //get completely in front of foundation
        drive.strafeRight(1);
        sleep(500);
        drive.setPower(0,0,0,0);
        sleep(500);

        //drive towards foundation
        drive.driveToPos(30, 0.4);
        sleep(1000);

        // clamp down on foundation
        hook.hook();
        sleep(1000);

        // drive back
        drive.driveToPos(-30, 0.4);
        sleep(1000);

        //unhook
        hook.unhook();
        sleep(1000);

        // strafe out of foundation lock
        drive.strafeLeft(1);
        sleep(750);
        drive.setPower(0,0,0,0);

        //drive forward
        drive.driveToPos(20, 0.75);
        sleep(500);

        // strafe under bridge
        drive.strafeLeft(1);
        sleep(250);
        drive.setPower(0,0,0,0);
    }
}
