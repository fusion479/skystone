package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Hook;
import org.firstinspires.ftc.teamcode.hardware.Lift;

import java.util.concurrent.TimeUnit;

@Autonomous(name="Red Loading Pick Drop and Back Park")

public class Pick extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Drivetrain drive = new Drivetrain(this);
    private Hook hook = new Hook(this);
    private Lift lift = new Lift(this);
    private Claw claw = new Claw(this);
    private Camera camera = new Camera(this);

    @Override
    public void runOpMode() throws InterruptedException {
        claw.init(hardwareMap);
        camera.init(hardwareMap);
        hook.init(hardwareMap);
        drive.init(hardwareMap);
        lift.init(hardwareMap);

        camera.activateTrackables();
        drive.getCamera(camera);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // bring claw to front
        claw.front();
        sleep(1400);

        // open the claw
        claw.open();
        sleep(1000);

        // drive forward to block
        drive.driveToPos(25.75,0.2);

        //slightly strafe for lineup
        drive.strafe(-0.1,0.5);

        //acquire block
        claw.close();
        sleep(1000);

        //bring the lift up
        lift.liftUp(0.4);
        sleep(400);
        lift.liftOff();
        sleep(500);

        //drive back
        drive.driveToPos(10, -0.6);

        //get to building site
        drive.strafe(-0.5, 2.6);

        //lift up more
        lift.liftUp(0.5);
        sleep(400);
        lift.liftOff();

        // drive forward to foundation
        drive.driveToPos(10,0.6);

        //drop the block
        claw.open();
        sleep(1000);

        drive.strafe(-0.5, 0.7);

        lift.liftDown(0.4);
        sleep(400);
        lift.liftOff();

        claw.close();
        sleep(1000);

        claw.back();
        sleep(1400);

        hook.unhook();
        sleep(1000);

        drive.driveToPos(3,0.2);

        hook.hook();
        sleep(1200);

        drive.driveToPos(30.5, -0.5);

        hook.unhook();
        sleep(1000);

        drive.strafe(0.4,2.5);
    }
}
