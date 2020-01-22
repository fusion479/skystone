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

@Autonomous(name="test")

public class test extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Drivetrain drive = new Drivetrain(this);
//    private Hook hook = new Hook(this);
    private Lift lift = new Lift(this);
    private Claw claw = new Claw(this);
    private Camera camera = new Camera(this);

    @Override
    public void runOpMode() throws InterruptedException {
        claw.init(hardwareMap);
        camera.init(hardwareMap);
        drive.init(hardwareMap);
        lift.init(hardwareMap);

        camera.activateTrackables();
//        hook.init(hardwareMap);
        drive.getCamera(camera);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

//        drive.find_stone(0.5);
//        drive.strafe(-0.2, 4);
//        sleep(1000);
//        drive.resetAngle();
        drive.driveToPos(25, 0.2);
        sleep(1000);
        drive.find_stone(-0.06);
        sleep(1000);
//        claw.close();
//        sleep(2000);
//        lift.liftUp(0.5);
//        sleep(300);
//        lift.liftOff();
//        drive.driveToPos(25, -0.2);
//        sleep(300);
//        drive.strafe(-0.2,3);
//        sleep(500);
//        claw.open();
//        sleep(500);
//        drive.strafe(0.2, 4);
//        while(true) {
//            telemetry.addData("status", camera.isTargetVisible());
//            telemetry.update();
//        }
//        long now = runtime.now(TimeUnit.SECONDS);
//        while(runtime.now(TimeUnit.SECONDS) - now < 2) {
//            telemetry.addData("track", camera.isTargetVisible());
//            telemetry.update();
//        }
//        drive.strafe(0.5,0.54);
//        now = runtime.now(TimeUnit.SECONDS);
//        while(runtime.now(TimeUnit.SECONDS) - now < 1) {
//            telemetry.addData("track", camera.isTargetVisible());
//            telemetry.update();
//        }
//        drive.strafe(0.5,0.54);
//        now = runtime.now(TimeUnit.SECONDS);
//        while(runtime.now(TimeUnit.SECONDS) - now < 1) {
//            telemetry.addData("track", camera.isTargetVisible());
//            telemetry.update();
//        }
//
//        if (!camera.isTargetVisible().equals("none")) {
//            claw.close();
//            sleep(500);
//        }
    }
}
