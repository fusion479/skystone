package org.firstinspires.ftc.teamcode.opmode.auton.visionFoundationPark;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Hook;
import org.firstinspires.ftc.teamcode.hardware.Picker;

@Autonomous(name="Red-Vision-Foundation-Park")
public class CVFPRed extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Drivetrain drive = new Drivetrain(this);
    private Claw claw = new Claw(this);
    private Camera camera = new Camera(this);
    private Picker picker = new Picker(this);
    private Hook hook = new Hook(this);

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        claw.init(hardwareMap);
        camera.init(hardwareMap);
        hook.init(hardwareMap);
        picker.init(hardwareMap);

        TFObjectDetector tfod = camera.getTFod();
        if (tfod != null) { tfod.activate(); }
        drive.setCamera(camera);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        runtime.reset();

        drive.strafe(-0.6, 1);
        sleep(500);

        int pattern = drive.findStone(0.15);
        if(pattern == 0) {
            drive.driveToPos(4.55, 0.5);
        }
        if(pattern == 2) {
            drive.driveToPos(4.6, 0.5);
        }
//        if(pattern == 0 || pattern == 2) { drive.driveToPos(4.55,0.5); }
        telemetry.addData("Pattern", pattern);
        telemetry.update();
        if (tfod != null) { tfod.deactivate(); }
        picker.delatch();
        sleep(400);
        picker.extend();
        sleep(800);
        drive.strafe(-0.4, 0.7);
        picker.latch();
        sleep(800);
        picker.stoneRetract();
        sleep(800);

        drive.strafe(0.5, 0.5);

        if(pattern == 0) {
            drive.driveToPos(67, -0.5);
        } else if (pattern == 1) {
            drive.driveToPos(85, -0.5);
        } else if (pattern == 2) {
            drive.driveToPos(95, -0.5);
        }

        drive.strafe(-0.6, 0.35);

        picker.delatch();
        sleep(1000);
        picker.stoneRetract();
        sleep(1000);

        drive.strafe(0.6, 0.25);
        sleep(500);
        drive.turn(90, 1);
        hook.unhook();
        sleep(400);
        drive.driveToPos(3, 0.5);
        hook.hook();
        sleep(500);
        drive.driveToPos(20, -0.7);
        drive.turn(90, 1);
        sleep(500);
        drive.driveToPos(10, 0.7);
        hook.unhook();
        sleep(400);
    }
}