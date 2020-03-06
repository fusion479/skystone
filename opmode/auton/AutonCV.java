package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Picker;

@Autonomous(name="AutonCV")
public class AutonCV extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Drivetrain drive = new Drivetrain(this);
    private Claw claw = new Claw(this);
    private Camera camera = new Camera(this);
    private Picker picker = new Picker(this);

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        claw.init(hardwareMap);
        camera.init(hardwareMap);
        picker.init(hardwareMap);

        TFObjectDetector tfod = camera.getTFod();
        if (tfod != null) { tfod.activate(); }
        drive.setCamera(camera);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        runtime.reset();

        drive.strafe(-0.4, 1.3);
        sleep(500);

        int pattern = drive.findStone(0.2);
        if (tfod != null) { tfod.deactivate(); }
        if (pattern == 0) {
            drive.driveToPos(5, 0.5);
        } if (pattern == 1 || pattern == 2) {
            drive.driveToPos(7, 0.5);
        }
        picker.delatch();
        sleep(400);
        picker.extend();
        sleep(400);
        drive.strafe(-0.4, 0.65);
        picker.latch();
        sleep(1500);
        picker.stoneRetract();
        sleep(400);

        drive.strafe(0.4, 0.7);

        if(pattern == 0) {
            drive.driveToPos(70, -0.5);
        } else if (pattern == 1) {
            drive.driveToPos(80, -0.5);
        } else if (pattern == 2) {
            drive.driveToPos(90, -0.5);
        }

        drive.strafe(-0.4, 0.5);

    }
}
