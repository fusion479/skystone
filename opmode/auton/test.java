package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Hook;

@Autonomous(name="test")

public class test extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Drivetrain drive = new Drivetrain(this);
//    private Hook hook = new Hook(this);
    private Claw claw = new Claw(this);
//    private Camera camera = new Camera(this);

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        claw.init(hardwareMap);

        drive.camera.activateTrackables();
//        hook.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        drive.driveToPos(26, 0.3);
        drive.setPower(0,0,0,0);
        drive.sleep(500);
//        sleep(500);
//        claw.front();
//        sleep(1500);
//        claw.close();
//        sleep(1000);
        drive.auton_strafe(0.1);
//        while(camera.isTargetVisible().equals("none")) {
//            drive.strafe(0.3, 0.1);
//        }
//        drive.setPower(0,0,0,0);
//        drive.strafe(0.3,7 );
//        sleep(2000);
//        drive.setPower(0,0,0,0);

    }
}
