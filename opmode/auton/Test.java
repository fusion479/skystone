package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.TensorFlowCamera;

@Autonomous(name="test")
public class Test extends LinearOpMode {
    private Drivetrain drive = new Drivetrain(this);
    private TensorFlowCamera camera = new TensorFlowCamera(this);
    private Acquirer acquirer = new Acquirer(this);
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
//        drive.init(hardwareMap);
//        waitForStart();
//
//        drive.strafe(0.5,1);
//        sleep(1000);
//        drive.strafe(-0.5, 1);
//        sleep(1000);
//        drive.turn(40, 0.5);
//        sleep(1000);
//        drive.driveToPos(10, 0.5);
//        drive.strafe(0.5, 1);

        drive.init(hardwareMap);
        acquirer.init(hardwareMap);
        camera.init(hardwareMap);
        TFObjectDetector tfod = camera.getTFod();
        if (tfod != null) {
            tfod.activate();
        }
        drive.getCamera(camera);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        drive.driveToPos(20, 0.3);
        int pattern = drive.find_stone();
        if(pattern == 1) {
            drive.driveToPos(5,0.5);
            drive.strafe(0.5, 0.8);
            drive.turn(-45, 0.5);
            drive.acquire(acquirer, 0.7, 25);
            acquirer.teleIntake(1);
            sleep(800);
            acquirer.stop();
        }
        else if (pattern == 3) {
            drive.strafe(-0.5, 0.7);
            drive.driveToPos(30,0.5);
            sleep(1000);
            drive.driveToPos(10, -0.5);
            sleep(1000);
            drive.turn(85, 0.6);
            drive.acquire(acquirer,0.7, 12);
        }
        sleep(1000);
        acquirer.stop();
    }
}
