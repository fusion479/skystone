package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.TensorFlowCamera;

@Autonomous(name="test")
public class Test extends LinearOpMode {
    private Drivetrain drive = new Drivetrain(this);
    private TensorFlowCamera camera = new TensorFlowCamera(this);
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
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
        telemetry.addData("pattern", drive.find_stone());
        telemetry.update();
        sleep(10000);
//        drive.strafe(-0.5, 2);
//        sleep(1000);
//        drive.strafe(0.5, 2);
    }
}
