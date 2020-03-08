package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;

@TeleOp(name="CV Test", group="Teleop")
public class TeleCVTest extends LinearOpMode {
    private Drivetrain drive = new Drivetrain(this);
    private Camera camera = new Camera(this);
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        camera.init(hardwareMap);

        TFObjectDetector tfod = camera.getTFod();
        if (tfod != null) {tfod.activate();}
        drive.setCamera(camera);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        runtime.reset();

        camera.findSkystone();
        sleep(4000);
        if (tfod != null) {tfod.deactivate();}

    }
}
