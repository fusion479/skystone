package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Camera;

@Autonomous(name = "Basic")
public class Basic extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    //    Drivetrain drivetrain = new Drivetrain(this);
    Claw claw = new Claw(this);
    Camera camera = new Camera(this);

    public void runOpMode() {
//        drivetrain.init(hardwareMap);
        claw.init(hardwareMap);
        camera.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
//        telemetry.addData("imu status", drivetrain.imu.getCalibrationStatus().toString());
//        drivetrain.getAngle();
        telemetry.update();
        waitForStart();
        runtime.reset();  // Start counting run time from now.
//        drivetrain.resetAngle();
//        drivetrain.driveToPos(5, 0.5);
//        drivetrain.setPower(0.5, -0.5, 0.5, -0.5);
//        sleep(10000);
//        drivetrain.setPower(0, 0, 0, 0);
        camera.activateTrackables();

        while (!isStopRequested()) {
            String targetVisible = camera.isTargetVisible();

            if (!targetVisible.equals("none")) {
                float[] positions = camera.getLocation();

                if (targetVisible.equals("Stone Target") && positions[0] > -6.0) {
                    claw.close();
                    sleep(2000);
                    claw.back();
                    sleep(2000);
                    claw.open();
                    sleep(2000);
                }
            }
//            telemetry.addData("1 imu heading", drivetrain.getHeading());
//        drivetrain.turn(90, 0.5);
            camera.deactivateTrackables();
        }
    }
}
