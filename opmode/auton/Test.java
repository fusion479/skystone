package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Lift;

@Autonomous(name="test")
public class Test extends LinearOpMode {
    private Drivetrain drive = new Drivetrain(this);
    private Claw claw = new Claw(this);
    private Lift lift = new Lift(this);
    private Camera camera = new Camera(this);
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        claw.init(hardwareMap);
        camera.init(hardwareMap);
        lift.init(hardwareMap);
        camera.activateTrackables();

        drive.getCamera(camera);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        claw.front();
        sleep(1400);

        claw.open();
        sleep(1000);

        drive.driveToPos(24.3, 0.4);

        drive.find_stone(0.08);

        sleep(1000);

        drive.strafe(-0.1, 0.6);

        claw.close();
        sleep(2000);
    }
}
