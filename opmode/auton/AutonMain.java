package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Hook;
import org.firstinspires.ftc.teamcode.hardware.Lift;

@Autonomous(name="AutonMain")
public class AutonMain extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Drivetrain drive = new Drivetrain(this);
//    private Camera camera = new Camera(this);
//    private Claw claw = new Claw(this);
    private Hook hook = new Hook(this);
//    private Lift lift = new Lift(this);

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
//        camera.init(hardwareMap);
//        claw.init(hardwareMap);
        hook.init(hardwareMap);
//        lift.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        runtime.reset();
        hook.unhook();
        sleep(1000);

        drive.strafeRight(1);

        //drive strafeLeft remains untested

        sleep(400);
        drive.setPower(0,0,0,0);
        drive.driveToPos(31,0.45);
        hook.hook();
        sleep(1000);
        drive.driveToPos(-31, 0.45);
    }
}
