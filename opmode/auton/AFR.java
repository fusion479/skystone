package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Hook;
import org.firstinspires.ftc.teamcode.hardware.Lift;

@Autonomous(name="AFR")
public class AFR extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Drivetrain drive = new Drivetrain(this);
    private Hook hook = new Hook(this);

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

        // case 1 red loading
        // facing forward
//        drive.driveToPos(6, 0.75);
//        drive.turn(90, 0.5);
//        drive.driveToPos(10, 0.75);

        //case 2 pull platform blue building
        drive.turn(-90, 0.75);
        drive.driveToPos(5,0.4);
        drive.turn(90, 0.75);
        hook.unhook();
        sleep(500);
        drive.driveToPos(14, 0.4);
        sleep(500);
        hook.hook();
        sleep(500);
        drive.driveToPos(-17, 0.4);

//        drive.turn(90, 1);
//        drive.driveToPos(36, 1);
    }
}
