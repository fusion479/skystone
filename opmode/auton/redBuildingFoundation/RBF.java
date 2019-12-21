package org.firstinspires.ftc.teamcode.opmode.auton.redBuildingFoundation;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Hook;

public class RBF extends LinearOpMode {

    private Drivetrain drive = new Drivetrain(this);
    private ElapsedTime runtime = new ElapsedTime();
    private Hook hook = new Hook(this);

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        hook.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // facing parallel to bridge
        hook.unhook();
        sleep(1000);

        //get completely in front of foundation
        drive.strafeRight(1);
        sleep(500);
        drive.setPower(0,0,0,0);
        sleep(500);

        //drive towards foundation
        drive.driveToPos(30, 0.4);
        sleep(1000);

        // clamp down on foundation
        hook.hook();
        sleep(1000);

        // drive back
        drive.driveToPos(-30, 0.4);
        sleep(1000);

        //unhook
        hook.unhook();
    }
}
