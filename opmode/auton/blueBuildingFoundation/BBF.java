package org.firstinspires.ftc.teamcode.opmode.auton.blueBuildingFoundation;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Hook;

public class BBF extends LinearOpMode {
    private Drivetrain drive = new Drivetrain(this);
    private ElapsedTime runtime = new ElapsedTime();
    private Hook hook = new Hook(this);
    @Override

    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        hook.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        hook.unhook();
        sleep(1000);

        drive.strafe(0.5, 0.5);
        sleep(800);
        drive.setPower(0, 0, 0, 0);
        sleep(500);

        drive.driveToPos(31.5, 0.3);
        sleep(1000);

        hook.hook();
        sleep(1000);

        drive.driveToPos(31, -0.3);
        sleep(1000);

        hook.unhook();

        drive.strafe(0.5, 2);


    }
}
