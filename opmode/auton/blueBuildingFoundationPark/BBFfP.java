package org.firstinspires.ftc.teamcode.opmode.auton.blueBuildingFoundationPark;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Hook;


@Autonomous(name="Blue-Building-Foundation-FrontPark")
public class BBFfP extends LinearOpMode {
     private ElapsedTime runtime = new ElapsedTime();
     private Drivetrain drive = new Drivetrain(this);
     private Hook hook = new Hook(this);

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        hook.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        hook.unhook();
        sleep(1000);

        drive.strafe(0.4, 0.6);
        sleep(800);
        drive.setPower(0,0,0,0);
        sleep(500);

        drive.driveToPos(30.5,0.3);
        sleep(1000);

        hook.hook();
        sleep(1000);

        drive.driveToPos(32.5, -0.3);
        sleep(1000);

        hook.unhook();
        sleep(1000);

        drive.strafe(-0.5,1.2);
        sleep(750);
        drive.setPower(0, 0, 0, 0);

        drive.driveToPos(18, 0.5);
        sleep(500);

        drive.strafe(-0.5,0.8);
        sleep(500);
        drive.setPower(0, 0, 0, 0);
    }
}
