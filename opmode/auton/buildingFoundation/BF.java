package org.firstinspires.ftc.teamcode.opmode.auton.buildingFoundation;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Hook;

public class BF extends LinearOpMode {
    private Drivetrain drive = new Drivetrain(this);
    private ElapsedTime runtime = new ElapsedTime();
    private Hook hook  = new Hook(this);
    private Claw claw = new Claw(this);
    private int alliance;

    protected void init(String alliance) {
        drive.init(hardwareMap);
        hook.init(hardwareMap);
        claw.init(hardwareMap);
        this.alliance = (alliance.compareTo("red") == 0 )
                ? -1
                : 1;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        runtime.reset();

        hook.unhook();

        drive.strafe( alliance * 0.4, 0.9);
        sleep(250);

        drive.driveToPos(30, 0.3);
        sleep(250);

        hook.hook();

        drive.driveToPos(26, -0.4);
        sleep(250);

        hook.unhook();

        drive.strafe(alliance * -0.4, 0.65);
        sleep(250);

        drive.driveToPos(3, 0.4);

        hook.hook();
        sleep(600);

        drive.turn(alliance * 90, 1);
        sleep(250);

        drive.driveToPos(25, 0.7);
        sleep(500);

        hook.unhook();
    }
}
