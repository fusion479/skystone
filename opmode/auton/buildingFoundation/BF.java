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
        sleep(1000);

        drive.strafe( alliance * 0.4, 0.9);
        sleep(400);

        drive.driveToPos(30, 0.5);
        sleep(500);

        hook.hook();
        sleep(1000);

        drive.driveToPos(32, -0.5);
        sleep(500);

        hook.unhook();
        sleep(1000);

    }
}
