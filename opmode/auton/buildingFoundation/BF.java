package org.firstinspires.ftc.teamcode.opmode.auton.buildingFoundation;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Hook;

public class BF extends LinearOpMode {
    private Drivetrain drive = new Drivetrain(this);
    private ElapsedTime runtime = new ElapsedTime();
    private Hook hook  = new Hook(this);
    private int alliance;

    protected void init(String alliance) {
        drive.init(hardwareMap);
        hook.init(hardwareMap);
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

        drive.strafe( alliance * 0.4, 0.8);
        sleep(400);

        drive.driveToPos(2200, 0.3);
        sleep(500);

        hook.hook();
        sleep(1000);

        drive.driveToPos(2800, -0.3);
        sleep(500);

        hook.unhook();
        sleep(1000);

    }
}
