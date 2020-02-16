package org.firstinspires.ftc.teamcode.opmode.auton.buildingFoundationPark;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Hook;

public class BFP extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Drivetrain drive = new Drivetrain(this);
    private Hook hook = new Hook(this);
    private int alliance;
    private String parkPosition;

    protected void init(String alliance, String parkPosition) {
        drive.init(hardwareMap);
        hook.init(hardwareMap);
        this.alliance = (alliance.compareTo("red") == 0)
                ? -1
                : 1;
        this.parkPosition = parkPosition;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        runtime.reset();

        hook.unhook();
        sleep(1000);

        drive.strafe(alliance * 0.4, 0.8);
        sleep(400);

        drive.driveToPos(2200, 0.3);
        sleep(500);

        hook.hook();
        sleep(1000);

        drive.driveToPos(2800, -0.3);
        sleep(500);

        hook.unhook();
        sleep(1000);

        if(parkPosition.compareTo("back") == 0) {
            drive.strafe(1 * alliance * 0.4, 2.5);
        }
        else {
            drive.strafe(1 * alliance * 0.5, 1.5);
            drive.driveToPos(500, 0.5);
            sleep(250);
            drive.strafe(1 * alliance * 0.5, 1);
        }
    }
}
