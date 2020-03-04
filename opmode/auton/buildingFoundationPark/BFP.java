package org.firstinspires.ftc.teamcode.opmode.auton.buildingFoundationPark;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Hook;

public class BFP extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Drivetrain drive = new Drivetrain(this);
    private Hook hook = new Hook(this);
    private Claw claw = new Claw(this);
    private int alliance;
    private String parkPosition;

    protected void init(String alliance, String parkPosition) {
        drive.init(hardwareMap);
        hook.init(hardwareMap);
        claw.init(hardwareMap);
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
        sleep(400);

        drive.strafe( alliance * 0.4, 0.9);
        sleep(250);

        drive.driveToPos(30, 0.3);
        sleep(250);

        hook.hook();
        sleep(400);

        drive.driveToPos(26, -0.4);
        sleep(250);

        hook.unhook();
        sleep(400);

        drive.strafe(alliance * -0.4, 0.65);
        sleep(250);

        drive.driveToPos(3, 0.4);

        hook.hook();
        sleep(1000);

        drive.turn(alliance * 90, 1);
        sleep(250);

        drive.driveToPos(25, 0.7);
        sleep(500);

        if(parkPosition.compareTo("back") == 0) {
            drive.strafe(alliance * 0.4, 1.2);
            hook.unhook();
            sleep(400);
            drive.driveToPos(40, -0.4);
        }
        else {
            hook.unhook();
            sleep(400);
            drive.strafe(-1 * alliance * 0.5, 0.5);
            drive.driveToPos(40, -0.5);
        }
    }
}
