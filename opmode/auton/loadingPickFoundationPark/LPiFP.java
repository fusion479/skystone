package org.firstinspires.ftc.teamcode.opmode.auton.loadingPickFoundationPark;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Hook;
import org.firstinspires.ftc.teamcode.hardware.Lift;

public class LPiFP extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Drivetrain drive = new Drivetrain(this);
    private Claw claw = new Claw(this);
    private Hook hook = new Hook(this);
    private Lift lift = new Lift(this);
    private Camera camera = new Camera(this);
    private int alliance;

    protected  void init(String alliance) {
        drive.init(hardwareMap);
        hook.init(hardwareMap);
        claw.init(hardwareMap);
//        camera.init(hardwareMap);
//        camera.activateTrackables();
        lift.init(hardwareMap);

//        drive.getCamera(camera);

        this.alliance = (alliance.compareTo("red") == 0)
                ? -1
                : 1;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    @Override
    public void runOpMode() throws InterruptedException {

        runtime.reset();

        // bring claw to front
        claw.front();
        sleep(1100);

        // open the claw
        claw.open();
        sleep(300);

        // drive forward to block
        drive.driveToPos(25.5, 0.3);

        //slightly strafe for lineup
        drive.strafe(-0.1, 0.5);

        //acquire block
        claw.close();
        sleep(300);

        //bring the lift up
        lift.liftUp(0.4);
        sleep(300);
        lift.liftOff();

        //drive back
        drive.driveToPos(10, -1);

        //get to building site
        drive.strafe(alliance * 0.7, 2);

        //lift up more
        lift.liftUp(0.5);
        sleep(500);
        lift.liftOff();

        // drive forward to foundation
        drive.driveToPos(10,0.8);

        //drop the block
        claw.open();
        sleep(300);

        // center with foundation
        drive.strafe(alliance * 0.5, 0.5);

        // lower life
        lift.liftDown(0.4);
        sleep(400);
        lift.liftOff();

        // close claw
        claw.close();
        sleep(300);

        // retract claw
        claw.back();
        sleep(500);

        // get foundation
        hook.unhook();
        sleep(400);

        drive.driveToPos(4,0.2);

        hook.hook();
        sleep(400);

        // drive backwards
        drive.driveToPos(31, -0.5);

        hook.unhook();
        sleep(400);

        //park
        drive.strafe(alliance * -0.4,2.5);
    }
}
