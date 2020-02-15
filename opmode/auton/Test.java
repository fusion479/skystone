package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Lift;

@Autonomous(name="test")
public class Test extends LinearOpMode {
    private Drivetrain drive = new Drivetrain(this);
//    private Claw claw = new Claw(this);
//    private Lift lift = new Lift(this);
//    private Camera camera = new Camera(this);
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
//        claw.init(hardwareMap);
//        camera.init(hardwareMap);
//        lift.init(hardwareMap);
//        camera.activateTrackables();
//
//        drive.getCamera(camera);
//
        telemetry.addData("Status", "Initialized");
        telemetry.update();
//
        waitForStart();
        runtime.reset();

        drive.driveToPos(10, 1);
        sleep(1000);
        drive.driveToPos(10, -1);
        sleep(1000);
        drive.strafe(0.5, 0.5); //left
        sleep(1000);
        drive.strafe(-0.5, 0.5); // right
        sleep(1000);
        drive.turn(90, 0.8); //left
        sleep(1000);
        drive.turn(-90, 0.8); // right

//        claw.front();
//        sleep(1400);
//
//        claw.open();
//        sleep(1000);
//
//        drive.driveToPos(23.5, 0.3);
//
//        drive.find_stone(0.05);
//
//        float[] positions = camera.getLocation();
//        telemetry.addData("Pos (in)", "{X, Y, Z} = %.2f, %.2f, %.2f", positions[0], positions[1], positions[2]);
//
//        // express the rotation of the robot in degrees.
//        float[] degrees = camera.getRotation();
//        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.2f, %.2f, %.2f", degrees[0], degrees[1], degrees[2]);
//
//        telemetry.update();
//        sleep(1000);
//
//        if(Math.abs(degrees[0]) - 90 > 1) {
//            drive.strafe(-0.1, 0.6);
//        }
////        else if(Math.abs(degrees[1]) - 10 > 0) {
////            drive.strafe(0.1, 0.6);
////        }
//
//        claw.close();
//        sleep(8000);
    }
}
