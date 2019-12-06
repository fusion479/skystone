package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;

public class Main extends LinearOpMode {

    private Drivetrain drivetrain = new Drivetrain(this);
    private Camera camera = new Camera(this);
    private Claw claw = new Claw(this);

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap);
        camera.init(hardwareMap);
        claw.init(hardwareMap);

        while(!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;

            drivetrain.teleDrive(r, robotAngle, rightX);

            if (gamepad1.a) claw.open();

            if (gamepad1.b) claw.close();

            if (gamepad1.left_bumper) claw.front();

            if (gamepad1.right_bumper) claw.back();

            // gamepad1.left_trigger and gamepad1.right_trigger are floating point values
            // left_trigger goes down, right_trigger goes up for lift

        }
    }
}
