package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;

@TeleOp(name = "DriveTuner", group = "Test")
public class DriveTuner extends LinearOpMode {

    private int driveDistance = 0;
    private int driveAngle = 0;
    private double drivePower;
    private Drivetrain drive = new Drivetrain(this);

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);

        while(!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Set distance", "bumpers");
            telemetry.addData("Set power", "triggers");
            telemetry.addData("Set angle", "dpad horizontal");

            telemetry.addData("Change controller", "y");
            telemetry.addData("Change coefficient", "x");
            telemetry.addData("Set coefficient", "dpad vertical");

            telemetry.addData("Current controller", drive.controller());
            telemetry.addData("Current coefficient", drive.coefficient());

            for(double coefficient : drive.getCoefficients())
                telemetry.addData("coefficient", coefficient);

            telemetry.addData("drive", "a");
            telemetry.addData("turn", "b");
            telemetry.addData("strafe", "back, start");
            telemetry.update();

            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;

            drive.teleDrive(r, robotAngle, rightX);

            if (gamepad1.a) drive.driveToPos(driveDistance, drivePower);

            if (gamepad1.b) drive.turn(driveAngle, drivePower);

            if (gamepad1.back) {
                drive.strafeLeft(drivePower);
                sleep(250);
                drive.setPower(0,0,0,0);
            }

            if(gamepad1.start) {
                drive.strafeRight(drivePower);
                sleep(250);
                drive.setPower(0,0,0,0);
            }

            // set power
            if(gamepad1.right_trigger > 0) drivePower += (double) gamepad1.right_trigger;

            if(gamepad1.left_trigger > 0) drivePower -= (double) gamepad1.left_trigger;

            // set distance
            if(gamepad1.right_bumper) driveDistance++;

            if(gamepad1.left_bumper) driveDistance--;

            // set angle
            if(gamepad1.dpad_right) driveAngle++;

            if(gamepad1.dpad_left) driveAngle--;

            // change PID
            if(gamepad2.x) drive.changeCoefficient();

            if(gamepad2.y) drive.changeController();

            if(gamepad2.dpad_up) drive.increaseCoefficient();

            if(gamepad2.dpad_down) drive.decreaseCoefficient();
        }
    }
}
