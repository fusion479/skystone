package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Claw;


@TeleOp(name="Drive", group="Teleop")
public class DriveTest extends LinearOpMode {

    double servoPosition;




    @Override
    public void runOpMode() throws InterruptedException {


        while(!opModeIsActive() && !isStopRequested()) {
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            telemetry.addData("Status", "Waiting in Init");
            telemetry.addData("r", r);
            telemetry.addData("robotAngle", robotAngle);
            telemetry.addData("rightX", rightX);
            telemetry.addData("right-stickx", gamepad1.right_stick_x);
            telemetry.addData("right-sticky", gamepad1.right_stick_y);
            telemetry.addData("left-stickx", gamepad1.left_stick_x);
            telemetry.addData("left-sticky", gamepad1.left_stick_y);
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()) {
//            servoPosition = 0;
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;

//            if(gamepad1.b) {
//                servoPosition = 0.5;
//            }
//            if (gamepad1.a) {
//                servoPosition = 1.0;
//            }

//            drive.servo.setPosition(servoPosition);

            telemetry.addData("r", r);
            telemetry.addData("robotAngle", robotAngle);
            telemetry.addData("rightX", rightX);
            telemetry.addData("right-stickx", gamepad1.right_stick_x);
            telemetry.addData("right-sticky", gamepad1.right_stick_y);
            telemetry.addData("left-stickx", gamepad1.left_stick_x);
            telemetry.addData("left-sticky", gamepad1.left_stick_y);
            telemetry.addData("bbutton", gamepad1.b);
//            telemetry.addData("servoPosition", servoPosition);
            telemetry.addData("abutton", gamepad1.a);
            telemetry.update();
        }
    }
}
