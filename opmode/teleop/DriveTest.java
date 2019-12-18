package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Claw;

@TeleOp(name="Drive", group="Teleop")
public class DriveTest extends LinearOpMode {

    double servoPosition;

    private Drivetrain drive = new Drivetrain(this);
    @Override
    public void runOpMode() throws InterruptedException {

        drive.init(hardwareMap);

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

            drive.teleDrive(r, robotAngle, rightX);

            if(gamepad1.a) {
                drive.resetAngle();
                drive.driveToPos(5, 0.75);
            }

            if(gamepad1.b) {
                drive.setPower(0,0,0,0);
            }

            if(gamepad1.dpad_up) {
                drive.setP(0.01);
            }

            if(gamepad1.dpad_down) {
                drive.setP(-0.01);
            }

            telemetry.addData("p", drive.getP());
//            telemetry.addData("r", r);
//            telemetry.addData("robotAngle", robotAngle);
//            telemetry.addData("rightX", rightX);
//            telemetry.addData("right-stickx", gamepad1.right_stick_x);
//            telemetry.addData("right-sticky", gamepad1.right_stick_y);
//            telemetry.addData("left-stickx", gamepad1.left_stick_x);
//            telemetry.addData("left-sticky", gamepad1.left_stick_y);
//            telemetry.addData("bbutton", gamepad1.b);
////            telemetry.addData("servoPosition", servoPosition);
//            telemetry.addData("abutton", gamepad1.a);
            telemetry.update();
        }
    }
}
