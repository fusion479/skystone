package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;

@TeleOp(name="Drive", group="Teleop")
public class DriveTest extends LinearOpMode {

    double servo0Position = 0.2;
    double servo1Position;
    double servo2Position;
    double servo3Position;
    double servo4Position;


    private Drivetrain drive = new Drivetrain();

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);

        while(!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
            telemetry.update();
        }

        waitForStart();


        while(opModeIsActive()) {
            telemetry.addData("Status", drive.servo0.getPosition());

            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;

            drive.servo0.setDirection(Servo.Direction.FORWARD);



          if(gamepad1.a) {
              servo0Position = 0.2;

            }
          if(gamepad1.b){
              drive.servo0.setDirection(Servo.Direction.REVERSE);
              servo0Position = 0.6;
          }

          drive.servo0.setPosition(servo0Position);

            telemetry.addData("r", r);
            telemetry.addData("robotAngle", robotAngle);
            telemetry.addData("rightX", rightX);
            telemetry.addData("stickx", gamepad1.right_stick_x);
            telemetry.addData("sticky", gamepad1.right_stick_y);
            telemetry.addData("bbutton", gamepad1.b);
            telemetry.addData("LastGivenDirection", drive.servo0.getPosition());
            telemetry.addData("servoDirection", drive.servo0.getDirection());
            telemetry.addData("servoPositionVariable", servo0Position);
            telemetry.addData("abutton", gamepad1.a);
            telemetry.update();
        }
    }
}
