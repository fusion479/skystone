package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;

@TeleOp(name="Drive", group="Teleop")
public class DriveTest extends LinearOpMode {

    double servo0Position;
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

//            servo1Position = 0;
//            servo2Position = 0;
//            servo3Position = 0;
//            servo4Position = 0;

            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;

          if(gamepad1.a) {
                servo0Position = .8;
                drive.servo0.setDirection(Servo.Direction.REVERSE);


            }
          if(gamepad1.b){
              servo0Position = 0.25;
          }
            drive.servo0.setPosition(servo0Position);
//            if (gamepad1.b) {
//                servo1Position = 0.5;
//            }
//            if(gamepad1.x){
//                servo2Position = 0.5;
//            }
//            if(gamepad1.y){
//                servo3Position = 0.5;
//            }

//            drive.servo0.setPosition(servo0Position);
//            drive.servo2.setPosition(servo2Position);
//            drive.servo3.setPosition(servo3Position);

            telemetry.addData("r", r);
            telemetry.addData("robotAngle", robotAngle);
            telemetry.addData("rightX", rightX);
            telemetry.addData("stickx", gamepad1.right_stick_x);
            telemetry.addData("sticky", gamepad1.right_stick_y);
            telemetry.addData("bbutton", gamepad1.b);
            telemetry.addData("servoPosition", drive.servo0.getPosition());
            telemetry.addData("servoPosition", drive.servo0.getDirection());
            telemetry.addData("abutton", gamepad1.a);
            telemetry.update();
        }
    }
}
