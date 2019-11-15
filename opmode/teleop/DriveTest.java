package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;

@TeleOp(name="Drive", group="Teleop")
public class DriveTest extends LinearOpMode {

    double servo0Position;
    double servo1Position;
    double servo2Position;

    private Drivetrain drive = new Drivetrain();

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);

        while(!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            servo0Position = 0;
            servo1Position = 0;
            servo2Position = 0;

            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;





            if(gamepad1.b) {
                servo0Position = 0.5;
            }
            if (gamepad1.a) {
                servo0Position = 1.0;
            }

            if(gamepad1.x){
                servo2Position = 0.5;
            }
            if(gamepad1.y){
                servo2Position = 1.0;
            }



            drive.servo0.setPosition(servo0Position);
            drive.servo1.setPosition(servo1Position);
            drive.servo2.setPosition(servo2Position);

            telemetry.addData("r", r);
            telemetry.addData("robotAngle", robotAngle);
            telemetry.addData("rightX", rightX);
            telemetry.addData("stickx", gamepad1.right_stick_x);
            telemetry.addData("sticky", gamepad1.right_stick_y);
            telemetry.addData("bbutton", gamepad1.b);
            telemetry.addData("servoPosition", servo0Position);
            telemetry.addData("abutton", gamepad1.a);
            telemetry.update();
        }
    }
}
