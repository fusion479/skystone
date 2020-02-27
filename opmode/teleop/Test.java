package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.TensorFlowCamera;

import java.util.List;

@TeleOp(name="TensorFlow Test")
public class Test extends LinearOpMode {
    private Drivetrain drive = new Drivetrain(this);
    private Acquirer acquirer = new Acquirer(this);

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        acquirer.init(hardwareMap);

        while(!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }

        while(opModeIsActive()) {
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;

            if (gamepad1.dpad_up) {
                drive.setSlow();
            }

            if (gamepad1.dpad_down) {
                drive.reverse();
            }

            if (gamepad1.right_bumper) {
                acquirer.teleIntake(1);
            } else if (gamepad1.left_bumper) {
                acquirer.teleOuttake(1);
            } else {
                acquirer.stop();
            }

            drive.teleDrive(r, robotAngle, rightX);
        }
    }
}
