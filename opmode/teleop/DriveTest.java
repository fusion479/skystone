package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Launcher;


@TeleOp(name="Drive", group="Teleop")
public class DriveTest extends LinearOpMode {

    double leftInput, rightInput, slideInput, launchInput;

    private Drivetrain drive = new Drivetrain();
//    private Launcher launcher = new Launcher();

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
//        launcher.init(hardwareMap);

        while(!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;

            drive.teleDrive(r, robotAngle, rightX);

            if(gamepad1.left_trigger > 0) {
//                launcher.launch(-gamepad1.left_trigger);
            }
            else if (gamepad1.right_trigger > 0) {
//                launcher.launch(gamepad1.right_trigger);
            }
            else if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0) {
//                launcher.launch(0);
            }

            telemetry.addData("r", r);
            telemetry.addData("robotAngle", robotAngle);
            telemetry.addData("rightX", rightX);

            telemetry.addData("stickx", gamepad1.right_stick_x);
            telemetry.addData("sticky", gamepad1.right_stick_y);
            telemetry.update();
//            leftInput = gamepad1.left_stick_y;
//            rightInput = gamepad1.right_stick_y;
//            slideInput = -gamepad1.left_trigger + gamepad1.right_trigger;

//            drive.tankDriveScaled(leftInput, rightInput, slideInput);
        }
    }
}
