package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;


@TeleOp(name="Drive", group="Teleop")
public class DriveTest extends LinearOpMode {

    double leftInput, rightInput, slideInput, launchInput;

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
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;

            drive.teleDrive(r, robotAngle, rightX);


//            leftInput = gamepad1.left_stick_y;
//            rightInput = gamepad1.right_stick_y;
//            slideInput = -gamepad1.left_trigger + gamepad1.right_trigger;

//            drive.tankDriveScaled(leftInput, rightInput, slideInput);
        }
    }
}
