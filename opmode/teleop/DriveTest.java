package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;

@TeleOp(name="Drive and Claw", group="Teleop")
public class DriveTest extends LinearOpMode {

    private Claw claw = new Claw();
    private Drivetrain drive = new Drivetrain();

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        claw.init(hardwareMap);

        while(!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;

            if(gamepad1.a) claw.openClaw();

            if(gamepad1.b) claw.closeClaw();

            telemetry.addData("r", r);
            telemetry.addData("robotAngle", robotAngle);
            telemetry.addData("rightX", rightX);
            telemetry.addData("stickx", gamepad1.right_stick_x);
            telemetry.addData("sticky", gamepad1.right_stick_y);
            telemetry.addData("bbutton", gamepad1.b);
            telemetry.addData("abutton", gamepad1.a);
            telemetry.update();
        }
    }
}
