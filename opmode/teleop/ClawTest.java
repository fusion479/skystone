package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;

@TeleOp(name="ClawTest", group="Teleop")
public class ClawTest extends LinearOpMode {

    private Claw claw = new Claw(this);
//    private Drivetrain drive = new Drivetrain(this);

    @Override
    public void runOpMode(){
//        drive.init(hardwareMap);
        claw.init(hardwareMap);

        while(!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;

            if(gamepad1.a) claw.open();

            if(gamepad1.b) claw.close();

            if(gamepad1.x) claw.front();

            if(gamepad1.y) claw.back();

            telemetry.addData("r", r);
            telemetry.addData("robotAngle", robotAngle);
            telemetry.addData("rightX", rightX);
            telemetry.addData("rightY", gamepad1.right_stick_y);
            telemetry.addData("b button", gamepad1.b);
            telemetry.addData("a button", gamepad1.a);
            telemetry.addData("x button", gamepad1.x);
            telemetry.addData("y button", gamepad1.y);
            telemetry.update();
        }
    }
}
