package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Hook;
import org.firstinspires.ftc.teamcode.hardware.Lift;

@TeleOp(name="Main", group="Teleop")
public class Main extends LinearOpMode {

    private Drivetrain drivetrain = new Drivetrain(this);
//    private Camera camera = new Camera(this);
    private Claw claw = new Claw(this);
    private Lift lift = new Lift(this);
    private Hook hook = new Hook(this);

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap);
//        camera.init(hardwareMap);
        claw.init(hardwareMap);
        hook.init(hardwareMap);
        lift.init(hardwareMap);

        while(!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;

            drivetrain.teleDrive(r, robotAngle, rightX);

            if (gamepad1.start) drivetrain.setSlow();

            if (gamepad1.b) claw.open();

            if (gamepad1.a) claw.close();

            if (gamepad1.x) hook.hook();

            if (gamepad1.y) hook.unhook();

            if (gamepad1.left_bumper) claw.front(false);

            if (gamepad1.right_bumper) claw.back();

            if (gamepad1.left_trigger > 0) lift.liftDown(gamepad1.left_trigger);

            else if (gamepad1.right_trigger > 0 ) lift.liftUp(gamepad1.right_trigger);

            else lift.liftUp(0);

            telemetry.addData("a", gamepad1.a);
            telemetry.addData("b", gamepad1.b);
            telemetry.addData("l", gamepad1.left_bumper);
            telemetry.addData("r", gamepad1.right_bumper);
            telemetry.addData("lt", gamepad1.left_trigger);
            telemetry.addData("rt", gamepad1.right_trigger);
            telemetry.update();

        }
    }
}
