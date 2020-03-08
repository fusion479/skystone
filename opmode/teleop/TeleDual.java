package org.firstinspires.ftc.teamcode.opmode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Hook;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.Lock;
import org.firstinspires.ftc.teamcode.hardware.Picker;
import org.firstinspires.ftc.teamcode.hardware.TapeMeasure;

@TeleOp(name="TeleDual", group="Teleop")
public class TeleDual extends LinearOpMode{
    private Drivetrain drive = new Drivetrain(this);
    private Claw claw = new Claw(this);
    private Lift lift = new Lift(this);
    private Hook hook = new Hook(this);
    private Lock lock = new Lock(this);
    private Acquirer acquirer = new Acquirer(this);
    private TapeMeasure tapeMeasure = new TapeMeasure(this);
    private Picker picker = new Picker(this);
    private ElapsedTime runtime = new ElapsedTime();

    private boolean modeToggle = true;

    @Override
    public void runOpMode() throws InterruptedException{
        drive.init(hardwareMap);
        claw.init(hardwareMap);
        hook.init(hardwareMap);
        lift.init(hardwareMap);
        tapeMeasure.init(hardwareMap);
        lock.init(hardwareMap);
        acquirer.init(hardwareMap);
        picker.init(hardwareMap);

        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }

        while(opModeIsActive()){
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;

            drive.teleDrive(r, robotAngle, rightX);

            if (gamepad1.start) {
                double current = runtime.seconds();
                while(gamepad1.start) {
                    if(runtime.seconds() - current > 0.25) {
                        modeToggle = !modeToggle;
                        break;
                    }
                }
            }

            if (modeToggle){
                if(drive.getSlow()) drive.setSlow();
                if(drive.getReverse()) drive.reverse();

                if(gamepad1.b && hook.getHooked()) hook.unhook();
                else if (gamepad1.b && !hook.getHooked()) hook.hook();

                if (gamepad1.left_trigger > 0) acquirer.teleOuttake(gamepad1.left_trigger);
                else if (gamepad1.right_trigger > 0 ) acquirer.teleIntake(gamepad1.right_trigger);
                else acquirer.stop();

                if(gamepad1.x && lock.getLocked()) lock.unlock();
                else if (gamepad1.x && !lock.getLocked()) lock.lock();

                if (gamepad1.left_bumper) tapeMeasure.retract();
                else if (gamepad1.right_bumper) tapeMeasure.extend();
                else tapeMeasure.stop();
            }
            else {
                if(!drive.getReverse()) drive.reverse();
                if (gamepad1.y) drive.setSlow();

                if (gamepad1.right_trigger > 0) lift.liftUp(gamepad1.right_trigger);
                else if (gamepad1.left_trigger > 0) lift.liftDown(gamepad1.left_trigger);
                else lift.liftOff();

                if(gamepad1.x && claw.getSwinged()) claw.front();
                else if (gamepad1.x && !claw.getSwinged()) claw.back();

                if(gamepad1.b) claw.fullBack();
            }

            if(gamepad1.a && claw.getGripped()) claw.open();
            else if (gamepad1.a && !claw.getGripped()) claw.close();

            telemetry.addData("mode toggle", (modeToggle) ? 0 : 1);
            telemetry.addData("slow mode", drive.getSlow());
            telemetry.addData("reverse mode", drive.getReverse());
            telemetry.addData("r", r);
            telemetry.addData("robot angle", robotAngle);
            telemetry.addData("rightX", rightX);
            telemetry.addData("start", gamepad1.start);
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