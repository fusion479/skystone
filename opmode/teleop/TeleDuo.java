package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Hook;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.Lock;
import org.firstinspires.ftc.teamcode.hardware.TapeMeasure;

@TeleOp(name="TeleDuo", group="Teleop")
public class TeleDuo extends LinearOpMode {

     private Drivetrain drive = new Drivetrain(this);
     private Claw claw = new Claw(this);
     private Hook hook = new Hook(this);
     private Lift lift = new Lift(this);
     private Lock lock = new Lock(this);
     private TapeMeasure tapeMeasure = new TapeMeasure(this);
     private Acquirer acquirer = new Acquirer(this);

    @Override
    public void runOpMode() throws InterruptedException {


        drive.init(hardwareMap);
        claw.init(hardwareMap);
        hook.init(hardwareMap);
        lift.init(hardwareMap);
        lock.init(hardwareMap);
        acquirer.init(hardwareMap);


        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }

        while(opModeIsActive()) {

               // MECANUM DRIVE CONTROLS
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;

            drive.teleDrive(r,robotAngle,rightX);


             //   DRIVER CONDITIONS (gamepad1)

                if(gamepad1.left_trigger > 0){ acquirer.teleOuttake(gamepad1.left_trigger); }

                if (gamepad1.right_trigger > 0){acquirer.teleIntake(gamepad1.right_trigger);}

                if (gamepad1.right_bumper){hook.hook();}

                if (gamepad1.left_bumper){hook.unhook();}

                if (gamepad1.dpad_up){tapeMeasure.retract();}

                if (gamepad1.dpad_down){tapeMeasure.extend();}

                if(gamepad1.x && lock.getLocked()) lock.unlock();
                else if (gamepad1.x && !lock.getLocked()) lock.lock();

                if (gamepad1.b) drive.setSlow();

                if (gamepad1.a){drive.reverse();}


                // Operator Mode

                if (gamepad2.left_trigger > 0) { lift.liftDown(gamepad2.left_trigger); }

                if (gamepad2.right_trigger > 0){lift.liftUp(gamepad2.right_trigger);}

                if (gamepad2.left_bumper){claw.back();}

                if (gamepad2.right_bumper){claw.front();}

                if (gamepad2.y){claw.fullBack();}

                if(gamepad2.x && lock.getLocked()) lock.unlock();
                else if (gamepad1.x && !lock.getLocked()) lock.lock();

                if (gamepad2.a && claw.getGripped()) claw.open();
                else if (gamepad2.a && !claw.getGripped()) claw.close();

                if (gamepad2.dpad_up){tapeMeasure.retract();}

                if (gamepad2.dpad_down){tapeMeasure.extend();}


               // TELEMETRY

            telemetry.addData()
        }
    }
}
