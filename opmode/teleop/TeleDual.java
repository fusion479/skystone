package org.firstinspires.ftc.teamcode.opmode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Hook;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.Lock;

@TeleOp(name="TeleDual", group="Teleop")
public class TeleDual extends LinearOpMode{


        private Drivetrain drive = new Drivetrain(this);
        private Claw claw = new Claw(this);
        private Lift lift = new Lift(this);
        private Hook hook = new Hook(this);
        private Lock lock = new Lock(this);
        private Acquirer acquirer = new Acquirer(this);

        @Override
        public void runOpMode() throws InterruptedException{
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

            int modeToggle = 0;
            int gripToggle = 1;
            int lockToggle = 1;
            int swingToggle = 1;

            while(opModeIsActive()){
                double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
                double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
                double rightX = gamepad1.right_stick_x;

                drive.teleDrive(r, robotAngle, rightX);

                if (gamepad1.start){
                    if (modeToggle % 2 == 0){
                        if (gamepad1.left_trigger > 0) acquirer.teleOuttake(gamepad1.left_trigger);
                        else if (gamepad1.right_trigger > 0 ) acquirer.teleIntake(gamepad1.right_trigger);

                        if (gamepad1.a){
                            if (claw.getGripped()){
                                claw.open();
                            } else{claw.close(); }

                        }

                        if (gamepad1.x){
                            if (lock.getLocked()){
                                lock.unlock();
                            } else {
                                lock.lock();
                            }

                        }
                    }
                    else if (modeToggle % 2 == 1 ){
                        drive.reverse();
                        if (gamepad1.right_trigger > 0){
                            lift.liftUp(gamepad1.right_trigger);

                        }
                        if (gamepad2.left_trigger > 0){
                            lift.liftDown(gamepad1.left_trigger);
                        }
                        if (gamepad1.a){
                            if (claw.getGripped()){
                                claw.open();
                            } else {claw.close();}


                        }
                        if (gamepad1.x){
                            if (claw.getSwinged()){
                                claw.front();
                            } else {
                                claw.back();
                            }
                        }
                        if (gamepad1.y){
                            drive.setSlow();
                        }

                    }
                }



                telemetry.addData("slow mode", drive.getSlow());
                telemetry.addData("reverse mode", drive.getReverse());
                telemetry.addData("r", r);
                telemetry.addData("robot angle", robotAngle);
                telemetry.addData("rightX", rightX);
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