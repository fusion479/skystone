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
    TensorFlowCamera camera = new TensorFlowCamera(this);
    Drivetrain drive = new Drivetrain(this);
    Acquirer acquirer = new Acquirer(this);

    @Override
    public void runOpMode() throws InterruptedException {
        camera.init(hardwareMap);
        drive.init(hardwareMap);
        acquirer.init(hardwareMap);

        TFObjectDetector tfod = camera.getTFod();
        if (tfod != null) {
            tfod.activate();
        }

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

            if(gamepad1.dpad_down) {
                drive.reverse();
            }

            if(gamepad1.right_bumper) {
                acquirer.teleIntake(1);
            } else if (gamepad1.left_bumper) {
                acquirer.teleOuttake(1);
            } else {
                acquirer.stop();
            }

            drive.teleDrive(r, robotAngle, rightX);

            if(tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
//                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
//                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
//                                recognition.getLeft(), recognition.getTop());
//                        telemetry.addData(String.format("  right,bottom (%d)", i),
//                                "%.03f , %.03f",
//                                recognition.getRight(), recognition.getBottom());
                        telemetry.addData("Width", recognition.getWidth());
                        telemetry.addData("Height", recognition.getImageHeight());
                        telemetry.addData("Angle", recognition.estimateAngleToObject(AngleUnit.DEGREES));
                        telemetry.addData("Confidence", recognition.getConfidence());
                    }
                    telemetry.update();
                }
            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }
    }
}
