package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Camera;

//for testing purposes only
@TeleOp(name="WebcamTele")
public class Webcam extends LinearOpMode {

    Camera camera = new Camera(this);
    @Override
    public void runOpMode() {

        camera.init(hardwareMap);

        waitForStart();

        camera.activateTrackables();

        while (!isStopRequested()) {

            String targetVisible = camera.isTargetVisible();

            telemetry.addData("Visible Target", targetVisible);

            if(!targetVisible.equals("none")){
                // Provide feedback as to where the robot is located (if we know).

                // express position (translation) of robot in inches.
                float[] positions = camera.getLocation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f", positions[0], positions[1], positions[2]);

                // express the rotation of the robot in degrees.
                float[] degrees = camera.getRotation();

                //robot is in front of skystone when roll is 90 degrees
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", degrees[0], degrees[1], degrees[2]);
            }

            telemetry.update();
        }

        // Disable Tracking when we are done;
        camera.deactivateTrackables();
    }
}
