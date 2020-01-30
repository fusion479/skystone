package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.Camera;

@Autonomous(name="Webcam")
public class Webcam extends LinearOpMode {

    Camera camera = new Camera(this);
    @Override
    public void runOpMode() {

        camera.init(hardwareMap);

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

         waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        camera.activateTrackables();

        while (!isStopRequested()) {

            String targetVisible = camera.isTargetVisible();

            if(!targetVisible.equals("none")) {
                // Provide feedback as to where the robot is located (if we know).

                // express position (translation) of robot in inches.
                float[] positions = camera.getLocation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.2f, %.2f, %.2f", positions[0], positions[1], positions[2]);

                // express the rotation of the robot in degrees.
                float[] degrees = camera.getRotation();
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.2f, %.2f, %.2f", degrees[0], degrees[1], degrees[2]);
            }

            telemetry.addData("Visible Target", targetVisible);
            telemetry.update();
        }

        // Disable Tracking when we are done;
        camera.deactivateTrackables();
    }
}
