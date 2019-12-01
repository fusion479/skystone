package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.Camera;

@Autonomous(name="Webcam")
public class Webcam extends LinearOpMode {

    private Camera camera = new Camera(this);
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

            telemetry.addData("Visible Target", targetVisible);

            if(!targetVisible.equals("none")){
                // Provide feedback as to where the robot is located (if we know).

                // express position (translation) of robot in inches.
                float[] positions = camera.getLocation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f", positions[0], positions[1], positions[2]);

                // express the rotation of the robot in degrees.
                float[] degrees = camera.getRotation();
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", degrees[0], degrees[1], degrees[2]);
            }

            //this sequence probably should not remain within the if statement
            //once the block is picked up, it may not be in the view of the camera
            //if the robot is within a degree margin of error,
            //move forward
            //once the robot is close enough (use positions[0]
            // pick up block with servo
            // move back
            // move to the left and drop off brick

            telemetry.update();
        }

        // Disable Tracking when we are done;
        camera.deactivateTrackables();
    }
}
