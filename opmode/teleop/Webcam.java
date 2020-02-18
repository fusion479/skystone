package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.VuforiaCamera;

//for testing purposes only
@TeleOp(name="WebcamTele")

public class Webcam extends LinearOpMode {

    VuforiaCamera camera = new VuforiaCamera(this);
    @Override
    public void runOpMode() {

        camera.init(hardwareMap);

//        waitForStart();

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

                //this sequence probably should not remain within the if statement
                //once the block is picked up, it may not be in the view of the camera
                //if the robot is within a degree margin of error,
                //move forward
                //once the robot is close enough (use positions[0]
                // pick up block with servo
                // move back
                // move to the left and drop off brick
            }

            telemetry.update();
        }

        // Disable Tracking when we are done;
        camera.deactivateTrackables();
    }
}
