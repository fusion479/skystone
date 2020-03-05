package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleDuo", group="Teleop")
public class TeleDuo extends LinearOpMode {
    /*
        CONSTRUCT MECHANISMS
     */

    @Override
    public void runOpMode() throws InterruptedException {

        /*
          INITIALIZE MECHANISMS
        */

        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }

        while(opModeIsActive()) {
            /*
                MECANUM DRIVE CONTROLS
             */

            /*
                DRIVER CONDITIONS (gamepad1)
             */

            /*
                OPERATOR CONDITIONS (gamepad2)
             */

            /*
                TELEMETRY
             */
        }
    }
}
