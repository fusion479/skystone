package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.TapeMeasure;

@Autonomous(name="Tape-Measure-Park")
public class TapePark extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private TapeMeasure tapeMeasure  = new TapeMeasure(this);
    private Claw claw = new Claw(this);

    @Override
    public void runOpMode() throws InterruptedException {
        claw.init(hardwareMap);
        tapeMeasure.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        runtime.reset();
        tapeMeasure.extend();
        sleep(1200);
        tapeMeasure.stop();
    }
}
