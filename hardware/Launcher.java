package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Launcher extends Mechanism {
    public DcMotor center;
    public Launcher() {

    }

    public Launcher(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init(HardwareMap hwMap) {
        center = hwMap.dcMotor.get("launchMotor");
        center.setDirection(DcMotor.Direction.FORWARD);
        center.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        center.setPower(0);
    }

    public void launch(float power) {
        center.setPower(power);
    }
}
