package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TapeMeasure extends Mechanism {
    private DcMotor motor;

    public TapeMeasure(LinearOpMode opMode) {this.opMode = opMode;}
    @Override
    public void init(HardwareMap hwMap) {
        motor = hwMap.dcMotor.get("tape");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
    }

    public void extend() {
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setPower(1);
    }

    public void retract() {
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setPower(1);
    }

    public void stop() {
        motor.setPower(0);
    }
}
