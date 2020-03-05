package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hook extends Mechanism{

    private Servo left;
    private boolean hooked;
    private Servo right;

    public Hook(LinearOpMode opMode) {this.opMode = opMode;}

    @Override
    public void init(HardwareMap hwMap) {
        left = hwMap.servo.get("leftHook");
        right = hwMap.servo.get("rightHook");
    }

    public void hook() {
        left.setPosition(0);
        right.setPosition(1);
        hooked = true;
        opMode.sleep(400);
    }

    public void unhook() {
        left.setPosition(1);
        right.setPosition(0);
        hooked = false;
        opMode.sleep(400);
    }

    public boolean getHooked() {
        return hooked;
    }
}
