package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw extends Mechanism {
    private Servo servo0;
    private Servo servo1;

    public Claw() {

    }

    public Claw(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init(HardwareMap hwMap){
        servo0 = hwMap.servo.get("servo0");
        servo1 = hwMap.servo.get("servo1");
    }
    public void open(){
        servo0.setDirection(Servo.Direction.FORWARD);
        servo0.setPosition(0.2);
    }

    public void close(){
        servo0.setDirection(Servo.Direction.REVERSE);
        servo0.setPosition(0.6);
    }

    public void swivel() {
        servo1.setPosition(0);
    }

    public void swivel2() {
        servo1.setPosition(1);
    }
}

