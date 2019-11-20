package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw extends Mechanism {
    private Servo servo0;

    public Claw() {

    }

    public Claw(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init(HardwareMap hwMap){
        servo0 = hwMap.servo.get("servo0");
    }
    public void openClaw(){
        servo0.setDirection(Servo.Direction.FORWARD);
        servo0.setPosition(0.2);
    }

    public void closeClaw(){
        servo0.setDirection(Servo.Direction.REVERSE);
        servo0.setPosition(0.6);
    }
}

