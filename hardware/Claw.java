package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw extends Mechanism {
    private Servo grip;
    private Servo swivel;

    public Claw(LinearOpMode opMode) { this.opMode = opMode; }

    public void init(HardwareMap hwMap){
        grip = hwMap.servo.get("grip");
        swivel = hwMap.servo.get("swivel");
    }

    public void open(){
        grip.setDirection(Servo.Direction.FORWARD);
        grip.setPosition(0.2);
    }

    public void close(){ grip.setPosition(0.8); }

    public void front() {
        swivel.setPosition(0);
    }

    public void back() { swivel.setPosition(1); }
}

