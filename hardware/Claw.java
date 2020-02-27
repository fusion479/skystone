package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw extends Mechanism {
    private Servo grip;
    private Servo swing;

    public Claw(LinearOpMode opMode) { this.opMode = opMode; }

    public void init(HardwareMap hwMap){
        grip = hwMap.servo.get("grip");
        swing = hwMap.servo.get("swing");
    }

    public void open(){grip.setPosition(0.9);}

    public void close(){ grip.setPosition(0.2);}

    public void front(){
        swing.setPosition(0.1);
    }

    public void back(){
        swing.setPosition(1);
    }
}

