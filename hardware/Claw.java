package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw extends Mechanism {
    private Servo grip;
    private Servo swing;
    private boolean swinged;
    private boolean gripped;

    public Claw(LinearOpMode opMode) { this.opMode = opMode; }

    public void init(HardwareMap hwMap){
        grip = hwMap.servo.get("grip");
        swing = hwMap.servo.get("swing");
        front();
    }

    public void open(){
        grip.setPosition(0.3);
        gripped = false;
    }

    public void close(){
        grip.setPosition(0.05);
        gripped = true;
    }

    public void front(){
        swing.setPosition(0.05);
        swinged = false;
    }

    public void back(){
        swing.setPosition(0.5);
        swinged = true;
    }

    public boolean getSwinged(){
        return swinged;
    }
    public boolean getGripped(){
        return gripped;
    }
}

