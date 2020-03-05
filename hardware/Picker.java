package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Picker extends Mechanism{
    private Servo swivel;
    private Servo latch;

    public Picker(LinearOpMode opMode) {this.opMode = opMode;}

    public void init(HardwareMap hardwareMap){
        swivel = hardwareMap.servo.get("swivel");
        latch = hardwareMap.servo.get("latch");
        retract();
        latch();
    }

    public void latch(){
        latch.setPosition(1);
    }
    public void delatch(){
        latch.setPosition(0);
    }

    public void retract(){
        swivel.setPosition(1);
    }

    public void extend() {
        swivel.setPosition(0);
    }
}
