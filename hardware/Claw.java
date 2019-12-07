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
//        swivel.setDirection(Servo.Direction.REVERSE);
        try {
            front(true);
        } catch (InterruptedException e) {

        }
        open();
    }

    public void open(){grip.setPosition(0.8);}

    public void close(){ grip.setPosition(0.2);}

    public void front(boolean init) throws InterruptedException{
        swivel.setPosition(0.98);
        if(init)
            Thread.sleep(2000);
    }

    public void back() throws InterruptedException{
        close();
        Thread.sleep(2000);
        swivel.setPosition(0.5);
    }
}

