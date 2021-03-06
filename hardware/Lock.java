package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lock extends Mechanism{
    private Servo lock;
    private boolean locked;

    public Lock(LinearOpMode opMode){this.opMode = opMode;}

    public void init(HardwareMap hardwareMap){
        lock = hardwareMap.servo.get("lock");
    }

    public void lock(){
        lock.setPosition(0.45);
        locked = true;
        opMode.sleep(400);
    }

    public void unlock(){
        lock.setPosition(0);
        locked = false;
        opMode.sleep(400);
    }

    public boolean getLocked(){ return locked;}
}
