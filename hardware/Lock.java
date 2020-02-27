package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lock extends Mechanism{
    private Servo lock;

    public Lock(LinearOpMode opMode){this.opMode = opMode;}

    public void init(HardwareMap hardwareMap){
        lock = hardwareMap.servo.get("lock");
    }

    public void lock(){lock.setPosition(1);}

    public void unlock(){lock.setPosition(0);}
}
