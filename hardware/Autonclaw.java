package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Autonclaw extends Mechanism{
    private Servo swivel;
    private Servo latch;

    public Autonclaw(LinearOpMode opMode) {this.opMode = opMode;}

    public void init(HardwareMap hwMap){
        swivel = hwMap.servo.get("swivel");
        latch = hwMap.servo.get("latch");

    }

    public void open(){latch.setPosition(0);}
    public void close(){latch.setPosition(1);}

    public void swivelin(){swivel.setPosition(1);}
    public void swivelout(){swivel.setPosition(0);}
}
