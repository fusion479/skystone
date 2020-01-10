package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Acquirer extends Mechanism{
    private CRServo wheel1;
    private CRServo wheel2;

    public Acquirer (LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        wheel1 = hwMap.crservo.get("wheel1");
        wheel2 = hwMap.crservo.get("wheel2");

    }
    public void intake(){
        wheel1.setPower(1);
        wheel2.setPower(-1);
    }

    public void output(){
        wheel1.setPower(-1);
        wheel2.setPower(1);
    }
}
