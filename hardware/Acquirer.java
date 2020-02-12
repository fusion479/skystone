package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Acquirer extends Mechanism{
    private DcMotor left;
    private DcMotor right;

    public Acquirer (LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        left = hwMap.dcMotor.get("left");
        right = hwMap.dcMotor.get("right");

    }
    public void intake(){
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        left.setPower(1);
        right.setPower(-1);
    }

    public void outtake(){
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        left.setPower(-1);
        right.setPower(1);
    }
}
