package org.firstinspires.ftc.teamcode.hardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Acquirer extends Mechanism{
    private DcMotor leftAcquirer;
    private DcMotor rightAcquirer;

    public Acquirer (LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        leftAcquirer = hwMap.dcMotor.get("leftAcquirer");
        rightAcquirer = hwMap.dcMotor.get("right");

    }
    public void intake(){
        leftAcquirer.setDirection(DcMotorSimple.Direction.FORWARD);
        rightAcquirer.setDirection(DcMotorSimple.Direction.FORWARD);
        leftAcquirer.setPower(1);
        rightAcquirer.setPower(-1);
    }

    public void outtake(){
        leftAcquirer.setDirection(DcMotorSimple.Direction.FORWARD);
        rightAcquirer.setDirection(DcMotorSimple.Direction.FORWARD);
        leftAcquirer.setPower(-1);
        rightAcquirer.setPower(1);
    }
}
