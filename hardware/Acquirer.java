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
        leftAcquirer = hwMap.dcMotor.get("leftAcquire");
        rightAcquirer = hwMap.dcMotor.get("rightAcquire");
        leftAcquirer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightAcquirer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftAcquirer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightAcquirer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void teleOuttake(float power){
        leftAcquirer.setDirection(DcMotorSimple.Direction.REVERSE);
        rightAcquirer.setDirection(DcMotorSimple.Direction.FORWARD);
        leftAcquirer.setPower(power);
        rightAcquirer.setPower(power);
    }

    public void teleIntake(float power){
        leftAcquirer.setDirection(DcMotorSimple.Direction.FORWARD);
        rightAcquirer.setDirection(DcMotorSimple.Direction.REVERSE);
        leftAcquirer.setPower(power);
        rightAcquirer.setPower(power);
    }

    public void stop() {
        leftAcquirer.setPower(0);
        rightAcquirer.setPower(0);
    }
}
