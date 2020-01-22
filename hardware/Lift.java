package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.hardware.Mechanism;


public class Lift extends Mechanism{
    private DcMotor lift;

    public Lift(LinearOpMode opMode) {this.opMode = opMode;}

    private static final double     COUNTS_PER_MOTOR_REV    = 1120;
    /**
     * Drivetrain gear ratio (< 1.0 if geared up).
     */
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0;
    /**
     * Diameter of wheel in inches.
     */
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0;
    /**
     * Calculated ticks per inch.
     */
    private static final double     COUNTS_PER_INCH         =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    public void init (HardwareMap hwMap){
        lift = hwMap.dcMotor.get("lift");
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setPower(0);
    }

    public void setLift(double power,double inches ){
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int tickCount = (int) (inches * COUNTS_PER_INCH);
        double set_power = power*inches/Math.abs(inches);
        lift.setTargetPosition(tickCount);
        while(opMode.opModeIsActive()&& lift.isBusy()){
            lift.setPower(set_power);
        }
        lift.setPower(0.0);
    }

    public void liftDown(double power) {
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setPower(power);
    }

    public void liftUp(double power) {
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setPower(power);
    }

    public void liftOff() {
        lift.setPower(0);
    }
}
