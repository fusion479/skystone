package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.hardware.Mechanism;


public class LinearSlide extends Mechanism{
    private DcMotor ravel;

    public LinearSlide(LinearOpMode opMode) {this.opMode = opMode;}

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
        ravel = hwMap.dcMotor.get("lift");
        ravel.setDirection(DcMotorSimple.Direction.FORWARD);
        ravel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ravel.setPower(0);
    }

    public void setRavel(double power,double inches ){
        ravel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int tickCount = (int) (inches * COUNTS_PER_INCH);
        double set_power = power*inches/Math.abs(inches);
        ravel.setTargetPosition(tickCount);
        while(opMode.opModeIsActive()&& ravel.isBusy()){
            ravel.setPower(set_power);
        }
        ravel.setPower(0.0);
    }

}
