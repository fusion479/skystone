package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardware.Mechanism;

public class Drivetrain extends Mechanism {
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

//    private DcMotor[] motors;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private BNO055IMU imu;
    private PIDController pidDrive;
    private PIDController pidRotate;

    private double flPower = 0.0, frPower = 0.0, blPower = 0.0, brPower = 0.0;

    public Drivetrain() {
    }

    public Drivetrain(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init(HardwareMap hwMap) {
        frontLeft = hwMap.dcMotor.get("frontLeft");
        frontRight = hwMap.dcMotor.get("frontRight");
        backLeft = hwMap.dcMotor.get("backLeft");
        backRight = hwMap.dcMotor.get("backRight");

        // Set motor direction (AndyMark configuration)
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motor brake behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pidRotate = new PIDController(0.005, 0, 0);
        pidDrive = new PIDController(0.05,0,0);

        // Set all motors to zero power
        setPower(0.0);

        // Initialize IMU with parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void teleDrive(double r, double robotAngle, double rightX) {
        double v1 = r * Math.cos(robotAngle) - rightX;
        double v2 = r * Math.sin(robotAngle) + rightX;
        double v3 = r * Math.sin(robotAngle) - rightX;
        double v4 = r * Math.cos(robotAngle) + rightX;
        setPower(v1,v2,v3,v4);
    }

//    public double trueScaledInput(double joystickValue){
//        double signum = Math.signum(joystickValue);
//        double joystickScale = Math.pow(joystickValue,2) * signum;
//        return joystickScale;
//    }
//
//    public void tankDriveScaled(double leftY, double rightY, double slide){
//        flPower = trueScaledInput(leftY) - trueScaledInput(slide);
//        frPower = trueScaledInput(rightY) + trueScaledInput(slide);
//        blPower = trueScaledInput(leftY) + trueScaledInput(slide);
//        brPower = trueScaledInput(rightY) - trueScaledInput(slide);
//
//        frontLeft.setPower(Range.clip(flPower,-1,1));
//        backLeft.setPower(Range.clip(blPower,-1,1));
//        backRight.setPower(Range.clip(brPower,-1,1));
//        frontRight.setPower(Range.clip(frPower,-1,1));
//    }

    public void driveToPos(double inches, double power) {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int tickCount = (int) (inches * COUNTS_PER_INCH);
        double set_power = power*inches/Math.abs(inches);

        frontLeft.setTargetPosition(tickCount);
        backLeft.setTargetPosition(tickCount);
        backRight.setTargetPosition(tickCount);
        frontRight.setTargetPosition(tickCount);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(opMode.opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            setPower(set_power);
        }

        setPower(0.0);
    }

    private void setPower(double power) {
        setPower(power, power, power, power);
    }

    private void setPower(double FL, double FR, double BL, double BR) {
        frontLeft.setPower(FL);
        backRight.setPower(BR);
        backLeft.setPower(BL);
        frontRight.setPower(FR);
    }

    public void turn(double angle) {

    }
}

