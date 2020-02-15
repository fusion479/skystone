package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardware.Mechanism;

public class Drivetrain extends Mechanism {

    private Camera camera;

    private static boolean slow_mode = false;
    private static boolean reverse_mode = false;

    private static final double COUNTS_PER_MOTOR_REV = 537.6;
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


    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    public BNO055IMU imu;
    private PIDController pidDrive;
    private PIDController pidRotate;
    private PIDController pidStrafe;

    private PIDController current;

    private int coefficientIndex;
    private int controllerIndex;

    private double  globalAngle = .30;
    private Orientation lastAngles = new Orientation();

    public Drivetrain(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init(HardwareMap hwMap) {

        frontLeft = hwMap.dcMotor.get("frontLeft");
        frontRight = hwMap.dcMotor.get("frontRight");
        backLeft = hwMap.dcMotor.get("backLeft");
        backRight = hwMap.dcMotor.get("backRight");

        // Set motor direction (AndyMark configuration)
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set motors to run without encoders
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set motor brake behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pidRotate = new PIDController(0.005, 0.1, 0);
        pidDrive = new PIDController(0.04,0,0);
        pidStrafe = new PIDController(0.01, 0, 0);

        current = pidDrive;

        // Set all motors to zero power
        setPower(0.0);

        // Initialize IMU with parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    private void setPower(double power) {
        setPower(power, power, power, power);
    }

    public void setPower(double FL, double FR, double BL, double BR) {
        frontLeft.setPower(FL);
        backRight.setPower(BR);
        backLeft.setPower(BL);
        frontRight.setPower(FR);
    }

    private void setMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        backRight.setMode(mode);
        backLeft.setMode(mode);
        frontRight.setMode(mode);
    }

    public void teleDrive(double r, double robotAngle, double rightX) {
        double multiplier = (slow_mode) ? 0.5 : 1;
        double reversed = (reverse_mode) ? -1 : 1;
        double v1 = reversed * multiplier * r * Math.sin(robotAngle) - multiplier * rightX;
        double v2 = reversed * multiplier * r * Math.cos(robotAngle) + multiplier * rightX;
        double v3 = reversed * multiplier * r * Math.cos(robotAngle) - multiplier * rightX;
        double v4 = reversed * multiplier * r * Math.sin(robotAngle) + multiplier * rightX;
        setPower(v1,v2,v3,v4);
    }

    public void reverse() {
        if (reverse_mode) {
            reverse_mode = false;
        } else {
            reverse_mode = true;
        }
    }

    public void driveToPos(double inches, double power) {
        if (power > 0) {
            frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else if (power <= 0) {
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        double correction;
        resetAngle();
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int tickCount = (int) (inches * COUNTS_PER_INCH);
        double set_power = power*inches/Math.abs(inches);

        frontLeft.setTargetPosition(tickCount);
        backLeft.setTargetPosition(tickCount);
        backRight.setTargetPosition(tickCount);
        frontRight.setTargetPosition(tickCount);

        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(opMode.opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            pidDrive.setSetpoint(0);
            pidDrive.setOutputRange(0, set_power);
            pidDrive.setInputRange(-90, 90);
            pidDrive.enable();
            correction = pidDrive.performPID(getAngle());

            if (Math.signum(inches) > 0) {
                setPower(set_power + correction, set_power - correction, set_power + correction, set_power - correction);
            } else if (Math.signum(inches) < 0) {
                setPower(set_power - correction, set_power + correction, set_power - correction, set_power + correction);
            }
            opMode.telemetry.addData("angle", getAngle());
            opMode.telemetry.addData("correction", correction);
            opMode.telemetry.update();
        }
        setPower(0.0);

        if (power < 0) {
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

//    public float getHeading() {
//        return lastAngles.firstAngle;
//    }
//
//    public double getGlobal() { return globalAngle; }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    public double getAngle()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public void getCamera(Camera camera) {
        this.camera = camera;
    }

    // negative = right positive = left
    public int find_stone(double power){
        ElapsedTime time = new ElapsedTime();
        int pattern;
        time.reset();

        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pidStrafe.reset();
        pidStrafe.setSetpoint(0);
        pidStrafe.setOutputRange(0, power);
        pidStrafe.setInputRange(-90, 90);
        pidStrafe.enable();

        while(opMode.opModeIsActive() && camera.isTargetVisible().equals("none") && time.seconds() < 5.5) {
            opMode.telemetry.addData("target", camera.isTargetVisible());
            opMode.telemetry.update();
            double corrections = pidStrafe.performPID(getAngle());
            setPower( power + corrections, - power - corrections, - power + corrections, power - corrections);
        }

        if (camera.isTargetVisible().equals("Stone Target") && time.seconds() <= 2.75) {
            pattern = 1;
        } else if (camera.isTargetVisible().equals("Stone Target") && time.seconds() < 5.5) {
            pattern = 2;
        } else {
            pattern = 3;
        }
        opMode.telemetry.addData("pattern", pattern);
        opMode.telemetry.update();

        setPower(0.0);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return pattern;
    }

   public void strafe (double power, double duration){
       ElapsedTime time = new ElapsedTime();
       time.reset();

       setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       pidStrafe.reset();
       pidStrafe.setSetpoint(0);
       pidStrafe.setOutputRange(0, power);
       pidStrafe.setInputRange(-90, 90);
       pidStrafe.enable();

       while(opMode.opModeIsActive() && (time.seconds() <= duration)) {
           double corrections = pidStrafe.performPID(getAngle());
           setPower( power + corrections, - power - corrections, - power + corrections, power - corrections);
       }

       setPower(0.0);
       setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       setMode(DcMotor.RunMode.RUN_USING_ENCODER);
   }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right

     */
    public void turn(int degrees, double power) {
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // restart imu angle tracking.
        resetAngle();
        double p = Math.abs(power/degrees);
        double i = p / 115.0;
        pidRotate.setPID(p, i, 0);

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1.0 / Math.abs(degrees) * 115.0);
        pidRotate.enable();
        //Negative --> right ,Positive --> left
        if (degrees < 0) {
            while (getAngle() == 0) {
                setPower(-power, power, -power, power);
            }
            do {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                setPower(power, -power, power, -power);
            }
            while (!pidRotate.onTarget());
        }
        else    // left turn.
            do {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                setPower(power, -power, power, -power);
            }
            while (!pidRotate.onTarget());

        setPower(0);
        resetAngle();
    }

    public void setSlow() {
        if(slow_mode) {
            slow_mode = false;
        } else {
            slow_mode = true;
        }
    }

    public boolean getSlow() { return slow_mode; }

    public String coefficient() {
        return (coefficientIndex == 0)
            ? "P"
            : (coefficientIndex == 1)
                ? "I"
                : "D";
    }

    public void changeCoefficient() { coefficientIndex = (coefficientIndex == 2) ? 0 : coefficientIndex + 1; }

    public void changeController() {
        if(controllerIndex == 1) {
            controllerIndex = 0;
            current = pidDrive;
        }
        else {
            controllerIndex++;
            current = pidRotate;
        }
    }

    public double[] getCoefficients() { return new double[]{current.getP(), current.getI(), current.getD()}; }

    public String controller() { return (controllerIndex == 0) ? "drive" : "turn"; }

    public void increaseCoefficient() { setCoefficient(0.001); }

    public void decreaseCoefficient() { setCoefficient(-0.001); }

    private void setCoefficient(double change) {
        if(coefficientIndex == 0)
            current.setPID(current.getP() + change, current.getI(), current.getD() );
        else if(coefficientIndex == 1)
            current.setPID(current.getP(), current.getI() + change, current.getD() );
        else if(coefficientIndex == 2)
            current.setPID(current.getP(), current.getI(), current.getD() + change );
    }
}
