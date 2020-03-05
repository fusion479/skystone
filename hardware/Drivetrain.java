package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.List;

public class Drivetrain extends Mechanism {

    private Camera camera;

    private static boolean slow_mode = false;
    private static boolean reverse_mode = false;

    private static final double COUNTS_PER_MOTOR_REV = 537.6;
    /**
     * Drivetrain gear ratio (< 1.0 if geared up).
     */
    private static final double DRIVE_GEAR_REDUCTION = 1.0;

    /**
     * Diameter of wheel in inches.
     */
    private static final double WHEEL_DIAMETER_INCHES = 4.0;

    /**
     * Calculated ticks per inch.
     */
    private static final double COUNTS_PER_INCH =
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

    private double globalAngle = .30;
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

        pidRotate = new PIDController(0.007, 0.1, 0);
        pidDrive = new PIDController(0.001, 0, 0);
        pidStrafe = new PIDController(0.03, 0, 0);

        current = pidDrive;

        // Set all motors to zero power
        setPower(0.0);

        // Initialize IMU with parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
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
        setPower(v1, v2, v3, v4);
    }

    public void reverse() {
        reverse_mode = !reverse_mode;
    }

    public void driveToPos(double inches, double power) {
        ElapsedTime time = new ElapsedTime();
        time.reset();
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        if (power > 0) {
            frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        } else if (power <= 0) {
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        double correction;
        resetAngle();
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int tickCount = (int) (inches * COUNTS_PER_INCH);
        frontLeft.setTargetPosition(tickCount);
        backLeft.setTargetPosition(tickCount);
        backRight.setTargetPosition(tickCount);
        frontRight.setTargetPosition(tickCount);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double set_power = power * inches / Math.abs(inches);

        while (opMode.opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy() && time.seconds() < 7) {
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

    public float getHeading() {
        return lastAngles.firstAngle;
    }

    public double getGlobal() { return globalAngle; }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    public double getAngle() {
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

    public void setCamera(Camera camera) {
        this.camera = camera;
    }

    public void findStone(double power) {
        int inches = 20;
        boolean foundStone = false;
        ElapsedTime time = new ElapsedTime();
        time.reset();
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        if (power > 0) {
            frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        } else if (power <= 0) {
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        double correction;
        resetAngle();
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int tickCount = (int) (inches * COUNTS_PER_INCH);
        frontLeft.setTargetPosition(tickCount);
        backLeft.setTargetPosition(tickCount);
        backRight.setTargetPosition(tickCount);
        frontRight.setTargetPosition(tickCount);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double set_power = power * inches / Math.abs(inches);

        while (!foundStone && opMode.opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy() && time.seconds() < 3) {
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

            if (camera.getTFod() != null) {
                List<Recognition> updatedRecognitions = camera.getTFod().getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    opMode.telemetry.addData("# Objects Detected", updatedRecognitions.size());
                    for (Recognition recognition : updatedRecognitions) {
                        opMode.telemetry.addData("label", recognition.getLabel());
                        opMode.telemetry.update();
                        if (recognition.getLabel().equals("Skystone")) {
                            opMode.telemetry.addData("Angle", recognition.estimateAngleToObject(AngleUnit.DEGREES));
                            opMode.telemetry.update();
                            foundStone = true;
                        }
                    }
                }
            }
        }
        setPower(0.0);

        if (power < 0) {
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

   public void strafe (double power, double duration){
       frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
       backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
       frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
       backRight.setDirection(DcMotorSimple.Direction.FORWARD);
       ElapsedTime time = new ElapsedTime();
       time.reset();

       setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       resetAngle();
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
     * @param degrees Degrees to turn, + is right - is left

     */
    public void turn(int degrees, double power) {
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
            while (getAngle() == 0 && opMode.opModeIsActive()) {
                setPower(-power, power, -power, power);
            }
            do {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                setPower(power, -power, power, -power);
            }
            while (!pidRotate.onTarget() && opMode.opModeIsActive());
        }
        else    // left turn.
            do {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                setPower(power, -power, power, -power);
            }
            while (!pidRotate.onTarget() && opMode.opModeIsActive());

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

    public boolean getReverse() {return reverse_mode;}

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

    public void acquire(Acquirer acquirer, double power, int inches) {
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        if (power > 0) {
            frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        } else if (power <= 0) {
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        double correction;
        resetAngle();
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int tickCount = (int) (inches * COUNTS_PER_INCH);
        frontLeft.setTargetPosition(tickCount);
        backLeft.setTargetPosition(tickCount);
        backRight.setTargetPosition(tickCount);
        frontRight.setTargetPosition(tickCount);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double set_power = power * inches / Math.abs(inches);

        while (opMode.opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            acquirer.teleIntake(1);
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
        acquirer.stop();

        if (power < 0) {
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }
}
