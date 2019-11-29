package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardware.Mechanism;

import java.util.LinkedHashMap;
import java.util.Map;

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

    Map<String, DcMotor> motors = new LinkedHashMap<>();

    public BNO055IMU imu;
    private PIDController pidDrive;
    private PIDController pidRotate;

    double  globalAngle = .3;
    Orientation lastAngles = new Orientation();

    public Drivetrain(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init(HardwareMap hwMap) {
        String[] names = new String[] {"frontLeft", "frontRight", "backLeft", "backRight"};

        // Set each motor to refer to the motors on the phone configuration
        for(String name: names) { motors.put(name, hwMap.dcMotor.get(name)); }

        // Set motor direction
        for(String motor: motors.keySet()) {
            // left motors go forward
            if(motor.contains("Left")) motors.get(motor).setDirection(DcMotorSimple.Direction.FORWARD);
            // right motors go backwards
            else motors.get(motor).setDirection(DcMotorSimple.Direction.REVERSE);
        }

        // Set motor brake behavior to brake
        for(DcMotor motor: motors.values()) { motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); }

        pidRotate = new PIDController(0.005, 0.1, 0);
        pidDrive = new PIDController(0.05,0,0);

        // Set all motors to zero power
        setPower(0.0);

        // Initialize IMU with parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    private void setPower(double power) {
        setPower(new double[]{power, power, power, power});
    }

    public void setPower(double[] powers) {
        // sequence is frontLeft, frontRight, backLeft, backRight
        DcMotor[] dcMotors = motors.values().toArray(new DcMotor[motors.size()]);
        for(int motor = 0; motor < 4; dcMotors[motor].setPower(powers[motor]), motor++) { }
    }

    public void setMode(DcMotor.RunMode mode) {
        for(DcMotor motor: motors.values()) { motor.setMode(mode); }
    }

    public void teleDrive(double r, double robotAngle, double rightX) {
        double v1 = r * Math.sin(robotAngle) - rightX; //frontLeft
        double v2 = r * Math.cos(robotAngle) + rightX; //frontRight
        double v3 = r * Math.cos(robotAngle) - rightX; //backLeft
        double v4 = r * Math.sin(robotAngle) + rightX; //backRight
        setPower(new double[]{v1, v2, v3, v4});
    }

    public void driveToPos(double inches, double power) {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int tickCount = (int) (inches * COUNTS_PER_INCH);
        double set_power = power*inches/Math.abs(inches);

        for(DcMotor motor : motors.values()) { motor.setTargetPosition(tickCount); }

        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        boolean isBusy = true;

        while(opMode.opModeIsActive() && isBusy) {
            for(DcMotor motor : motors.values()) { if (!motor.isBusy()) isBusy = false; }
            setPower(set_power);
        }

        setPower(0.0);
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    public float getHeading() { return lastAngles.firstAngle; }

    public double getGlobal() { return globalAngle; }

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

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void turn(int degrees, double power) {
        // restart imu angle tracking.
        resetAngle();
        double p = Math.abs(power/degrees);
        double i = p / 115.0;
        pidRotate.setPID(p, i, 0);

        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1.0 / Math.abs(degrees) * 115.0);
        pidRotate.enable();
        if (degrees < 0) {
            while (getAngle() == 0) {
                setPower(new double[] {-power, power, -power, power});
            }
            do {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                setPower(new double[] {power, -power, power, -power});
            }
            while (!pidRotate.onTarget());
        }
        else    // left turn.
            do {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                setPower(new double[] {power, -power, power, -power});
            }
            while (!pidRotate.onTarget());

        setPower(0);
        resetAngle();
    }
}
