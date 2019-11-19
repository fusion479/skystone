package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardware.Mechanism;

public class Drivetrain extends Mechanism {

    public Servo servo0;
//    public Servo servo1;
//    public Servo servo2;
//    public Servo servo3;
//    public Servo servo4;

    public Drivetrain() {
    }

    public Drivetrain(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init(HardwareMap hwMap) {
        servo0 = hwMap.servo.get("servo0");
//        servo1 = hwMap.servo.get("servo1");
//        servo2 = hwMap.servo.get("servo2");
//        servo3 = hwMap.servo.get("servo3");
//        servo4 = hwMap.servo.get("servo4");
    }

}

