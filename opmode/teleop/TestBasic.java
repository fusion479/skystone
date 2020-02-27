package org.firstinspires.ftc.teamcode.opmode.teleop;
import org.firstinspires.ftc.teamcode.hardware.Lock;
import org.firstinspires.ftc.teamcode.hardware.TapeMeasure;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="TestBasic", group = "Teleop")

public class TestBasic extends LinearOpMode {
    private Lock lock = new Lock(this);
    private TapeMeasure tapeMeasure = new TapeMeasure(this);

    @Override
    public void runOpMode()throws InterruptedException{
        lock.init(hardwareMap);
        tapeMeasure.init(hardwareMap);
        while(!opModeIsActive()&&!isStopRequested()){
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }
        while(opModeIsActive()){
            if(gamepad1.x){
                lock.lock();
            }
            if(gamepad1.y){
                lock.unlock();
            }
            if(gamepad1.a) {
                tapeMeasure.extend();
            }
            else if (gamepad1.b){
                tapeMeasure.retract();
            }
            else {
                tapeMeasure.stop();
            }
        }


    }

}
