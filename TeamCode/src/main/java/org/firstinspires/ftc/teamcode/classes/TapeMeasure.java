package org.firstinspires.ftc.teamcode.classes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.File;

@Config
public class TapeMeasure implements Runnable{

    private Servo liftL, liftR, zAxis, xAxis;
    private CRServo  feed;
    public static double liftInit = 0, zInit = 0, xInit = 0;
    public static double liftOpen = 1, xOpen = 1;
    public static double targetX = 36, targetY = 36;
    public static double leftBound = 0, rightBound = 1;
    private double lastAngle = 0, globalAngle = 0, baseAngle = 0;
    private boolean toggle;
    BNO055IMU imu;
    private boolean isOpen = false;
    Gamepad gamepad1;
    String telemetry;

    public TapeMeasure(HardwareMap hardwareMap, Gamepad gamepadOne){
        liftL = hardwareMap.get(Servo.class, "liftL");
        liftR = hardwareMap.get(Servo.class, "liftR");
        zAxis = hardwareMap.get(Servo.class, "zAxis");
        xAxis = hardwareMap.get(Servo.class, "xAxis");
        feed = hardwareMap.get(CRServo.class, "feed");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        zAxis.scaleRange(leftBound, rightBound);

        gamepad1 = gamepadOne;

        liftL.setPosition(liftInit);
        liftR.setPosition(liftInit);
        zAxis.setPosition(zInit);
        xAxis.setPosition(xInit);

        while(!imu.isGyroCalibrated()){
            telemetry = "gyro is not calibrated";
        }

        telemetry = "gyro is calibrated";
    }

    public void open (){
        liftL.setPosition(liftOpen);
        liftR.setPosition(liftOpen);
        xAxis.setPosition(xOpen);
        zAxis.setPosition(Math.atan2(targetX, targetY) - robotRotation() / 180);
        isOpen = true;
    }

    public void close(){
        liftL.setPosition(liftInit);
        liftR.setPosition(liftInit);
        xAxis.setPosition(xInit);
        isOpen = false;
    }

    public void extend(double power){
        feed.setPower(power);
    }

    public boolean isOpen(){
        return isOpen;
    }

    public void alternate(){
        if(isOpen){
            close();
        }else{
            open();
        }
    }

    public double robotRotation(){
        double heading = imu.getAngularOrientation().secondAngle;

        double deltaAngle = heading - lastAngle;

        if(lastAngle < -180){
            deltaAngle += 360;
        }else if(lastAngle > 180){
            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;

        lastAngle = heading;

        return globalAngle - baseAngle;
    }

    public void resetRobotRotation(){
        baseAngle = robotRotation();
    }

    public String toString(){
        return telemetry;
    }

    public void run(){
        while(!Thread.interrupted()){
            if(gamepad1.start){
                if(toggle){
                    alternate();
                    toggle = false;
                }
            }else{
                toggle = true;
            }

//          TODO: Need to add code that will only extend the tape when the system is up,
//            and only retract the tape otherwise.
            if(gamepad1.dpad_up){
                extend(1);
            }else if(gamepad1.dpad_down){
                extend(-1);
            }else{
                extend(0);
            }

            telemetry = "" + robotRotation();
        }
    }
}
