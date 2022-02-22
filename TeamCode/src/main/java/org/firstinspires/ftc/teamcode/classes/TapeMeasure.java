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
    public static double liftInit = 0, zInit = 0, xInit = 1;
    public static double liftOpen = 1, xOpen = .53;
    public static double targetX = -36, targetY = -36;
    public static double leftBound = 0, rightBound = 1, bottomBound = 0, topBound = .5;
    private double lastAngle = 0, globalAngle = 0, baseAngle = 0, targetXAxis, slowSpeed;
    public static double targetZ;
    private boolean toggle = false;
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
        liftL.scaleRange(0, .37);
        liftR.scaleRange(0, .33);

        gamepad1 = gamepadOne;

        liftL.setPosition(0);
        liftR.setPosition(1);
        zAxis.setPosition(zInit);
        xAxis.setPosition(xInit);

        while(!imu.isGyroCalibrated()){
            telemetry = "gyro is not calibrated";
        }

        telemetry = "gyro is calibrated";
    }

    public void open (){
        liftL.setPosition(1);
        liftR.setPosition(0);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        xAxis.setPosition(xOpen);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        zAxis.setPosition(1 - Math.atan2(targetY, targetX) - robotRotation() / 180);
        targetZ = zAxis.getPosition();
        targetXAxis = xAxis.getPosition();
        isOpen = true;
    }

    public void close(){
        zAxis.setPosition(zInit);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        xAxis.setPosition(xInit);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        liftL.setPosition(0);
        liftR.setPosition(1);
        targetZ = zAxis.getPosition();
        targetXAxis = xAxis.getPosition();
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
        double heading = imu.getAngularOrientation().firstAngle;

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
                extend(-1);
            }else if(gamepad1.dpad_down){
                extend(1);
            }else{
                extend(0);
            }

            if(isOpen) {
                targetZ += gamepad1.right_stick_x * .0003 * slowSpeed;
                targetXAxis += -gamepad1.right_stick_y * .0003 * slowSpeed;

                if(gamepad1.left_trigger > 0){
                    slowSpeed = .5;
                }else{
                    slowSpeed = 1;
                }

//                if(targetZ > 1)
//                    targetZ = 1;
//                if(targetZ > 0)
//                    targetZ = 0;
//                if(targetXAxis > 1)
//                    targetXAxis = 1;
//                if(targetXAxis < 0)
//                    targetXAxis = 0;

                zAxis.setPosition(targetZ);
                xAxis.setPosition(targetXAxis);
            }

            telemetry = "" + robotRotation() + "\nTarget value: " + (Math.atan2(targetY, targetX));
        }
    }
}
