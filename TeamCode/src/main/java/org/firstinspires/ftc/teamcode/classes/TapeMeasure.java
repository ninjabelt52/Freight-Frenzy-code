package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TapeMeasure {

    private Servo liftL, liftR, zAxis, xAxis, feedL, feedR;
    public static double liftInit = 0, zInit = 0, xInit = 0;

    public TapeMeasure(HardwareMap hardwareMap){
        liftL = hardwareMap.get(Servo.class, "liftL");
        liftR = hardwareMap.get(Servo.class, "liftR");
        zAxis = hardwareMap.get(Servo.class, "zAxis");
        xAxis = hardwareMap.get(Servo.class, "xAxis");
        feedL = hardwareMap.get(Servo.class, "feedL");
        feedR = hardwareMap.get(Servo.class, "feedR");

        liftL.setPosition(liftInit);
        liftR.setPosition(liftInit);
        zAxis.setPosition(zInit);
        xAxis.setPosition(xInit);
    }
}
