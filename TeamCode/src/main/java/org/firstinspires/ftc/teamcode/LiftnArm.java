package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftnArm {
    DcMotor Lift1;
    DcMotor Lift2;
    DcMotor arm;
    Gamepad gamepad1, gamepad2;
    double liftHeight, armHeight;

    public LiftnArm (HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2){
        Lift1 = hardwareMap.get(DcMotor.class, "Lift1");
        Lift2 = hardwareMap.get(DcMotor.class, "Lift2");
        arm = hardwareMap.get(DcMotor.class, "arm");

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void setHeight(double height, int fineTune){
        int targetPos = 0;

        if(height == 0){
            targetPos = 0;
            armHeight = 0;
        }else if(height == 1){
            targetPos = 300;
            armHeight = 1316;
        }else if(height == 2){
            targetPos = 600;
            armHeight = 1316;
        }else if(height == 3){
            targetPos = 900;
            armHeight = 1316;
        }else if(height == 4){
            targetPos = 1200;
            armHeight = 1316;
        }
        liftHeight = targetPos + fineTune;
    }




}
