package org.firstinspires.ftc.teamcode.Old_code.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp
public class DuckWheel extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor rimWheel;
        rimWheel = hardwareMap.get(DcMotor.class, "Rim");

        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.left_bumper){
               rimWheel.setPower(.5);
            }else{
                rimWheel.setPower(0);
            }


        }
    }
}
