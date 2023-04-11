package org.firstinspires.ftc.teamcode.tests.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class SampleServo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo;

        servo = hardwareMap.get(Servo.class, "ServoTest");

        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.dpad_left){
                servo.setPosition(0);
            } else if(gamepad1.dpad_right){
                servo.setPosition(1);
            }
        }
    }
}
