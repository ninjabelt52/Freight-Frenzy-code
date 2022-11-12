package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
@Disabled
public class AutoLiftTest extends LinearOpMode {
    public void runOpMode(){
        DcMotor lift1, lift2;

        lift1 = hardwareMap.get(DcMotor.class, "Lift1");
        lift2 = hardwareMap.get(DcMotor.class, "Lift2");

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift1.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        lift1.setTargetPosition(1500);
        lift2.setTargetPosition(lift1.getTargetPosition());
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift1.setPower(1);
        lift2.setPower(lift1.getPower());

        while (lift1.getCurrentPosition() < 1500) {

        }
    }
}
