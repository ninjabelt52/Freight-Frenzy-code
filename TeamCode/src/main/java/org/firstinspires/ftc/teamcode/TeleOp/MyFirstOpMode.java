package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MyFirstOpMode extends LinearOpMode {

    DcMotor frontleftMotor;
    DcMotor frontrightMotor;
    DcMotor backleftMotor;
    DcMotor backrightMotor;

    double power = 0.0;
    double rotation = 0.0;
    double strafe = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {

        frontleftMotor = hardwareMap.dcMotor.get("fl");
        frontrightMotor = hardwareMap.dcMotor.get("fr");
        backleftMotor = hardwareMap.dcMotor.get("bl");
        backrightMotor = hardwareMap.dcMotor.get("br");

        frontleftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backleftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
       while (opModeIsActive()) {
        frontleftMotor.setPower(power+strafe+rotation);
        frontrightMotor.setPower(power-strafe-rotation);
        backleftMotor.setPower(power-strafe+rotation);
        backrightMotor.setPower(power+strafe-rotation);


        power = gamepad1.left_stick_y;
        rotation = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;

       }
    }
}
