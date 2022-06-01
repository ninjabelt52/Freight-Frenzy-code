package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class MyFirstOpMode extends LinearOpMode {

    DcMotor frontleftMotor;
    DcMotor frontrightMotor;
    DcMotor backleftMotor;
    DcMotor backrightMotor;

    double power = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        frontleftMotor = hardwareMap.dcMotor.get("fl");
        frontrightMotor = hardwareMap.dcMotor.get("fr");
        backleftMotor = hardwareMap.dcMotor.get("bl");
        backrightMotor = hardwareMap.dcMotor.get("br");
        waitForStart();
       while (opModeIsActive()) {
        frontleftMotor.setPower(power);
        frontrightMotor.setPower(power);
        backleftMotor.setPower(power);
        backrightMotor.setPower(power);

        sleep(2000);

        power = 0.0;

        frontleftMotor.setPower(power);
        frontrightMotor.setPower(power);
        backleftMotor.setPower(power);
        backrightMotor.setPower(power);
        }
    }
}
