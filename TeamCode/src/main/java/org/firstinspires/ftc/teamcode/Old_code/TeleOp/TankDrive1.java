package org.firstinspires.ftc.teamcode.Old_code.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TankDrive1 extends LinearOpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;
    double xpower = 0.0;
    double ypower = 0.0;

    public void runOpMode() throws InterruptedException {

        leftMotor = hardwareMap.dcMotor.get("l");
        rightMotor = hardwareMap.dcMotor.get("r");

        waitForStart();
        while (opModeIsActive()) {

            leftMotor.setPower(ypower+xpower);
            rightMotor.setPower(ypower-xpower);

            xpower = gamepad1.left_stick_x;
            ypower = gamepad1.left_stick_y;
        }

    }
}
