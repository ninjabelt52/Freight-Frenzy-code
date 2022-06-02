package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MyFirstOpMode extends LinearOpMode {
// Define variables
    DcMotor frontleftMotor;
    DcMotor frontrightMotor;
    DcMotor backleftMotor;
    DcMotor backrightMotor;
    BNO055IMU IMU;

    double power = 0.0;
    double rotation = 0.0;
    double strafe = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
// Initialization
        frontleftMotor = hardwareMap.dcMotor.get("fl");
        frontrightMotor = hardwareMap.dcMotor.get("fr");
        backleftMotor = hardwareMap.dcMotor.get("bl");
        backrightMotor = hardwareMap.dcMotor.get("br");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        IMU =hardwareMap.get(BNO055IMU.class,"IMU");
        IMU.initialize(parameters);

        frontleftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backleftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
// IMU calibration
        while (!IMU.isGyroCalibrated()) {
            telemetry.addData("status","IMU is not calibrated");
            telemetry.update();

        }
        telemetry.addData("status","IMU is calibrated");
        telemetry.update();
// Run
        waitForStart();
       while (opModeIsActive()) {
// Motion
        frontleftMotor.setPower(power+strafe+rotation);
        frontrightMotor.setPower(power-strafe-rotation);
        backleftMotor.setPower(power-strafe+rotation);
        backrightMotor.setPower(power+strafe-rotation);

// Set movement variables
        power = gamepad1.left_stick_y;
        rotation = gamepad1.right_stick_x;
        strafe = -gamepad1.left_stick_x;
// Update X, Y, and Z variables
        telemetry.addData("X axis rotation",IMU.getAngularOrientation().firstAngle);
        telemetry.addData("Y axis rotation",IMU.getAngularOrientation().secondAngle);
        telemetry.addData("Z axis rotation",IMU.getAngularOrientation().thirdAngle);
        telemetry.update();
       }
    }
}
