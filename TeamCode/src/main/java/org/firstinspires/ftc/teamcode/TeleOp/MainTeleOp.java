package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Freight Frenzy TeleOp")
public class MainTeleOp extends LinearOpMode {
    public void runOpMode() {

        DcMotor bl, br, fl, fr, intake, arm, armSupport;
        DcMotorEx duckWheel;
        Servo gate;
        DigitalChannel limit;
        BNO055IMU imu;
        double straight, strafe, rotation, slowDown = 1, armPower = .5;
        int bottomLimit = 0, armState = 0, targetPos = 0;
        boolean toggle = false;

        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        intake = hardwareMap.get(DcMotor.class, "intake");
        duckWheel = hardwareMap.get(DcMotorEx.class, "Rim");
        arm = hardwareMap.get(DcMotor.class, "arm");
        armSupport = hardwareMap.get(DcMotor.class, "arm2");
        gate = hardwareMap.get(Servo.class, "gate");
        limit = hardwareMap.get(DigitalChannel.class, "limit");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        armSupport.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armSupport.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        duckWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armSupport.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        limit.setMode(DigitalChannel.Mode.INPUT);

        gate.scaleRange(0,1);

        imu.initialize(parameters);

        while(!isStopRequested() && !imu.isGyroCalibrated()){
            telemetry.addLine("not initialized");
            telemetry.update();
        }

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()){

            if(gamepad1.dpad_down){
                slowDown = .5;
            }else{
                slowDown = .95;
            }

            straight = gamepad1.left_stick_y * slowDown;
            strafe = gamepad1.left_stick_x * slowDown;
            rotation = gamepad1.right_stick_x * slowDown * .5;

            bl.setPower(straight + strafe + rotation);
            br.setPower(straight - strafe - rotation);
            fl.setPower(straight - strafe + rotation);
            fr.setPower(straight + strafe - rotation);

            if(gamepad2.x){
                //TODO: I think we should investigate this further in the future, but as for now,
                // competition is next week.
//                int startPos = duckWheel.getCurrentPosition();
//                duckWheel.setVelocity(180, AngleUnit.DEGREES);
//                if(duckWheel.getCurrentPosition() + startPos > 400){
//                    duckWheel.setVelocity(360, AngleUnit.DEGREES);
//                }
                duckWheel.setVelocity(180, AngleUnit.DEGREES);
            }else if(gamepad2.b){
                duckWheel.setVelocity(-180, AngleUnit.DEGREES);
            }else{
                duckWheel.setPower(0);
            }

            if(gamepad1.right_bumper){
                intake.setPower(1);
            }else if(gamepad1.left_bumper){
                intake.setPower(-1);
            }else{
                intake.setPower(0);
            }

            if(!limit.getState()){
                bottomLimit = arm.getCurrentPosition();
            }

            if(arm.getCurrentPosition() <= -400 && !(gamepad1.left_trigger > 0) && !(imu.getAngularOrientation().thirdAngle < (90 - 15))){
                arm.setTargetPosition(bottomLimit - 450);
                armSupport.setTargetPosition(arm.getTargetPosition());
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armSupport.setMode(arm.getMode());
                arm.setPower(armPower);
                armSupport.setPower(arm.getPower());
            }else if(gamepad1.left_trigger > 0 && limit.getState()){
                armPower = .25;
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armSupport.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.setPower(gamepad1.left_trigger * armPower);
                armSupport.setPower(arm.getPower());
                targetPos = arm.getCurrentPosition();
            }else if(gamepad1.right_trigger > 0 && arm.getCurrentPosition() > bottomLimit - 400){
                armPower = .5;
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armSupport.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.setPower(-gamepad1.right_trigger * armPower);
                armSupport.setPower(arm.getPower());
                targetPos = arm.getCurrentPosition();
            }else{
                arm.setTargetPosition(targetPos);
                armSupport.setTargetPosition(arm.getTargetPosition());
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armSupport.setMode(arm.getMode());
                arm.setPower(.5);
                armSupport.setPower(arm.getPower());
            }

            if(gamepad1.a || gamepad1.b || gamepad1.x){
                if(toggle){
                   armState ++;
                   toggle = false;
                }
            }else{
                toggle = true;
            }

            if(armState > 1){
                armState = 0;
            }

            if(gamepad1.a && armState == 1){
                targetPos = bottomLimit - 350;
            }else if((gamepad1.a || gamepad1.b || gamepad1.x) && armState == 0){
                targetPos = bottomLimit;
            }else if(gamepad1.b && armState == 1){
                targetPos = bottomLimit - 150;
            }else if(gamepad1.x && armState == 1){
                targetPos = bottomLimit - 250;
            }

            if(imu.getAngularOrientation().thirdAngle < (90 - 15)){
                targetPos = bottomLimit - 100;
                armState = 1;
            }

            if(gamepad1.y || gamepad1.right_bumper){
                gate.setPosition(.65);
            }else{
                gate.setPosition(.45);
            }

            telemetry.addData("target pos", targetPos);
            telemetry.addData("current pos", arm.getCurrentPosition() - bottomLimit);
            telemetry.addData("armSupport pos", armSupport.getCurrentPosition());
            telemetry.addData("left stick y", gamepad1.left_stick_y);
            telemetry.addData("motor power", intake.getPower());
            telemetry.addData("duck wheel", duckWheel.getPower());
            telemetry.addData("touch sensor", limit.getState());
            telemetry.addData("arm power", arm.getPower());
            telemetry.addData("armSupport power", arm.getPower());
            telemetry.addData("armState", armState);
            telemetry.update();
        }
    }
}
