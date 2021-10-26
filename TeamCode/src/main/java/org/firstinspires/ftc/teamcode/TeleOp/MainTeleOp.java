package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Freight Frenzy TeleOp")
public class MainTeleOp extends LinearOpMode {
    public void runOpMode() {

        DcMotor bl, br, fl, fr, intake, duckWheel, arm;
        Servo gate;
        double straight, strafe, rotation, targetPos = 0;

        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        intake = hardwareMap.get(DcMotor.class, "intake");
        duckWheel = hardwareMap.get(DcMotor.class, "Rim");
        arm = hardwareMap.get(DcMotor.class, "arm");
        gate = hardwareMap.get(Servo.class, "gate");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        gate.scaleRange(0,1);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()){

            straight = -gamepad1.left_stick_y;
            strafe = -gamepad1.left_stick_x;
            rotation = gamepad1.right_stick_x;

            bl.setPower(straight + strafe + rotation);
            br.setPower(straight - strafe - rotation);
            fl.setPower(straight - strafe + rotation);
            fr.setPower(straight + strafe - rotation);

            if(gamepad1.left_bumper){
                duckWheel.setPower(.75);
            }else{
                duckWheel.setPower(0);
            }

            if(gamepad1.right_trigger > 0){
                intake.setPower(gamepad1.right_trigger);
            }else if(gamepad1.left_trigger > 0){
                intake.setPower(-gamepad1.left_trigger);
            }else{
                intake.setPower(0);
            }

            targetPos += gamepad2.left_stick_y * .75;

            if(gamepad1.a){
                targetPos = -280;
            }


            arm.setTargetPosition((int)targetPos);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1);

            if(gamepad1.y){
                gate.setPosition(.5);
            }else{
                gate.setPosition(0);
            }

            telemetry.addData("target pos", targetPos);
            telemetry.addData("left stick y", gamepad1.left_stick_y);
            telemetry.addData("motor power", intake.getPower());
            telemetry.addData("duck wheel", duckWheel.getPower());
            telemetry.update();
        }
    }
}
