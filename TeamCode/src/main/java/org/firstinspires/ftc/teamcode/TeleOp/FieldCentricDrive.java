/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Locale;
import java.util.Scanner;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
//@Disabled
public class FieldCentricDrive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor rightFrontDrive = null;

    int bottom = 0;

    BNO055IMU imu;

    Orientation angles;

    @Override
    public void runOpMode() {

        //RevBlinkinLedDriver blinkinLedDriver;
        //RevBlinkinLedDriver.BlinkinPattern pattern;

        //imu set up!
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        DcMotor Lift1;
        DcMotor Lift2;
        DcMotor arm;
        CRServo Slurper;
        TouchSensor Lift, armStop;

        double slowSpeed = .8;

        boolean toggle = true;
        boolean toggle2 = true;
        boolean toggle3 = true;
        boolean armUp = false;

        boolean isLiftUp = false;
        int robotPresetHeight = 1;
        int height = 0;
        int fineTune = 0;
        int armFineTune = 0;

        //blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        String data = "0";
        try {
            File obj = new File("/sdcard/FIRST/nums.txt");
            Scanner scan = new Scanner(obj);

            while (scan.hasNextLine()){
                telemetry.addLine("Reading...");
                telemetry.update();
                data = scan.nextLine();
            }
            scan.close();
        }catch (FileNotFoundException e){
            telemetry.addLine("couldn't read");
            telemetry.update();
            e.printStackTrace();
        }

        double offset = Double.parseDouble(data) +90;


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftBackDrive  = hardwareMap.get(DcMotor.class, "bl");
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "fl");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");
        rightFrontDrive  = hardwareMap.get(DcMotor.class, "fr");
        arm = hardwareMap.get(DcMotor.class, "arm");

        Lift = hardwareMap.get(TouchSensor.class, "Lift");
        armStop = hardwareMap.get(TouchSensor.class, "armStop");
        //armStop = hardwareMap.get(TouchSensor.class, "armStop");

        Lift1 = hardwareMap.get(DcMotor.class, "Lift1");
        Lift2 = hardwareMap.get(DcMotor.class, "Lift2");
        Slurper = hardwareMap.get(CRServo.class, "Slurper");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        Lift2.setDirection(DcMotorSimple.Direction.FORWARD);
        Lift1.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("offset", offset);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        Lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        Lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();

        Lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double heading = angles.firstAngle + offset;

            telemetry.addData("heading", heading);

            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            double radientsHeding = heading * Math.PI/180;
            final double v1 = r * Math.cos(robotAngle - radientsHeding) + rightX;
            final double v2 = r * Math.sin(robotAngle - radientsHeding) - rightX;
            final double v3 = r * Math.sin(robotAngle - radientsHeding) + rightX;
            final double v4 = r * Math.cos(robotAngle - radientsHeding) - rightX;

            leftFrontDrive.setPower(v1 * slowSpeed);
            rightFrontDrive.setPower(v2 * slowSpeed);
            leftBackDrive.setPower(v3 * slowSpeed);
            rightBackDrive.setPower(v4 * slowSpeed);

            if(Lift1.getCurrentPosition() - bottom > 530){
                slowSpeed = .5;
            }else{
                slowSpeed = .8;
            }

            if(gamepad1.a){
                Slurper.setPower(-1);
            }else if(gamepad1.y){
                Slurper.setPower(1);
            }else if(gamepad1.b){
                Slurper.setPower(0);
            }

            if(armUp && isLiftUp){
                arm.setTargetPosition(-500 + armFineTune);
            }else{
                arm.setTargetPosition(0);
            }

            //            if(gamepad1.right_trigger > 0){
//                Lift1.setPower(gamepad1.right_trigger);
//                Lift2.setPower(gamepad1.right_trigger);
//            }else if(gamepad1.left_trigger > 0){
//                Lift1.setPower(-gamepad1.left_trigger);
//                Lift2.setPower(-gamepad1.left_trigger);
//            }

            if(Lift.isPressed()){
                bottom = Lift1.getCurrentPosition();
            }

            if(gamepad2.right_trigger > 0 && gamepad2.y){
                height = 1500;
                fineTune = 0;
                armUp = true;
            }else if(gamepad2.right_trigger > 0 && gamepad2.b){
                height = 200;
                fineTune = 0;
                armUp = true;
            }else if(gamepad2.right_trigger > 0 && gamepad2.x){
                height = 1000;
                fineTune = 0;
                armUp = true;
            }else if(gamepad2.right_trigger > 0 && gamepad2.a){
                height = 500;
                fineTune = 0;
                armUp = false;
            }


            if(gamepad2.dpad_up){
                fineTune = fineTune + 2;
            }else if(gamepad2.dpad_down) {
                fineTune = fineTune - 2;
            }
            if(gamepad2.right_bumper){
                if(toggle2){
                    armFineTune -= 200;
                    toggle2 = false;
                }
            }else{
                toggle2 = true;
            }
            if(gamepad2.left_bumper){
                if(toggle3){
                    armFineTune += 100;
                    toggle3 = false;
                }
            }else{
                toggle3 = true;
            }

//            mid: 1150
//            low: 600


            if(gamepad1.x){
                if(toggle){
                    isLiftUp = !isLiftUp;
                    toggle = false;
                }
            }else{
                toggle = true;
            }
            if(gamepad1.right_stick_button){
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            if(isLiftUp){
                Lift1.setTargetPosition(height + fineTune);
                Lift2.setTargetPosition(Lift1.getTargetPosition());
                Lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lift2.setMode(Lift1.getMode());
                Lift1.setPower(1);
                Lift2.setPower(Lift1.getPower());
            }else{
                Lift1.setTargetPosition(0);
                Lift2.setTargetPosition(Lift1.getTargetPosition());
                Lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lift2.setMode(Lift1.getMode());
                Lift1.setPower(1);
                Lift2.setPower(Lift1.getPower());
            }

//            if(gamepad1.dpad_up){
//                arm.setPower(1);
//            }else if(gamepad1.dpad_down){
//                arm.setPower(-1);
//            }else{
//                arm.setPower(0);
//            }

//            if(armStop.isPressed()){
//                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                arm.setTargetPosition(10);
//            }

            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.5);


            //lightTimer(runtime.seconds(), blinkinLedDriver);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Lift1", Lift1.getCurrentPosition() - bottom);
            telemetry.addData("Lift2", Lift2.getCurrentPosition() - bottom);
            telemetry.addData("liftTouch", Lift.isPressed());
            telemetry.addData("armTouch", armStop.isPressed());
            telemetry.addData("armPos", arm.getCurrentPosition());
            telemetry.addData("height", height);
            telemetry.addData("heading", heading);
            telemetry.update();
        }
    }
    public void lightTimer(double runtime, RevBlinkinLedDriver Blinker){
        double secLightSwitch = 0;
        boolean lightPattern = false;
        if(runtime > 85 && runtime < 90){
            if(secLightSwitch <= 0){
                secLightSwitch = 90 - runtime;
                if (lightPattern == true){
                    Blinker.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                    lightPattern = false;
                }else{
                    Blinker.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    lightPattern = true;
                }
            }
            else if(90 - runtime != secLightSwitch){
                secLightSwitch --;
            }
        }if(runtime >= 90){
            Blinker.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }
    }
    public void setHeight(double height, DcMotor lift1, DcMotor lift2, int fineTune){

        if (height == 1 ||height == 2 ||height == 3 ||height == 4) {
            int targetPos = 0;
            if (height == 1) {
                targetPos = 1;
            } else if (height == 2) {
                targetPos = 250;
            } else if (height == 3) {
                targetPos = 700;
            } else if (height == 4) {
                targetPos = 1200;
            }
            lift1.setTargetPosition(targetPos);
        }else{
            if(lift1.getCurrentPosition() + fineTune > 0 && lift1.getCurrentPosition() + fineTune < 1300) {
                lift1.setTargetPosition(lift1.getCurrentPosition() + fineTune);
            }
        }
        lift2.setTargetPosition(lift1.getCurrentPosition());



        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift1.setPower(0.4);
        lift2.setPower(0.4);
    }
    public void turnOffLift(DcMotor lift1, DcMotor lift2){
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift1.setPower(0);
        lift2.setPower(0);
    }
    public void sluperArm(int Pos, DcMotor slurper){
        slurper.setTargetPosition(Pos * 1316);
        slurper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slurper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slurper.setPower(0.4);
    }
    public void onOffSwitchSlurper(DcMotor slurper){
        slurper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slurper.setPower(0);
    }
}