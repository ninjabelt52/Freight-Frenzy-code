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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

import java.util.Locale;


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

    BNO055IMU imu;

    Orientation angles;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        RevBlinkinLedDriver blinkinLedDriver;
        RevBlinkinLedDriver.BlinkinPattern pattern;

        //imu set up!
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        DcMotor Lift1;
        DcMotor Lift2;
        CRServo Slurper;

        boolean toggle = true;
        boolean toggle2 = true;
        boolean toggle3 = true;

        boolean liftOnOff = false;


        //blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftBackDrive  = hardwareMap.get(DcMotor.class, "bl");
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "fl");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");
        rightFrontDrive  = hardwareMap.get(DcMotor.class, "fr");

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

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            float heading = angles.firstAngle;
            double SlurperPower = 0.0;
            int fineTuneLift = 0;

            telemetry.addData("heading", heading);

            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            double radientsHeding = angles.firstAngle * Math.PI/180;
            final double v1 = r * Math.cos(robotAngle - radientsHeding) + rightX;
            final double v2 = r * Math.sin(robotAngle - radientsHeding) - rightX;
            final double v3 = r * Math.sin(robotAngle - radientsHeding) + rightX;
            final double v4 = r * Math.cos(robotAngle - radientsHeding) - rightX;

            leftFrontDrive.setPower(v1);
            rightFrontDrive.setPower(v2);
            leftBackDrive.setPower(v3);
            rightBackDrive.setPower(v4);
            Slurper.setPower(SlurperPower);

            if (gamepad1.dpad_left){
                SlurperPower -= -1;
            }
            if (gamepad1.dpad_right) {
                SlurperPower += .1;
            }


            if(gamepad2.left_trigger > 0.2 && gamepad2.y && liftOnOff == true){
                fineTuneLift = 0;
                setHeight(4, Lift1, Lift2, fineTuneLift );
            }else if(gamepad2.left_trigger > 0.2 && gamepad2.x && liftOnOff == true){
                fineTuneLift = 0;
                setHeight(3, Lift1, Lift2, fineTuneLift);
            }else if(gamepad2.left_trigger > 0.2 && gamepad2.b && liftOnOff == true){
                fineTuneLift = 0;
                setHeight(2, Lift1, Lift2, fineTuneLift);
            }else if(gamepad2.left_trigger > 0.2 && gamepad2.a && liftOnOff == true){
                fineTuneLift = 0;
                setHeight(1, Lift1, Lift2, fineTuneLift);
            }

            if(gamepad2.right_bumper){
                if(toggle){
                    fineTuneLift += 219;
                    setHeight(5, Lift1, Lift2, fineTuneLift);
                    toggle = false;
                }
            }else{
                toggle = true;
            }
            if(gamepad2.left_bumper){
                if(toggle2){
                    fineTuneLift -= 219;
                    setHeight(5, Lift1, Lift2, fineTuneLift);
                    toggle2 = false;
                }
            }else{
                toggle2 = true;
            }


            if(gamepad1.a){
                if(toggle3){
                    if(liftOnOff){
                        liftOnOff = false;
                        turnOffLift(Lift1, Lift2);
                    }else{
                        liftOnOff = true;
                    }
                    toggle3 = false;
                }
            }else{
                toggle3 = true;
            }


            //lightTimer(runtime.seconds(), blinkinLedDriver);



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
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
            lift1.setTargetPosition(lift1.getCurrentPosition() + fineTune);
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
//250, 700, 1200