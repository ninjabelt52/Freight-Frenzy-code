package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.classes.TapeMeasure;

import java.net.DatagramPacket;
import java.net.DatagramSocket;

@TeleOp(name = "TeleDrive LinearOpMode", group = "")
public class TeleDrive_LinearOpMode extends LinearOpMode {
    private DatagramSocket socket;
    private boolean canRunGamepadThread;
    private Thread gamepadHandler;

    private void startGamepadHandlerThread() {
        telemetry.setAutoClear(true);
        gamepadHandler = new Thread(new Runnable() {
            @Override
            public void run() {
                while (canRunGamepadThread) {
                    String gamepadAction = "";
                    try {
                        byte[] buffer = new byte[1024];
                        DatagramPacket response = new DatagramPacket(buffer, buffer.length);
                        socket.receive(response);
                        gamepadAction = new String(buffer, 0, response.getLength());
                    } catch (Exception ignore) {

                    }

                    if (!gamepadAction.isEmpty()) {
                        if(gamepadAction.contains("E-STOP")) {
                            requestOpModeStop();
                        }
                        if (gamepadAction.contains("G1")) {
                            if (gamepadAction.contains("_A")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.a = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.a = false;
                                }
                            }
                            if (gamepadAction.contains("_B")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.b = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.b = false;
                                }
                            }
                            if (gamepadAction.contains("_X")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.x = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.x = false;
                                }
                            }
                            if (gamepadAction.contains("_Y")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.y = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.y = false;
                                }
                            }
                            if (gamepadAction.contains("_D")) {
                                if (gamepadAction.contains("DU")) {
                                    gamepad1.dpad_up = true;
                                    gamepad1.dpad_down = false;
                                    gamepad1.dpad_left = false;
                                    gamepad1.dpad_right = false;
                                }
                                if (gamepadAction.contains("DD")) {
                                    gamepad1.dpad_down = true;
                                    gamepad1.dpad_up = false;
                                    gamepad1.dpad_left = false;
                                    gamepad1.dpad_right = false;
                                }
                                if (gamepadAction.contains("DL")) {
                                    gamepad1.dpad_left = true;
                                    gamepad1.dpad_up = false;
                                    gamepad1.dpad_down = false;
                                    gamepad1.dpad_right = false;
                                }
                                if (gamepadAction.contains("DR")) {
                                    gamepad1.dpad_right = true;
                                    gamepad1.dpad_up = false;
                                    gamepad1.dpad_down = false;
                                    gamepad1.dpad_left = false;
                                }
                                if (gamepadAction.contains("NONE")) {
                                    gamepad1.dpad_up = false;
                                    gamepad1.dpad_down = false;
                                    gamepad1.dpad_left = false;
                                    gamepad1.dpad_right = false;
                                }
                            }
                            if (gamepadAction.contains("_RT")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.right_trigger = 1.0f;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.right_trigger = 0.0f;
                                }
                            }
                            if (gamepadAction.contains("_LT")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.left_trigger = 1.0f;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.left_trigger = 0.0f;
                                }
                            }
                            if (gamepadAction.contains("_RB")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.right_bumper = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.right_bumper = false;
                                }
                            }
                            if (gamepadAction.contains("_LB")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.left_bumper = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.left_bumper = false;
                                }
                            }
                            if (gamepadAction.contains("_RS")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.right_stick_button = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.right_stick_button = false;
                                }
                            }
                            if (gamepadAction.contains("_LS")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.left_stick_button = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.left_stick_button = false;
                                }
                            }
                            if (gamepadAction.contains("_START")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.start = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.start = false;
                                }
                            }
                            if (gamepadAction.contains("_BACK")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad1.back = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad1.back = false;
                                }
                            }
                            if (gamepadAction.contains("_LX")) {
                                gamepad1.left_stick_x = Float.parseFloat(gamepadAction.replace("G1_LX_", ""));
                            }
                            if (gamepadAction.contains("_LY")) {
                                gamepad1.left_stick_y = Float.parseFloat(gamepadAction.replace("G1_LY_", ""));
                            }
                            if (gamepadAction.contains("_RX")) {
                                gamepad1.right_stick_x = Float.parseFloat(gamepadAction.replace("G1_RX_", ""));
                            }
                            if (gamepadAction.contains("_RY")) {
                                gamepad1.right_stick_y = Float.parseFloat(gamepadAction.replace("G1_RY_", ""));
                            }
                        }
                        if (gamepadAction.contains("G2")) {
                            if (gamepadAction.contains("_A")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.a = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.a = false;
                                }
                            }
                            if (gamepadAction.contains("_B")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.b = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.b = false;
                                }
                            }
                            if (gamepadAction.contains("_X")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.x = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.x = false;
                                }
                            }
                            if (gamepadAction.contains("_Y")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.y = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.y = false;
                                }
                            }
                            if (gamepadAction.contains("_D")) {
                                if (gamepadAction.contains("DU")) {
                                    gamepad2.dpad_up = true;
                                    gamepad2.dpad_down = false;
                                    gamepad2.dpad_left = false;
                                    gamepad2.dpad_right = false;
                                }
                                if (gamepadAction.contains("DD")) {
                                    gamepad2.dpad_down = true;
                                    gamepad2.dpad_up = false;
                                    gamepad2.dpad_left = false;
                                    gamepad2.dpad_right = false;
                                }
                                if (gamepadAction.contains("DL")) {
                                    gamepad2.dpad_left = true;
                                    gamepad2.dpad_up = false;
                                    gamepad2.dpad_down = false;
                                    gamepad2.dpad_right = false;
                                }
                                if (gamepadAction.contains("DR")) {
                                    gamepad2.dpad_right = true;
                                    gamepad2.dpad_up = false;
                                    gamepad2.dpad_down = false;
                                    gamepad2.dpad_left = false;
                                }
                                if (gamepadAction.contains("NONE")) {
                                    gamepad2.dpad_up = false;
                                    gamepad2.dpad_down = false;
                                    gamepad2.dpad_left = false;
                                    gamepad2.dpad_right = false;
                                }
                            }
                            if (gamepadAction.contains("_RT")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.right_trigger = 1.0f;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.right_trigger = 0.0f;
                                }
                            }
                            if (gamepadAction.contains("_LT")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.left_trigger = 1.0f;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.left_trigger = 0.0f;
                                }
                            }
                            if (gamepadAction.contains("_RB")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.right_bumper = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.right_bumper = false;
                                }
                            }
                            if (gamepadAction.contains("_LB")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.left_bumper = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.left_bumper = false;
                                }
                            }
                            if (gamepadAction.contains("_RS")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.right_stick_button = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.right_stick_button = false;
                                }
                            }
                            if (gamepadAction.contains("_LS")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.left_stick_button = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.left_stick_button = false;
                                }
                            }
                            if (gamepadAction.contains("_START")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.start = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.start = false;
                                }
                            }
                            if (gamepadAction.contains("_BACK")) {
                                if (gamepadAction.contains("P")) {
                                    gamepad2.back = true;
                                } else if (gamepadAction.contains("R")) {
                                    gamepad2.back = false;
                                }
                            }
                            if (gamepadAction.contains("_LX")) {
                                gamepad2.left_stick_x = Float.parseFloat(gamepadAction.replace("G1_LX_", ""));
                            }
                            if (gamepadAction.contains("_LY")) {
                                gamepad2.left_stick_y = Float.parseFloat(gamepadAction.replace("G1_LY_", ""));
                            }
                            if (gamepadAction.contains("_RX")) {
                                gamepad2.right_stick_x = Float.parseFloat(gamepadAction.replace("G1_RX_", ""));
                            }
                            if (gamepadAction.contains("_RY")) {
                                gamepad2.right_stick_y = Float.parseFloat(gamepadAction.replace("G1_RY_", ""));
                            }
                        }
                    }
                }
                gamepadHandler.interrupt();
            }
        });
        gamepadHandler.setName("Gamepad Handler Thread");
        gamepadHandler.setPriority(Thread.NORM_PRIORITY);
        gamepadHandler.start();
    }

    @Override
    public void runOpMode() {

        String address = "192.168.43.1"; //Check "Program and Manage" tab on the Driver Station and verify the IP address
        int port = 11039; //Change as needed
        canRunGamepadThread = false;

        try {
            this.socket = new DatagramSocket(port);
        } catch (Exception ex) {
            telemetry.addData("Exception: ", ex.getMessage());
        }

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Connect your server to " + address + ":" + port, "");
        telemetry.update();

        waitForStart();

        canRunGamepadThread = true;

        startGamepadHandlerThread();


        //CUSTOM CODE GOES HERE

        double gateClosed = .1, gateOpen = .37, bottomClosed = 1, bottomOpen = 0, quasiGateOpen = .2, straight, strafe, rotation;
        int BOTTOM = 175, MIDDLE = 250, HIGH = 350;
        int targetPos = 0;
        int SHARED = 175;

        DcMotor bl, br, fl, fr, intake, arm, armSupport;
        DcMotorEx duckWheel;
        Servo gate, bottom;
        DigitalChannel limit;
        BNO055IMU imu;
        double slowDown = 1, armPower = 1;
        int bottomLimit = 0, level = 0, levelHeight = 350;
        boolean toggle = false, armState = false, toggle2 = false, toggle3 = false;
        boolean rumble1 = true, rumble2 = true, rumble3 = true;
        String currentTarget = "top";
        TapeMeasure tapeMeasure = new TapeMeasure(hardwareMap, gamepad1,gamepad2, TapeMeasure.Side.BLUE);
        Thread tapeMeasureThread = new Thread(tapeMeasure);

        ElapsedTime timer = new ElapsedTime();
        Gamepad.RumbleEffect fastBlip;

        fastBlip = new Gamepad.RumbleEffect.Builder()
                .addStep(1,1,150)
                .addStep(0,0,50)
                .addStep(1,1,150)
                .addStep(0,0,50)
                .addStep(1,1,150)
                .addStep(0,0,50)
                .addStep(1,1,150)
                .addStep(0,0,50)
                .addStep(1,1,150)
                .build();

        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        intake = hardwareMap.get(DcMotor.class, "intake");
        duckWheel = hardwareMap.get(DcMotorEx.class, "Rim");
        arm = hardwareMap.get(DcMotor.class, "arm");
        armSupport = hardwareMap.get(DcMotor.class, "arm2");
        gate = hardwareMap.get(Servo.class, "gate");
        bottom = hardwareMap.get(Servo.class, "bottom");
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
        bottom.scaleRange(0,1);

        imu.initialize(parameters);

        while(!isStopRequested() && !imu.isGyroCalibrated()){
            telemetry.addLine("not initialized");
            telemetry.update();
        }

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        tapeMeasureThread.start();
        timer.reset();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                if (gamepad1.right_bumper) {
                    intake.setPower(1);
                } else if (gamepad1.left_bumper) {
                    intake.setPower(-1);
                } else {
                    intake.setPower(0);
                }

                if(!tapeMeasure.isControlsActive()){
                    if(rumble1 && timer.seconds() >= 75){
                        gamepad1.rumble(500);
                        rumble1 = false;
                    }else if(rumble2 && timer.seconds() >= 85){
                        gamepad1.runRumbleEffect(fastBlip);
                        rumble2 = false;
                    }else if(rumble3 && timer.seconds() >= 115){
                        gamepad1.runRumbleEffect(fastBlip);
                        rumble3 = false;
                    }

                    if (gamepad1.dpad_down) {
                        slowDown = .5;
                    } else {
                        slowDown = 1;
                    }

                    straight = gamepad1.left_stick_y * slowDown;
                    strafe = (gamepad1.left_stick_x * slowDown);
                    rotation = gamepad1.right_stick_x * slowDown * .5;

                    bl.setPower(straight + strafe + rotation);
                    br.setPower(straight - strafe - rotation);
                    fl.setPower(straight - strafe + rotation);
                    fr.setPower(straight + strafe - rotation);

                    if (arm.getCurrentPosition() <= -400 && !(gamepad1.left_trigger > 0) && !(imu.getAngularOrientation().thirdAngle < (90 - 15))) {
                        arm.setTargetPosition(bottomLimit - 450);
                        armSupport.setTargetPosition(arm.getTargetPosition());
                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armSupport.setMode(arm.getMode());
                        arm.setPower(armPower);
                        armSupport.setPower(arm.getPower());
                    } else if (gamepad1.left_trigger > 0 && limit.getState()) {
                        armPower = .5;
                        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        armSupport.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        arm.setPower(gamepad1.left_trigger * armPower);
                        armSupport.setPower(arm.getPower());
                        targetPos = arm.getCurrentPosition();
                    } else if (gamepad1.right_trigger > 0 && arm.getCurrentPosition() > bottomLimit - 400) {
                        armPower = .5;
                        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        armSupport.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        arm.setPower(-gamepad1.right_trigger * armPower);
                        armSupport.setPower(arm.getPower());
                        targetPos = arm.getCurrentPosition();
                    } else {
                        arm.setTargetPosition(targetPos);
                        armSupport.setTargetPosition(arm.getTargetPosition());
                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armSupport.setMode(arm.getMode());
                        arm.setPower(.5);
                        armSupport.setPower(arm.getPower());
                    }

                    if(gamepad1.a){
                        if(toggle){
                            armState = !armState;
                            toggle = false;
                        }
                    }else{
                        toggle = true;
                    }

                    if(gamepad1.a && armState){
                        targetPos = bottomLimit - levelHeight;
                    }else if(gamepad1.a && !armState) {
                        targetPos = bottomLimit;
                    }

                    if(imu.getAngularOrientation().thirdAngle < (90 - 15)){
                        targetPos = bottomLimit - 100;
                        armState = true;
                    }

                    if((arm.getCurrentPosition() <= bottomLimit - 300) && gamepad1.y){
                        gate.setPosition(gateOpen);

                        if(gamepad1.b){
                            bottom.setPosition(bottomOpen);
                        }else {
                            bottom.setPosition(bottomClosed);
                        }
                    }else if(arm.getCurrentPosition() >= bottomLimit - 100 && (gamepad1.right_bumper || gamepad1.y)){
                        gate.setPosition(gateOpen);

                        if(gamepad1.b){
                            bottom.setPosition(bottomOpen);
                        }else {
                            bottom.setPosition(bottomClosed);
                        }
                    }else if(bottomLimit - 100 > arm.getCurrentPosition() && arm.getCurrentPosition() > bottomLimit - 300 && gamepad1.y){
                        bottom.setPosition(bottomOpen);

                        if(gamepad1.x){
                            gate.setPosition(gateOpen);
                        }else{
                            gate.setPosition(quasiGateOpen);
                        }
                    }else{
                        if(gamepad1.b){
                            bottom.setPosition(bottomOpen);
                        }else {
                            bottom.setPosition(bottomClosed);
                        }

                        if(gamepad1.x){
                            gate.setPosition(gateOpen);
                        }else{
                            gate.setPosition(gateClosed);
                        }
                    }

                    if (gamepad2.x) {
                        //TODO: I think we should investigate this further in the future, but as for now,
                        // competition is next week.
//                int startPos = duckWheel.getCurrentPosition();
//                duckWheel.setVelocity(180, AngleUnit.DEGREES);
//                if(duckWheel.getCurrentPosition() + startPos > 400){
//                    duckWheel.setVelocity(360, AngleUnit.DEGREES);
//                }
                        duckWheel.setVelocity(180, AngleUnit.DEGREES);
                    } else if (gamepad2.b) {
                        duckWheel.setVelocity(-180, AngleUnit.DEGREES);
                    } else {
                        duckWheel.setPower(0);
                    }

                    if (!limit.getState()) {
                        bottomLimit = arm.getCurrentPosition();
                    }

                    if (gamepad2.dpad_up) {
                        if (toggle2) {
                            level++;
                            toggle2 = false;
                        }
                    } else if (gamepad2.dpad_down) {
                        if (toggle2) {
                            level--;
                            toggle2 = false;
                        }
                    } else {
                        toggle2 = true;
                    }

                    if (level > 3) {
                        level = 0;
                    } else if (level < 0) {
                        level = 3;
                    }

                    if(level == 0) {
                        levelHeight = SHARED;
                        currentTarget = "SHARED SHIPPING HUB";
                    }else if(level == 1){
                        levelHeight = BOTTOM;
                        currentTarget = "BOTTOM LEVEL";
                    }else if(level == 2){
                        levelHeight = MIDDLE;
                        currentTarget = "MIDDLE LEVEL";
                    }else if(level == 3){
                        levelHeight = HIGH;
                        currentTarget = "TOP LEVEL";
                    }

                    if(gamepad2.right_bumper){
                        if(toggle3){
                            SHARED += 10;
                            toggle3 = false;
                        }
                    }else if(gamepad2.left_bumper){
                        if(toggle3){
                            SHARED -= 10;
                            toggle3 = false;
                        }
                    }else if(gamepad2.right_trigger > 0){
                        if(toggle3){
                            SHARED += 20;
                            toggle3 = false;
                        }
                    }else if(gamepad2.left_trigger > 0){
                        if(toggle3){
                            SHARED -= 20;
                            toggle3 = false;
                        }
                    }else{
                        toggle3 = true;
                    }

//            telemetry.addData("target pos", targetPos);
                    telemetry.addData("current pos", -(arm.getCurrentPosition() - bottomLimit));
                    telemetry.addData("arm pos", armSupport.getCurrentPosition());
//            telemetry.addData("left stick y", gamepad1.left_stick_y);
//            telemetry.addData("motor power", intake.getPower());
//            telemetry.addData("duck wheel", duckWheel.getPower());
//            telemetry.addData("touch sensor", limit.getState());
//            telemetry.addData("arm power", arm.getPower());
//            telemetry.addData("armSupport power", arm.getPower());
//            telemetry.addData("armState", armState);
                    telemetry.addData("SHARED HEIGHT", SHARED);
                    telemetry.addData("gate", gate.getPosition());
                    telemetry.addData("level", level);
                    telemetry.addData("toggle2", toggle2);
                    telemetry.addData("target level", currentTarget);
                    telemetry.addData("time", (int)timer.seconds());
                    telemetry.addData("gyro reading", imu.getAngularOrientation().thirdAngle);
                    telemetry.update();
                }else{
                    straight = gamepad2.left_stick_y * .25;
                    strafe = (gamepad2.left_stick_x * .3);
                    rotation = gamepad2.right_stick_x * .3;

                    bl.setPower(straight + strafe + rotation);
                    br.setPower(straight - strafe - rotation);
                    fl.setPower(straight - strafe + rotation);
                    fr.setPower(straight + strafe - rotation);

                    if (gamepad2.x) {
                        //TODO: I think we should investigate this further in the future, but as for now,
                        // competition is next week.
//                int startPos = duckWheel.getCurrentPosition();
//                duckWheel.setVelocity(180, AngleUnit.DEGREES);
//                if(duckWheel.getCurrentPosition() + startPos > 400){
//                    duckWheel.setVelocity(360, AngleUnit.DEGREES);
//                }
                        duckWheel.setVelocity(180, AngleUnit.DEGREES);
                    } else if (gamepad2.b) {
                        duckWheel.setVelocity(-180, AngleUnit.DEGREES);
                    } else {
                        duckWheel.setPower(0);
                    }

                    if (!limit.getState()) {
                        bottomLimit = arm.getCurrentPosition();
                    }

                    if (gamepad2.dpad_up) {
                        if (toggle2) {
                            level++;
                            toggle2 = false;
                        }
                    } else if (gamepad2.dpad_down) {
                        if (toggle2) {
                            level--;
                            toggle2 = false;
                        }
                    } else {
                        toggle2 = true;
                    }

                    if (level > 3) {
                        level = 0;
                    } else if (level < 0) {
                        level = 3;
                    }

                    if(level == 0) {
                        levelHeight = SHARED;
                        currentTarget = "SHARED SHIPPING HUB";
                    }else if(level == 1){
                        levelHeight = BOTTOM;
                        currentTarget = "BOTTOM LEVEL";
                    }else if(level == 2){
                        levelHeight = MIDDLE;
                        currentTarget = "MIDDLE LEVEL";
                    }else if(level == 3){
                        levelHeight = HIGH;
                        currentTarget = "TOP LEVEL";
                    }

                    if(gamepad2.right_bumper){
                        if(toggle3){
                            SHARED += 10;
                            toggle3 = false;
                        }
                    }else if(gamepad2.left_bumper){
                        if(toggle3){
                            SHARED -= 10;
                            toggle3 = false;
                        }
                    }else if(gamepad2.right_trigger > 0){
                        if(toggle3){
                            SHARED += 20;
                            toggle3 = false;
                        }
                    }else if(gamepad2.left_trigger > 0){
                        if(toggle3){
                            SHARED -= 20;
                            toggle3 = false;
                        }
                    }else{
                        toggle3 = true;
                    }

//            telemetry.addData("target pos", targetPos);
                    telemetry.addData("current pos", -(arm.getCurrentPosition() - bottomLimit));
                    telemetry.addData("arm pos", armSupport.getCurrentPosition());
//            telemetry.addData("left stick y", gamepad1.left_stick_y);
//            telemetry.addData("motor power", intake.getPower());
//            telemetry.addData("duck wheel", duckWheel.getPower());
//            telemetry.addData("touch sensor", limit.getState());
//            telemetry.addData("arm power", arm.getPower());
//            telemetry.addData("armSupport power", arm.getPower());
//            telemetry.addData("armState", armState);
                    telemetry.addData("SHARED HEIGHT", SHARED);
                    telemetry.addData("gate", gate.getPosition());
                    telemetry.addData("level", level);
                    telemetry.addData("toggle2", toggle2);
                    telemetry.addData("target level", currentTarget);
                    telemetry.addData("time", (int)timer.seconds());
                    telemetry.addData("gyro reading", imu.getAngularOrientation().thirdAngle);
                    telemetry.update();
                }


            }
            canRunGamepadThread = false;
            socket.close();
            tapeMeasureThread.interrupt();
        }
    }
}