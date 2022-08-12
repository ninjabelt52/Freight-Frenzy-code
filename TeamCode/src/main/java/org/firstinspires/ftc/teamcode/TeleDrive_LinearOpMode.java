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

        static final double INCREMENT   = 0.05;
        static final double MAX_POS     =  1.0;
        static final double MIN_POS     =  0.0;
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

        Servo   servo;
        servo = hardwareMap.get(Servo.class, "horizontal");
        Servo   servo2;
        servo2 = hardwareMap.get(Servo.class, "vertical");
        double  position = (MAX_POS - MIN_POS) / 2;
        double  position2 = (MAX_POS - MIN_POS) / 2;
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


            while (opModeIsActive()) {

                if(gamepad1.dpad_left){
                    position = position-INCREMENT;
                }
                if(position<MIN_POS){
                    position = MIN_POS;
                }
                if(gamepad1.dpad_right){
                    position = position+INCREMENT;
                }
                if(position>MAX_POS){
                    position = MAX_POS;
                }

                if(gamepad1.dpad_down){
                    position2 = position2-INCREMENT;
                }
                if(position2<MIN_POS){
                    position2 = MIN_POS;
                }
                if(gamepad1.dpad_up){
                    position2 = position2+INCREMENT;
                }
                if(position2>MAX_POS){
                    position2 = MAX_POS;
                }

                servo.setPosition(position);
                servo2.setPosition(position2);
                telemetry.addData("Horizontal Position", "%5.2f", position);
                telemetry.addData("Vertical Position", "%5.2f", position2);
                telemetry.update();
                }


            }
            canRunGamepadThread = false;
            socket.close();
    }
}