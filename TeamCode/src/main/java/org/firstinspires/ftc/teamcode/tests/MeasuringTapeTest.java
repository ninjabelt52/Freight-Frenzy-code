package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.classes.TapeMeasure;


@TeleOp
public class MeasuringTapeTest extends LinearOpMode {
    public void runOpMode(){
        TapeMeasure tape = new TapeMeasure(hardwareMap, gamepad1, gamepad2);
        boolean toggle = false;

        Thread tapeThread = new Thread(tape);

        telemetry.addData("Tape Telemetry", tape);
        telemetry.update();

        waitForStart();
        tapeThread.start();
        while (opModeIsActive()){
            telemetry.addData("Tape Telemetry", tape);
            telemetry.update();
        }
        tapeThread.interrupt();
    }
}
