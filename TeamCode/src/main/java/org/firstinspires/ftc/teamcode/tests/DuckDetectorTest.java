package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.classes.DuckDetectorPipeline;


@TeleOp
public class DuckDetectorTest extends LinearOpMode {
    public void runOpMode(){
        DuckDetectorPipeline pipeline = new DuckDetectorPipeline(hardwareMap, "Webcam 1");

        while(!isStarted()){
            telemetry.addData("pipeline telemetry", pipeline);
            telemetry.update();
        }

        //I think something is wrong with the camera itself

        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("pipeline telemetry", pipeline);
            telemetry.addData("estimated guess", pipeline.getPosition());
            telemetry.update();
        }
    }
}
