package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;

@TeleOp
@Disabled
public class FileTest extends LinearOpMode {
    public void runOpMode(){
        FtcDashboard dash = FtcDashboard.getInstance();
        Telemetry telemetry = dash.getTelemetry();
        File test = new File("1234");

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("File", test);
            telemetry.addData("toString", test.toString());
            telemetry.update();
        }
    }
}
