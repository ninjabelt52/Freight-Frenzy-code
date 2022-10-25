/* Copyright (c) 2020 FIRST. All rights reserved.
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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.classes.AutoAlignPipeline;

import java.util.List;

@TeleOp(name = "Concept: TensorFlow Object Detection Switchable Cameras", group = "Concept")
//@Disabled
public class FirstBlueParkAutoWithPipeline extends LinearOpMode {

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    Pose2d startPose = new Pose2d(36, 60, Math.toRadians(-90));

    @Override
    public void runOpMode() {

        AutoAlignPipeline.AutoDoubleCameras pipeline = new AutoAlignPipeline.AutoDoubleCameras();
        waitForStart();
        pipeline.initVuforAndTF();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (pipeline.tfod != null) {
                    pipeline.doCameraSwitching();
                    List<Recognition> recognitions = pipeline.tfod.getRecognitions();
                    telemetry.addData("# Objects Detected", recognitions.size());
                    // step through the list of recognitions and display image size and position
                    // Note: "Image number" refers to the randomized image orientation/number
                    for (Recognition recognition : recognitions) {
                        double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                        double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                        double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                        double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                        if(recognition.getConfidence() >= 0.9f) {
                            objectDetected(recognition.getLabel(), recognition.getConfidence());
                        }
                    }
                    telemetry.update();
                }
            }
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */



    public void objectDetected(String detectedLabel, float confidence){

        Trajectory Park1 = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(60, 36, Math.toRadians(180)))
                .build();
        Trajectory Park2 = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(36, 36, Math.toRadians(180)))
                .build();
        Trajectory Park3 = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(12, 60, Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(12, 36, Math.toRadians(180)))
                .build();


        if(detectedLabel == "Bolt"){
            telemetry.addData("We are going for BOLT, detected", "%s (%.0f %% Conf.)", detectedLabel, confidence * 100 );
            drive.followTrajectory(Park1);
        }else if(detectedLabel == "Bulb"){
            telemetry.addData("We are going for BULB, detected", "%s (%.0f %% Conf.)", detectedLabel, confidence * 100 );
            drive.followTrajectory(Park2);
        }else if(detectedLabel == "Panel"){
            telemetry.addData("We are going for PANEL, detected", "%s (%.0f %% Conf.)", detectedLabel, confidence * 100 );
            drive.followTrajectory(Park3);
        }

    }

}
