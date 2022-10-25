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

import java.util.List;

@TeleOp(name = "Concept: TensorFlow Object Detection Switchable Cameras", group = "Concept")
//@Disabled
public class FirstRedParkAuto extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    private static final String VUFORIA_KEY =
            "ATIZ5+n/////AAABmYR5zYS7b0fUh3KW5QXpeYpWedz730exbiG/c1eUwnbTLDy3S1zpCnndqs2H1bBRoRji6zODId1wNciirjMhrl3kb2xpxrhjWKGzfymlE48qjK1tSK9ctbVPDDEI7d3wiO1vISMt8XRVM0HJFumWl7snC0XtMEyTqy7IxrPv858Bm2voggonrgAx+WT/LcXxfsUQjd7SpX1AhzOEzksBRKfws0MP5hw7hLLbB+4QSb03hTA90Mh/7F78NxFG8yYmPrM0GMVg+3TRjteX6TWOHidLSmrIzsZNGFeqXCHOx4QIWIiMOIenqgau/XX2zCqvrMGCpOTb77oTjkvOlLNh9WizYdFPQjT/u7CeU1p3Y1ox";

    private VuforiaLocalizer vuforia;

    private WebcamName webcam1, webcam2;
    private SwitchableCamera switchableCamera;
    private boolean oldLeftBumper;
    private boolean oldRightBumper;


    private TFObjectDetector tfod;

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    Pose2d startPose = new Pose2d(36, -60, Math.toRadians(90));

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0/9.0);
        }

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    doCameraSwitching();
                    List<Recognition> recognitions = tfod.getRecognitions();
                    telemetry.addData("# Objects Detected", recognitions.size());
                    // step through the list of recognitions and display image size and position
                    // Note: "Image number" refers to the randomized image orientation/number
                    for (Recognition recognition : recognitions) {
                        double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                        double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                        double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                        double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                        telemetry.addData(""," ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                        telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                        telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
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
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // Indicate that we wish to be able to switch cameras.
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        parameters.cameraName = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Set the active camera to Webcam 1.
        switchableCamera = (SwitchableCamera) vuforia.getCamera();
        switchableCamera.setActiveCamera(webcam1);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

    private void doCameraSwitching() {
        // If the left bumper is pressed, use Webcam 1.
        // If the right bumper is pressed, use Webcam 2.
        boolean newLeftBumper = gamepad1.left_bumper;
        boolean newRightBumper = gamepad1.right_bumper;
        if (newLeftBumper && !oldLeftBumper) {
            switchableCamera.setActiveCamera(webcam1);
        } else if (newRightBumper && !oldRightBumper) {
            switchableCamera.setActiveCamera(webcam2);
        }
        oldLeftBumper = newLeftBumper;
        oldRightBumper = newRightBumper;

        if (switchableCamera.getActiveCamera().equals(webcam1)) {
            telemetry.addData("activeCamera", "Webcam 1");
            telemetry.addData("Press RightBumper", "to switch to Webcam 2");
        } else {
            telemetry.addData("activeCamera", "Webcam 2");
            telemetry.addData("Press LeftBumper", "to switch to Webcam 1");
        }
    }

    public void objectDetected(String detectedLabel, float confidence){

        Trajectory Park1 = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(60, -36, Math.toRadians(180)))
                .build();
        Trajectory Park2 = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(36, -36, Math.toRadians(180)))
                .build();
        Trajectory Park3 = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(12, -60, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(12, -36, Math.toRadians(180)))
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
