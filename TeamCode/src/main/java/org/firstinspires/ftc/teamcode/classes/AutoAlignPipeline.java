
package org.firstinspires.ftc.teamcode.classes;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.ArrayList;
import java.util.List;

@Config
public class AutoAlignPipeline {
    OpenCvCamera webcam;
    Threshold thresh;
    String telemetry = "waiting for input";
    public static Rect midBox = new Rect(185,115,25,25);
    public static int threshVal = 128;



    public AutoAlignPipeline(HardwareMap hardwareMap, String camName){


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,  camName), cameraMonitorViewId);

        thresh = new Threshold();
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.setPipeline(thresh);
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

        }
        });

            telemetry = "waiting for start";
    }

    public static class AutoDoubleCameras{
        HardwareMap hardwareMap = null;
        public static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";

        private final String[] LABELS = {
                "1 Bolt",
                "2 Bulb",
                "3 Panel"
        };

        private static final String VUFORIA_KEY =
                "ATIZ5+n/////AAABmYR5zYS7b0fUh3KW5QXpeYpWedz730exbiG/c1eUwnbTLDy3S1zpCnndqs2H1bBRoRji6zODId1wNciirjMhrl3kb2xpxrhjWKGzfymlE48qjK1tSK9ctbVPDDEI7d3wiO1vISMt8XRVM0HJFumWl7snC0XtMEyTqy7IxrPv858Bm2voggonrgAx+WT/LcXxfsUQjd7SpX1AhzOEzksBRKfws0MP5hw7hLLbB+4QSb03hTA90Mh/7F78NxFG8yYmPrM0GMVg+3TRjteX6TWOHidLSmrIzsZNGFeqXCHOx4QIWIiMOIenqgau/XX2zCqvrMGCpOTb77oTjkvOlLNh9WizYdFPQjT/u7CeU1p3Y1ox";

        private VuforiaLocalizer vuforia;

        public WebcamName webcam1, webcam2;
        public SwitchableCamera switchableCamera;
        public TFObjectDetector tfod;

        public void initVuforAndTF(){
            initVuforia();
            initTfod();

            if (tfod != null) {
                tfod.activate();
                tfod.setZoom(1.0, 16.0/9.0);
            }
        }


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

        public void doCameraSwitching() {
            // If the left bumper is pressed, use Webcam 1.
            // If the right bumper is pressed, use Webcam 2.
            if (true) {
                switchableCamera.setActiveCamera(webcam1);
            } else if (false) {
                switchableCamera.setActiveCamera(webcam2);
            }
        }
    }



    class Threshold extends OpenCvPipeline {

        int stage = 0;
        Mat greyscale = new Mat();
        Mat threshold = new Mat();
        Mat hierarchy = new Mat();


        double avg1, avg2;

        @Override
        public void onViewportTapped() {
            stage ++;

            if(stage > 3){
                stage = 0;
            }
        }

        @Override
        public Mat processFrame(Mat input){
            List <MatOfPoint> contours = new ArrayList<MatOfPoint>();

            Imgproc.cvtColor(input, greyscale, Imgproc.COLOR_BGR2GRAY);

            Imgproc.threshold(greyscale, threshold, threshVal, 255, Imgproc.THRESH_BINARY);


            Imgproc.rectangle(threshold, midBox,new Scalar(255,0,0), 2);

            Imgproc.findContours(threshold, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
            Imgproc.drawContours(input, contours, -1, new Scalar(0,255,0), 1);

            switch (stage){
                case 0:
                    telemetry = "active stage is ycrcb" + "\navg 1: " + avg1 + "\navg 2: " + avg2;
                    Imgproc.rectangle(greyscale, midBox,new Scalar(255,0,0), 2);
                    return greyscale;
                case 1:
                    telemetry = "active stage is coi" + "\navg 1: " + avg1 + "\navg 2: " + avg2;
                    Imgproc.rectangle(greyscale, midBox,new Scalar(255,0,0), 2);
                    return greyscale;
                case 2:
                    telemetry = "active stage is threshold" + "\navg 1: " + avg1 + "\navg 2: " + avg2;
                    Imgproc.rectangle(threshold, midBox, new Scalar(255,0,0), 2);
                    return threshold;
                default:
                    telemetry = "active stage is input" + "\navg 1: " + avg1 + "\navg 2: " + avg2;
                    Imgproc.rectangle(input, midBox,new Scalar(255,0,0), 2);
                    return  input;
            }
        }
    }

    public String toString(){
        return telemetry;
    }
}
