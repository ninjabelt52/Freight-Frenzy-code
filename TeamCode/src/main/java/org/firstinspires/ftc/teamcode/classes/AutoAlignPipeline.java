
package org.firstinspires.ftc.teamcode.classes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

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

    class Threshold extends OpenCvPipeline {

        int stage = 0;
        Mat ycrcb = new Mat();
        Mat coi = new Mat();
        Mat threshold = new Mat();


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
            Imgproc.cvtColor(input, ycrcb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(ycrcb, coi, 0);

            Imgproc.threshold(coi, threshold, threshVal, 255, Imgproc.THRESH_BINARY);


            Imgproc.rectangle(threshold, midBox,new Scalar(255,0,0), 2);

            Mat section2 = threshold.submat(midBox);

            avg2 = Core.mean(section2).val[0];

            switch (stage){
                case 0:
                    telemetry = "active stage is ycrcb" + "\navg 1: " + avg1 + "\navg 2: " + avg2;
                    Imgproc.rectangle(ycrcb, midBox,new Scalar(255,0,0), 2);
                    return ycrcb;
                case 1:
                    telemetry = "active stage is coi" + "\navg 1: " + avg1 + "\navg 2: " + avg2;
                    Imgproc.rectangle(coi, midBox,new Scalar(255,0,0), 2);
                    return coi;
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
