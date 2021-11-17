
package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class DuckDetectorPipeline {
    OpenCvCamera webcam;
    Threshold thresh;
    String telemetry = "waiting for input";
    DuckPos position;


    public DuckDetectorPipeline (HardwareMap hardwareMap, String camName){

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
        Mat coi2 = new Mat();
        Mat threshold = new Mat();


        double avg1, avg2, avg3;

        @Override
        public void onViewportTapped() {
            stage ++;

            if(stage > 3){
                stage = 0;
            }
        }

        @Override
        public Mat processFrame(Mat input){
//            telemetry = "pre-init";
//
//            try {
//                Thread.sleep(1000);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }

//            telemetry = "created submats";
//
//            try {
//                Thread.sleep(1000);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }

            Imgproc.cvtColor(input, ycrcb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(ycrcb, coi2, 1);

//            telemetry = "calculated channels";
//
//            try {
//                Thread.sleep(1000);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }

            Imgproc.threshold(coi2, threshold, 180, 100, Imgproc.THRESH_TOZERO);

//            telemetry = "thresholded";
//
//            try {
//                Thread.sleep(1000);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }


            Imgproc.rectangle(threshold, new Point(0,0),new Point(106,240),new Scalar(255,0,0), 2);
            Imgproc.rectangle(threshold, new Point(106,0),new Point(212,240),new Scalar(255,0,0), 2);
            Imgproc.rectangle(threshold, new Point(212,0),new Point(318,240),new Scalar(255,0,0), 2);

//            telemetry = "drew rectangles";
//
//            try {
//                Thread.sleep(1000);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }

            Mat section1 = threshold.submat(new Rect(new Point(0,0), new Point(106,240)));
            Mat section2 = threshold.submat(new Rect(new Point(106,0), new Point(212,240)));
            Mat section3 = threshold.submat(new Rect(new Point(212,0), new Point(318,240)));

//            telemetry = "created sections";
//
//            try {
//                Thread.sleep(1000);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }


            avg1 = Core.mean(section1).val[0];
            avg2 = Core.mean(section2).val[0];
            avg3 = Core.mean(section3).val[0];
//
//            telemetry = "calculated avgs";
//
//            try {
//                Thread.sleep(1000);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }


            if(avg1 > avg2){
                if(avg1 > avg3){
                    position = DuckPos.LEFT;
                    Imgproc.rectangle(threshold, new Point(0,0),new Point(106,240),new Scalar(255,0,0), -1);
                }else{
                    position = DuckPos.RIGHT;
                    Imgproc.rectangle(threshold, new Point(212,0),new Point(318,240),new Scalar(255,0,0), -1);
                }
            }else{
                if(avg2 > avg3){
                    position = DuckPos.CENTER;
                    Imgproc.rectangle(threshold, new Point(106,0),new Point(212,240),new Scalar(255,0,0), -1);
                }else{
                    position = DuckPos.RIGHT;
                    Imgproc.rectangle(threshold, new Point(212,0),new Point(318,240),new Scalar(255,0,0), -1);
                }
            }

            switch (stage){
                case 0:
                    telemetry = "active stage is ycrcb" + "\navg 1: " + avg1 + "\navg 2: " + avg2 + "\navg 3: " + avg3;
                    return ycrcb;
                case 1:
                    telemetry = "active stage is coi" + "\navg 1: " + avg1 + "\navg 2: " + avg2 + "\navg 3: " + avg3;
                    return coi2;
                case 2:
                    telemetry = "active stage is threshold" + "\navg 1: " + avg1 + "\navg 2: " + avg2 + "\navg 3: " + avg3;
                    return threshold;
                case 3:
                    telemetry = "active stage is input" + "\navg 1: " + avg1 + "\navg 2: " + avg2 + "\navg 3: " + avg3;
                    return input;
                default:
                    telemetry = "active stage is input" + "\navg 1: " + avg1 + "\navg 2: " + avg2 + "\navg 3: " + avg3;
                    return  input;
            }
        }
    }

    public enum DuckPos{
        LEFT,
        RIGHT,
        CENTER
    }

    public DuckPos getPosition(){
        return position;
    }

    public String toString(){
        return telemetry;
    }
}
