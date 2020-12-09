package org.firstinspires.ftc.teamcode.util.pipeline;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.RobotLog;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class openCvPipeLines {

    public static double lowerHue = 95;
    public static double lowerSat = 100;
    public static double lowerVal = 100;
    public static double upperHue = 110;
    public static double upperSat = 255;
    public static double upperVal = 255;

    public static double ringWidth = 50;
    public static double oneRingHeight = 15;
    public static double fourRingHeight = 35;

//    public static int rectX=75;
//    public static int rectY=50;
//    public static int rectW=20;
//    public static int rectH=25;

//    public static double oneRingPercent = .50;
//    public static double fourRingPercent = .60;


    //number of rings the camera saw for auton
    public enum RingCount {
        NONE, ONE, FOUR
    }


    private final Scalar green = new Scalar(0,255,0);
//    public static class RingDetectionPipelineV2 extends OpenCvPipeline {
//        enum Stage
//        {
//            RAW_IMAGE,
//            CROPPED,
//            MASK,
//        }
//
//        Stage stage = Stage.RAW_IMAGE;
//        Stage[] stages = Stage.values();
//
//        RingCount ringCount = RingCount.NONE; //defalt to no Ring;
//
//        Mat hsv = new Mat();
//
//        Mat mask2 = new Mat();
//
//        Mat mask = new Mat();
//
//        Mat cropped = new Mat();
//
//        List<MatOfPoint> contoursList = new ArrayList<>();
//
//        @Override
//        public Mat processFrame(Mat input) {
//
//            contoursList.clear();
//            Scalar lowerOrange = new Scalar(lowerHue,lowerSat,lowerVal);
//            Scalar upperOrange = new Scalar(upperHue,upperSat,upperVal);
//
//            //mask out rectangle
//            mask = new Mat(input.rows(), input.cols(), CvType.CV_8U, Scalar.all(0));
//
//            Rect rect = new Rect(rectX,rectY,rectW,rectH);
//            Imgproc.rectangle(mask, rect, new Scalar(255,255,255),-1);
//
//            input.copyTo(cropped, mask);
//
//            //mask out the ring color
//
//            Imgproc.cvtColor(cropped, hsv, Imgproc.COLOR_BGR2HSV);
//
//            Core.inRange(hsv, lowerOrange, upperOrange, mask2);
//
//
//            //calculate percent of mask.
//            int count = Core.countNonZero(mask2);
//            double percent = (double) count / (rect.height*rect.width);
//
//            if (percent > fourRingPercent) ringCount = RingCount.FOUR;
//            else if (percent > oneRingPercent) ringCount = RingCount.ONE;
//            else ringCount = RingCount.NONE;
//
//            RobotLog.v("Count: " + count + ", Percent: " + percent + "%, RingCount:" + ringCount);
//
//            switch (stage) {
//                case RAW_IMAGE:
//                    Imgproc.rectangle(input, rect, new Scalar(255,255,255),2); //for debugging
//                    return input;
//                case MASK:
//                    return mask2;
//                case CROPPED:
//                    return cropped;
//                default:
//                    return input;
//            }
//
//
//        }
//
//        public RingCount getRingCount() {
//            return ringCount;
//        }
//
//        @Override
//        public void onViewportTapped() {
//            /*
//             * Note that this method is invoked from the UI thread
//             * so whatever we do here, we must do quickly.
//             */
//
//            int currentStageNum = stage.ordinal();
//
//            int nextStageNum = currentStageNum + 1;
//
//            if(nextStageNum >= stages.length)
//            {
//                nextStageNum = 0;
//            }
//
//            stage = stages[nextStageNum];
//        }
//    }

    public static class RingDetectionPipeLine extends OpenCvPipeline {

        enum Stage
        {//color difference. greyscale
            MASK, //includes outlines
            RES,
            RAW_IMAGE,//displays raw view
        }

        Mat hsv = new Mat();
        Mat mask = new Mat();
        Rect ringRect;

        List<MatOfPoint> contoursList = new ArrayList<>();

        private static RingCount ringCount = RingCount.NONE; //defalt to no Ring;

        Stage stage = Stage.MASK;
        private Stage[] stages = Stage.values();


        @Override
        public Mat processFrame(Mat input) {

            //TODO: MOVE lowerOrange and UpperOrange to be global variables
            Scalar lowerOrange = new Scalar(lowerHue,lowerSat,lowerVal);
            Scalar upperOrange = new Scalar(upperHue,upperSat,upperVal);

            contoursList.clear();

            //isolate ring color
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
            Core.inRange(hsv, lowerOrange, upperOrange, mask);

            //Get blobs of ring color
            Imgproc.findContours(mask, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            //no blobs found
            if (contoursList.size() == 0) {
                ringCount = RingCount.NONE;
                Imgproc.putText(input, "NO RING", new Point(100,100), Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(0,255,0), 2);
                //return mask;
            } else {
                //get the biggest blob of ring color
                int maxValIndex = 0;
                double maxVal = 0;
                double contourArea=0;
                for (int i = 0; i < contoursList.size(); i++) {
                    contourArea = Imgproc.contourArea(contoursList.get(i));
                    if (maxVal < contourArea) {
                        maxVal = contourArea;
                        maxValIndex = i;
                    }
                }

                //get rectangle around big blob
                ringRect = Imgproc.boundingRect(contoursList.get(maxValIndex));

                //draw rectangle
                Imgproc.rectangle(input, ringRect, new Scalar(0, 255, 0));

                if (ringRect.height > fourRingHeight) ringCount = RingCount.FOUR;
                else if (ringRect.height > oneRingHeight) ringCount = RingCount.ONE;
                else ringCount = RingCount.NONE;

                Imgproc.putText(input, ringCount + " RINGS", new Point(ringRect.x,ringRect.y), Imgproc.FONT_HERSHEY_COMPLEX, 2, new Scalar(0,255,0), 2);

                RobotLog.v("RECT H:" + ringRect.height + ", RECT W: " + ringRect.width + ", RingCount: " + ringCount);
//                double zeroCount = Core.countNonZero(mask);
//
//                double percent = zeroCount / (ringRect.height * ringRect.width);

//                if (percent > fourRingPercent) ringCount = RingCount.FOUR;
//                else if (percent > oneRingPercent) ringCount = RingCount.ONE;
//                else ringCount = RingCount.NONE;

               // RobotLog.v("Non Zero: " + Core.countNonZero(mask) + ", Area: " + contourArea + ", Percent: " + percent + "%, RingCount:" + ringCount);
             }

            switch (stage) {
                case RES:
                    Core.bitwise_and(input, input, mask);
                    return input;
                case RAW_IMAGE:
                    return input;
                case MASK:
                    return mask;
            }
            return input;

        }

        public void switchStage() {
            int currentStageNum = stage.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stage = stages[nextStageNum];
        }
        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stage.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stage = stages[nextStageNum];
        }


        public static RingCount getRingCount() { return ringCount;}
    }
}
