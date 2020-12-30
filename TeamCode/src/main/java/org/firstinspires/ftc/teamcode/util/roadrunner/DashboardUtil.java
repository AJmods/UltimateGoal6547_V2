package org.firstinspires.ftc.teamcode.util.roadrunner;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.FieldConstants;

import java.util.ArrayList;
import java.util.List;

/**
 * Set of helper functions for drawing Road Runner paths and trajectories on dashboard canvases.
 */
public class DashboardUtil {

    private static final String TAG = "DashboardUtil.java";
    private static final double DEFAULT_RESOLUTION = 2.0; // distance units; presumed inches
    private static final double ROBOT_RADIUS = 9; // in

    private static double[][] vuforiaCirclePointsX;
    private static double[][] vuforiaCirclePointsY;

    public static void drawLine(Canvas canvas, Pose2d pos) {
        drawLine(canvas, pos, pos.getHeading());
    }
    public static void drawLine(Canvas canvas, Pose2d pos, double angle) {
        double slope = Math.tan(angle);
        //y = mx + b
        //b = y - mx
        double yIntercept = pos.getY() - (pos.getX() * slope);

        double endX;
        if (angle > 0) {
            endX = 72;
        } else if (angle < 0) {
            endX = -72;
        } else {
            RobotLog.ee(TAG, "Unable to Stroke Line on Dashboard");
            return;
        }

        double endY1 = (endX * slope) + yIntercept;
        double endY2 = (pos.getX() * slope) + yIntercept;

        canvas.strokeLine(endX, endY1, pos.getX(), endY2);

        //double bigNum = (144 * pos.)
    }


    public static void drawPoseHistory(Canvas canvas, List<Pose2d> poseHistory) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];
        for (int i = 0; i < poseHistory.size(); i++) {
            Pose2d pose = poseHistory.get(i);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledPath(Canvas canvas, Path path, double resolution) {
        int samples = (int) Math.ceil(path.length() / resolution);
        double[] xPoints = new double[samples];
        double[] yPoints = new double[samples];
        double dx = path.length() / (samples - 1);
        for (int i = 0; i < samples; i++) {
            double displacement = i * dx;
            Pose2d pose = path.get(displacement);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledPath(Canvas canvas, Path path) {
        drawSampledPath(canvas, path, DEFAULT_RESOLUTION);
    }

    public static void drawRobot(Canvas canvas, Pose2d pose) {
        canvas.strokeCircle(pose.getX(), pose.getY(), ROBOT_RADIUS);
        Vector2d v = pose.headingVec().times(ROBOT_RADIUS);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
        canvas.fillCircle(pose.getX(),pose.getY(),1);
    }

    //so apparently there's this method called setStroke().......
    //GOD FUCKING DAMMIT I SPENT HOURS ON A METHOD THAT IS ALREADY BUILT IN
//    public static void generateVuforiaPoints(double radius) {generateVuforiaPoints(radius, DEFAULT_RESOLUTION);}
//    public static void generateVuforiaPoints(double radius, double res) {
//        int circles = FieldConstants.VuforiaTargetsX.length;
//        List<Double>[] vuforiaCirclePointsXTemp = new ArrayList[circles];
//        List<Double>[] vuforiaCirclePointsYTemp = new ArrayList[circles];
//
//        for (int i = 0; i < circles; i++) {
//            //filter out x values outside of the field and skip x values outside of the circle's domain
//            double startPoint = Math.max(FieldConstants.BOTTOM_OF_FIELD, FieldConstants.VuforiaTargetsX[i] - radius);
//            double endPoint = Math.min(FieldConstants.TOP_OF_FIELD, FieldConstants.VuforiaTargetsX[i] + radius);
//
//            //get the center of circle
//            double centerX = FieldConstants.VuforiaTargetsX[i];
//            double centerY = FieldConstants.VuforiaTargetsY[i];
//
//            //get the points to make the top half of the circle
//            for (double x = startPoint; x <= endPoint; x+=res) {
//                //y = (r^2 - (x-h)^2)^.5 + k
//                double endY = Math.sqrt(Math.pow(radius,2) - Math.pow(x-centerX,2)) + centerY;
//
//                //filter out y values outside of the field
//                if (Math.abs(endY) < FieldConstants.TOP_OF_FIELD) {
//                    vuforiaCirclePointsXTemp[i].add(x);
//                    vuforiaCirclePointsYTemp[i].add(endY);
//                }
//            }
//            //get the points to make the top half of the circle if needed
//            if (startPoint != FieldConstants.BOTTOM_OF_FIELD && endPoint != FieldConstants.TOP_OF_FIELD) {
//                //start where the circle left off
//                boolean countUp = Math.abs(endPoint) > Math.abs(startPoint);
//                double startPoint2 = countUp ? startPoint : endPoint;
//                double endPoint2 = countUp ? endPoint : startPoint;
//                double resModifier = countUp ? 1 : -1;
//                for (double x = startPoint2; Math.abs(x) <= Math.abs(endPoint2); x+=res * resModifier) {
//
//                    double endY = -Math.sqrt(Math.pow(radius,2) - Math.pow(x-centerX,2)) + centerY;
//
//                    //filter out y values outside of the field
//                    if (Math.abs(endY) < FieldConstants.TOP_OF_FIELD) {
//                        vuforiaCirclePointsXTemp[i].add(x);
//                        vuforiaCirclePointsYTemp[i].add(endY);
//                    }
//                }
//            }
//            vuforiaCirclePointsX[i] = vuforiaCirclePointsXTemp[i].toArray(new double[vuforiaCirclePointsXTemp[i].size()]);
//        }
//    }
//    public static void drawVuforiaCircles(Canvas fieldOverlay) {
//        int circles = vuforiaCirclePointsX.length;
//        for (int i = 0; i < circles; i++) {
//            fieldOverlay.fillPolygon(vuforiaCirclePointsX[i], vuforiaCirclePointsY[i]);
//        }
//    }
}