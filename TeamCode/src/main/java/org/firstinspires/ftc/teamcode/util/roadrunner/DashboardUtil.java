package org.firstinspires.ftc.teamcode.util.roadrunner;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.List;

/**
 * Set of helper functions for drawing Road Runner paths and trajectories on dashboard canvases.
 */
public class DashboardUtil {

    private static final String TAG = "DashboardUtil.java";
    private static final double DEFAULT_RESOLUTION = 2.0; // distance units; presumed inches
    private static final double ROBOT_RADIUS = 9; // in

    public static void drawLine(Canvas canvas, Pose2d pos) {
        drawLine(canvas, pos, pos.getHeading());
    }
    public static void drawLine(Canvas canvas, Pose2d pos, double angle) {
        double slope = Math.tan(angle);
        //y = mx + b
        //b = y - mx
        double yInterecpt = pos.getY() - (pos.getX() * slope);

        double endX;
        if (angle > 0) {
            endX = 72;
        } else if (angle < 0) {
            endX = -72;
        } else {
            RobotLog.ee(TAG, "Unable to Stroke Line on Dashboard");
            return;
        }

        double endY1 = (endX * slope) + yInterecpt;
        double endY2 = (pos.getX() * slope) + yInterecpt;

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
}