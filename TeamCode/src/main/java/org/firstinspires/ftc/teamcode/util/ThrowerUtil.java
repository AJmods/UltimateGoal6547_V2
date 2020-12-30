package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@Config
public class ThrowerUtil {
    public static double POWER_SHOT_CONSTANT = 2.173;
    public static double GOAL_CONSTANT = 2.36;

    public static double MIN_Y = -29;
    public static double MAX_Y = -43;
    public static double INITAL_ANGLE = 29;
    public static double INITAL_HEIGHT = 13.75;
    public static final double wheelDiameter = 3.77953; //96 mm
    public static final double inchesPerRev = Math.PI * wheelDiameter;

    private final double RED_GOAL_X = FieldConstants.RED_GOAL_X;

    public static double getVi(double x1, double y1, double x2, double y2, double angle) {

        double gravity = -386.09;
        int numDigits = 10;
        double startNumber = 1000;

        double deltaX = Math.abs(x1 - x2);
        double deltaY = Math.abs(y2 - y1);
        angle = Math.toRadians(angle);

        //equation to get target Initial V
        return deltaX / (Math.cos(angle) * Math.sqrt((deltaY - (deltaX * Math.tan(angle))) / (gravity / 2)));
    }

    public static double getTargetY(Pose2d currentPos, double targetX) {
        double slope = Math.tan(currentPos.getHeading());
        //RobotLog.v("SLOPE: " + slope);
        //y = mx + b
        //b = y - mx
        double yIntercept = currentPos.getY() - (currentPos.getX() * slope);

        return (slope * targetX) + yIntercept;
    }
    public static double getTargetX(Pose2d currentPos, double targetY) {
        double slope = Math.tan(currentPos.getHeading());

        double yIntercept = currentPos.getY() - (currentPos.getX() * slope);
        return (targetY - yIntercept) / slope;
    }
//    public static Pose2d getEndPoint(Vector2d pos, double heading) { return getEndPoint(new Pose2d(pos.getX(), pos.getY(), heading));}
//    public static Pose2d getEndPoint(Pose2d currentPos) {
//
//        double slope = Math.tan(currentPos.getHeading());
//        double yIntercept = currentPos.getY() - (currentPos.getX() * slope);
//        //Equation of Line is: y = m(x - x1) + y1, or y = slope*
//        double lineEquation = slope*(3 - currentPos.getX()) + currentPos.getY();
//    }

    public static boolean isValidAngle(double x, double y, double angle) {
        double minAngle = Math.atan2(MIN_Y - y, FieldConstants.RED_GOAL_X - x);
        double maxAngle = Math.atan2(MAX_Y - y, FieldConstants.RED_GOAL_X - x);

        return (maxAngle > minAngle && minAngle < angle && maxAngle > angle) || (minAngle > maxAngle && maxAngle < angle && minAngle > angle);
    }
}
