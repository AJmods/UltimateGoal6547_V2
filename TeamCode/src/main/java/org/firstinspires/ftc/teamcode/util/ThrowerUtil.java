package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.RobotLog;

@Config
public class ThrowerUtil {

    /**
     * Multiply the theoretical speed from getVi() by this number to make the launcher hit the powershots
     */
    public static double POWER_SHOT_CONSTANT = 2.1715;
    public static double GOAL_CONSTANT = 2.36;


    public static double MIN_Y = -29;
    public static double MAX_Y = -43;

    /**
     * Angle of the thrower (in degrees)
     */
    public static double INITIAL_ANGLE = 29;

    /**
     * Height of where the ring leaves the robot thrower when launched (in inches)
     */
    public static double INITIAL_HEIGHT = 13.75;
    /**
     * Diameter of the wheels on the thrower (in inches)
     */
    public static final double wheelDiameter = 3.77953; //96 mm
    /**
     * Inches per revolution of the wheels on the thrower
     */
    public static final double inchesPerRev = Math.PI * wheelDiameter;


    /**
     * @param x1 where the robot starts (typically 0)
     * @param y1 initial height of the ring when it is launched
     * @param x2 distance from ring thrower to goal
     * @param y2 height of goal
     * @param angle angle of the ring thrower
     * @return theoretical velocity that an object needs to achieve to
     * reach the requirements set in the parameters.
     */
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

    /**
     * @param currentPos starting location (x1, y1, and heading)
     * @param targetX an x value (x2)
     * @return Assuming connecting points (x1, y1) and (x2, y2) make a line and the heading is known, return y2
     */
    public static double getTargetY(Pose2d currentPos, double targetX) {
        double slope = Math.tan(currentPos.getHeading());
        //RobotLog.v("SLOPE: " + slope);
        //y = mx + b
        //b = y - mx
        double yIntercept = currentPos.getY() - (currentPos.getX() * slope);

        RobotLog.v("SLOPE: (CURRENT POSITION): " + currentPos);
        RobotLog.v("SLOPE: " + slope + ", Y-INTERCEPT: " + yIntercept);

        return (slope * targetX) + yIntercept;
    }
    /**
     * @param currentPos starting location (x1, y1, and heading)
     * @param targetY an x value (y2)
     * @return Assuming connecting points (x1, y1) and (x2, y2) make a line and the heading is known, return x2
     */
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


    /**
     * @param x robot X position
     * @param y robot Y position
     * @param angle robot heading (must be in radians)
     * @return if the robot is facing the goal.
     */
    public static boolean isValidAngle(double x, double y, double angle) {
        double minAngle = Math.atan2(MIN_Y - y, FieldConstants.RED_GOAL_X - x);
        double maxAngle = Math.atan2(MAX_Y - y, FieldConstants.RED_GOAL_X - x);

        return (maxAngle > minAngle && minAngle < angle && maxAngle > angle) || (minAngle > maxAngle && maxAngle < angle && minAngle > angle);
    }
    public static boolean isValidAngle(Pose2d pose2d) {return isValidAngle(pose2d.getX(), pose2d.getY(), pose2d.getHeading());}
}
