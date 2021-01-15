package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

/**
 * Field and objects places (in inches)
 */
@Config
public class FieldConstants {

    public static double TOP_OF_FIELD = 72;
    public static double BOTTOM_OF_FIELD = -72;
    public static double RIGHT_OF_FIELD = -72;
    public static double LEFT_OF_FIELD = 72;

    public static double RED_GOAL_Y = -36;

    public static double RED_GOAL_X = TOP_OF_FIELD;
    public static double RED_GOAL_HEIGHT = 36;
    public static double POWER_SHOT_HEIGHT = 30;

    public static double RED_POWER_SHOT_1Y = 1;
    public static double RED_POWER_SHOT_1X = 75;
    public static double RED_POWER_SHOT_2Y = -6;
    public static double RED_POWER_SHOT_2X = 75;
    public static double RED_POWER_SHOT_3Y = -16.5;
    public static double RED_POWER_SHOT_3X = 75;

    //Red Tower, Red Side Wall, Audience wall, Blue Side Wall, Blue Tower
    public static double[] VuforiaTargetsX = new double[] {72,    1.5, -72, 1.5, 72};
    public static double[] VuforiaTargetsY = new double[] {-36.5, -72, 1.5, 72,  36.5};

//    public static double TOP_OF_FIELD = 72;
//
//    public static double RED_GOAL_Y = -36;
//    public static double RED_GOAL_X = TOP_OF_FIELD;
//    public static double RED_GOAL_HEIGHT = 36;
//    public static double POWER_SHOT_HEIGHT = 30;
//
//    public static double RED_POWER_SHOT_1Y = -2.5;
//    public static double RED_POWER_SHOT_1X = 73;
//    public static double RED_POWER_SHOT_2Y = -9.5;
//    public static double RED_POWER_SHOT_2X = 73;
//    public static double RED_POWER_SHOT_3Y = -14;
//    public static double RED_POWER_SHOT_3X = 73;

}
