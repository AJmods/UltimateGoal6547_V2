package org.firstinspires.ftc.teamcode.drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;

/**
 * Constants shared between the drive train
 * Should be set for this year's robot
 *
 */
/*
 * Constants shared between multiple drive types.
 *
 * TODO: Tune or adjust the following constants to fit your robot. Note that the non-final
 * fields may also be edited through the dashboard (connect to the robot's WiFi network and
 * navigate to https://192.168.49.1:8080/dash). Make sure to save the values here after you
 * adjust them in the dashboard; **config variable changes don't persist between app restarts**.
 *
 * These are not the only parameters; some are located in the localizer classes, drive base classes,
 * and op modes themselves.
 */
@Config
public class DriveConstants {

    /*
     * The type of motor used on the drivetrain. While the SDK has definitions for many common
     * motors, there may be slight gear ratio inaccuracies for planetary gearboxes and other
     * discrepancies. Additional motor types can be defined via an interface with the
     * @DeviceProperties and @MotorType annotations.
     */
   // private static final MotorConfigurationType MOTOR_CONFIG =
     //       MotorConfigurationType.getMotorType(GoBILDA1150RPM.class);

    public static final double TICKS_PER_REV = 145.6;
    public static final double MAX_RPM = 1150;
    /*
     * Set the first flag appropriately. If using the built-in motor velocity PID, update
     * MOTOR_VELO_PID with the tuned coefficients from DriveVelocityPIDTuner.
     */
    public static final boolean RUN_USING_ENCODER = true;
    public static final PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(20,0,10);

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    public static double WHEEL_RADIUS = 2; //1.9685; //2 in
    public static double GEAR_RATIO = .345;//.5 //.345 // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 30; //15; //9.7;//28 //14;

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    public static double kV =.016;// 0.01465;//0.01175;//1.0 / rpmToVelocity(1150);
    public static double kA = 0; //0
    public static double kStatic = 0; //0
    public static double WHEEL_BASE = 0;
    public static double LATERAL_MULTIPLIER=2.0;  // Drew had 2.4

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. The velocity and
     * acceleration values are required, and the jerk values are optional (setting a jerk of 0.0
     * forces acceleration-limited profiling).
     */
    public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(
            30, 30, 0.0,
            Math.toRadians(180.0), Math.toRadians(180.0), 0.0
    );

    /*
     * Note from LearnRoadRunner.com:
     * The velocity and acceleration constraints were calculated based on the following equation:
     * ((MAX_RPM / 60) * GEAR_RATIO * WHEEL_RADIUS * 2 * Math.PI) * 0.8
     * Resulting in 94.82478545840311 in/s.
     * This is only 80% of the theoretical maximum velocity of the bot, following the recommendation above.
     * This is capped at 80% because there are a number of variables that will prevent your bot from actually
     * reaching this maximum velocity: voltage dropping over the game, bot weight, general mechanical inefficiencies, etc.
     * However, you can push this higher yourself if you'd like. Perhaps raise it to 90-95% of the theoretically
     * max velocity. The theoretically maximum velocity is 118.53098182300388 in/s.
     * Just make sure that your bot can actually reach this maximum velocity. Path following will be detrimentally
     * affected if it is aiming for a velocity not actually possible.
     *
     * The maximum acceleration is somewhat arbitrary and it is recommended that you tweak this yourself based on
     * actual testing. Just set it at a reasonable value and keep increasing until your path following starts
     * to degrade. As of now, it simply mirrors the velocity results in 94.82478545840311 in/s/s
     *
     * Maximum Angular Velocity is calculated as maximum velocity / (trackWidth / 2) * (180 / Math.PI)
     */
    /*  Base values for 80% of theoretical max
     public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(
            94.82478545840311, 94.82478545840311, 0.0,
            Math.toRadians(603.6733333333333), Math.toRadians(603.6733333333333), 0.0
    );
     */

    public static double getTheatricalMaxVelocity()
    {
        return ((MAX_RPM / 60) * GEAR_RATIO * WHEEL_RADIUS * 2 * Math.PI) * 0.8;
    }
    public static double getTheoreticalMaxAngVelocity()
    {
        return getTheatricalMaxVelocity() / (TRACK_WIDTH / 2) * (180 / Math.PI);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF() {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 * 60 / (MAX_RPM * TICKS_PER_REV);
    }
}
