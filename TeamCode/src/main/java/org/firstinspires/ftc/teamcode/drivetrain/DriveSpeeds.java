package org.firstinspires.ftc.teamcode.drivetrain;

import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;

/**
 * preset speeds when for followtrajectory().
 */
public class DriveSpeeds {

    public static DriveConstraints slow = new DriveConstraints(5, 5, 0.0,
            Math.toRadians(45), Math.toRadians(45), 0.0);

    public static DriveConstraints medium = new DriveConstraints(25,25,0.0,
            Math.toRadians(180), Math.toRadians(180),0.0);

    public static DriveConstraints fast = new DriveConstraints(35,35,0.0,
            Math.toRadians(180),Math.toRadians(180),0.0);
    public static DriveConstraints reallyFast = new DriveConstraints(50,50,0.0,
            Math.toRadians(270),Math.toRadians(270),0.0);

    public DriveSpeeds() {}
}
