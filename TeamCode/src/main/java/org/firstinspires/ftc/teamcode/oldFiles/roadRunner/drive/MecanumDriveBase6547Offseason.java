package org.firstinspires.ftc.teamcode.oldFiles.roadRunner.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.roadrunner.DashboardUtil;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drivetrain.DriveConstants.BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.drivetrain.DriveConstants.LATERAL_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.drivetrain.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drivetrain.DriveConstants.WHEEL_BASE;
import static org.firstinspires.ftc.teamcode.drivetrain.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drivetrain.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drivetrain.DriveConstants.kV;

/*
 * Base class with shared functionality for sample mecanum drives. All hardware-specific details are
 * handled in subclasses.
 */
@Config
public abstract class MecanumDriveBase6547Offseason extends MecanumDrive {
    public static PIDCoefficients X_PID = new PIDCoefficients(.4, 0, 0);
    public static PIDCoefficients Y_PID = new PIDCoefficients(.25,0,0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(.65, 0, 0);

    public int AutonLiftTargetPos = 0;
    public boolean runLift = false;


    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    public FtcDashboard dashboard;
    private NanoClock clock;

    private Mode mode;

    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    private DriveConstraints constraints;
    private TrajectoryFollower follower;

    private List<Double> lastWheelPositions;
    private double lastTimestamp;

    private DcMotorEx leftEncoder,rightEncoder,frontEncoder;

    private int loops=0;

    public static int LOOP_REM = 10;

    TelemetryPacket packet;

    Canvas fieldOverlay;

    public MecanumDriveBase6547Offseason(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH,WHEEL_BASE,LATERAL_MULTIPLIER);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        clock = NanoClock.system();

        mode = Mode.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
        follower = new HolonomicPIDVAFollower(X_PID, Y_PID, HEADING_PID);

        leftEncoder = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        rightEncoder = (DcMotorEx) hardwareMap.dcMotor.get("sideEncoder");
        frontEncoder = (DcMotorEx) hardwareMap.dcMotor.get("vertRight");

        frontEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public TrajectoryBuilder trajectoryBuilder() {
        return new TrajectoryBuilder(getPoseEstimate(), constraints);
    }
    public TrajectoryBuilder trajectoryBuilder(boolean reversed, DriveConstraints constraints)
    {
        Pose2d pos;
        //turn robot heading
        if (reversed)
        {
            pos = new Pose2d(getPoseEstimate().getX(), getPoseEstimate().getY(),norm(getPoseEstimate().getHeading() + Math.toRadians(180)));
        }
        else
        {
            pos = getPoseEstimate();
        }
        return new TrajectoryBuilder(pos,reversed,constraints);

    }
    public TrajectoryBuilder trajectoryBuilder(boolean reversed)
    {
        return trajectoryBuilder(reversed,constraints);
    }

    public void turn(double angle) {
        double heading = getPoseEstimate().getHeading();
        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                constraints.maxAngVel,
                constraints.maxAngAccel,
                constraints.maxAngJerk
        );
        turnStart = clock.seconds();
        mode = Mode.TURN;
    }

    public void turnSync(double angle) {
        turn(angle);
        waitForIdle();
    }

    public void followTrajectory(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
        mode = Mode.FOLLOW_TRAJECTORY;
    }

    public void followTrajectorySync(Trajectory trajectory) {
        followTrajectory(trajectory);
        waitForIdle();
    }

    public Pose2d getLastError() {
        switch (mode) {
            case FOLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case IDLE:
                return new Pose2d();
        }
        throw new AssertionError();
    }

    public void update() {
        updatePoseEstimate();

        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();

        if (loops%LOOP_REM == 0) {



            packet = new TelemetryPacket();
            fieldOverlay = packet.fieldOverlay();

            packet.put("mode", mode);

            packet.put("x", currentPose.getX());
            packet.put("y", currentPose.getY());
            packet.put("heading", currentPose.getHeading());

            packet.put("xError", lastError.getX());
            packet.put("yError", lastError.getY());
            packet.put("headingError", lastError.getHeading());

            packet.put("Left Encoder Pos:", leftEncoder.getCurrentPosition());
            packet.put("Right Encoder Pos:", rightEncoder.getCurrentPosition());
            packet.put("Perpendicular Encoder Pos:", frontEncoder.getCurrentPosition());
        }

        switch (mode) {
            case IDLE:
                // do nothing
                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);

                turnController.setTargetPosition(targetState.getX());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                double correction = turnController.update(currentPose.getHeading(), targetOmega);

                setDriveSignal(new DriveSignal(new Pose2d(
                        0, 0, targetOmega + correction
                ), new Pose2d(
                        0, 0, targetAlpha
                )));

                if (t >= turnProfile.duration()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("4CAF50");

                fieldOverlay.setStroke("#3F51B5");
                //DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);
                DashboardUtil.drawRobot(fieldOverlay, currentPose);

                break;
            }
            case FOLLOW_TRAJECTORY: {
                setDriveSignal(follower.update(currentPose));

                Trajectory trajectory = follower.getTrajectory();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("4CAF50");
                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.getPath());
                double t = follower.elapsedTime();
                DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));

                fieldOverlay.setStroke("#3F51B5");
                //DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);
                DashboardUtil.drawRobot(fieldOverlay, currentPose);

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
        }
        runAtAllTimes();
        if (loops % LOOP_REM == 0) {
            dashboard.sendTelemetryPacket(packet);
        }
    }
    abstract void runAtAllTimes();

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update();
        }
    }

    public boolean isBusy() {
        return mode != Mode.IDLE;
    }

    public List<Double> getWheelVelocities() {
        List<Double> positions = getWheelPositions();
        double currentTimestamp = clock.seconds();

        List<Double> velocities = new ArrayList<>(positions.size());;
        if (lastWheelPositions != null) {
            double dt = currentTimestamp - lastTimestamp;
            for (int i = 0; i < positions.size(); i++) {
                velocities.add((positions.get(i) - lastWheelPositions.get(i)) / dt);
            }
        } else {
            for (int i = 0; i < positions.size(); i++) {
                velocities.add(0.0);
            }
        }

        lastTimestamp = currentTimestamp;
        lastWheelPositions = positions;

        return velocities;
    }

    public void setLiftTargetPos(int autonLiftTargetPos) {
        AutonLiftTargetPos = autonLiftTargetPos;
        //setRunLift(true);
    }
    private double norm(double angle)
    {
        while (angle>Math.toRadians(360)) angle-=Math.toRadians(360);
        while (angle<=0) angle+=Math.toRadians(360);
        return angle;
    }

    public void setConstraints(DriveConstraints constraints) {
        this.constraints = constraints;
    }

    public DriveConstraints getConstraints() {
        return constraints;
    }

    public int getLiftTargetPos() {
        return AutonLiftTargetPos;
    }

    public void setRunLift(boolean runLift) {
        this.runLift = runLift;
    }

    public abstract PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode);

    public abstract void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients);
}
