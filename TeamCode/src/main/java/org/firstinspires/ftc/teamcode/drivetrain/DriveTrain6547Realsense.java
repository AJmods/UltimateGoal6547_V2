package org.firstinspires.ftc.teamcode.drivetrain;

import androidx.annotation.NonNull;

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
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.util.command.PacketAction;
import org.firstinspires.ftc.teamcode.drivetrain.localizer.T265LocalizerRR;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.pipeline.openCvPipeLines;
import org.firstinspires.ftc.teamcode.util.roadrunner.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.roadrunner.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.util.homar.Button;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drivetrain.DriveConstants.BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.drivetrain.DriveConstants.LATERAL_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.drivetrain.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drivetrain.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drivetrain.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drivetrain.DriveConstants.WHEEL_BASE;
import static org.firstinspires.ftc.teamcode.drivetrain.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drivetrain.DriveConstants.getMotorVelocityF;
import static org.firstinspires.ftc.teamcode.drivetrain.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drivetrain.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drivetrain.DriveConstants.kV;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class DriveTrain6547Realsense extends MecanumDrive {

    public static boolean USE_REALSENSE = true;
    public static PIDCoefficients X_PID = new PIDCoefficients(3, 0, 0);
    public static PIDCoefficients Y_PID = new PIDCoefficients(4, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(1, 0, 0);

    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    //don't touch this unless you know what it does
    private PacketAction packetAction = (packet, fieldOverlay) -> {
        //do nothing
    };

    public static String POS_FILE_NAME = "pos.txt";

    public openCvPipeLines.RingDetectionPipeLine ringDetectionPipeLine;
    public OpenCvCamera webCam;

    private FtcDashboard dashboard;
    public NanoClock clock;

    public Mode mode;

    public PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    public DriveConstraints constraints;
    private TrajectoryFollower follower;

    private List<Pose2d> poseHistory;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private BNO055IMU imu;

    public Servo wobbleGoalGrabber;
    public Servo wobbleGoalElevator;
    public Servo indexer;
    public DcMotorEx intake;
    public DcMotorEx thrower1, thrower2;
    private double[] motorPowersToAdd = new double[]{0,0,0,0};

    public RevBlinkinLedDriver lights;

    private Pose2d lastPoseOnTurn;

    private OpMode opMode;

    public Button a1,a2,b1,b2,x1,x2,y1,y2,dpadUp1,dpadUp2,dpadDown1, dpadDown2, dpadLeft1, dpadLeft2,dpadRight1, dpadRight2,leftBumper1,leftBumper2,rightBumper1,rightBumper2,start1,start2, rightTrigger1, rightTrigger2, leftTrigger1, leftTrigger2;

    double currentHeadingV=0;
    double currentHeadingA=0;

    boolean inTaking = false;
    boolean outTaking = false;

    double targetVelocity = 0;
    double leewayVelo = 360;
    public DriveTrain6547Realsense(OpMode opMode) {
        super(kV, kA, kStatic, TRACK_WIDTH, WHEEL_BASE, LATERAL_MULTIPLIER);
        USE_REALSENSE = true;
        this.opMode = opMode;
        initRobot();
    }
    public DriveTrain6547Realsense(OpMode opMode, boolean resetRealsense) {
        super(kV, kA, kStatic, TRACK_WIDTH, WHEEL_BASE, LATERAL_MULTIPLIER);
        //this.USE_REALSENSE = USE_REALSENSE;
        this.opMode = opMode;
        initRobot(resetRealsense);
    }
    private void initRobot() {initRobot(true);}
    private void initRobot(boolean resetRealsense) {
//        if (!USE_REALSENSE) {
//            RobotLog.setGlobalWarningMessage("NOT USING REALSENSE, USING DEFAULT LOCALIZER", "");
//        }

        opMode.telemetry.log().add("Initializing Robot");
        HardwareMap hardwareMap=opMode.hardwareMap;

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        clock = NanoClock.system();

        mode = Mode.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
        follower = new HolonomicPIDVAFollower(X_PID, Y_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);
        //this is new, if follower is weird, mess with admissible error and timeout

        poseHistory = new ArrayList<>();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration

        //don't use IMU, using realsense instead
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        //new code
        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        // TODO: if desired, use setLocalizer() to change the localization method

        if (USE_REALSENSE) {
            try {
                opMode.telemetry.log().add("Initializing Realsense....This may take some time");
                setLocalizer(new T265LocalizerRR(hardwareMap, resetRealsense));
            } catch (Exception e) {
                RobotLog.setGlobalWarningMessage("FAILED TO INIT REALSENSE", "FAILED TO INIT REALSENSE");
            }
        }
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));

        initOtherHardware();
        initGamepads(); //for tele-op
    }
    private void initOtherHardware() {

        wobbleGoalGrabber = opMode.hardwareMap.get(Servo.class, "wob");
        indexer = opMode.hardwareMap.get(Servo.class, "indexer");
        wobbleGoalElevator = opMode.hardwareMap.get(Servo.class, "wobvator");

        thrower1 = opMode.hardwareMap.get(DcMotorEx.class, "thrower");
        thrower2 = opMode.hardwareMap.get(DcMotorEx.class, "thrower2");
        try {
            intake = opMode.hardwareMap.get(DcMotorEx.class, "intake");
        } catch (Exception e) {
            RobotLog.setGlobalWarningMsg((RobotCoreException) e, "WILL CONTINUE AS NORMAL");
        }

        lights = opMode.hardwareMap.get(RevBlinkinLedDriver.class, "lights");

        raiseWobvator();
        openIndexer();
        grabWobbleGoal();

        thrower1.setDirection(DcMotorSimple.Direction.REVERSE);
        thrower2.setDirection(DcMotorSimple.Direction.REVERSE);

        thrower1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        thrower1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        thrower2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        thrower2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void initOpenCV() {
        opMode.telemetry.log().add("Initializing OpenCV");
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
        webCam = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webCam.openCameraDevice();//open camera
        //set pipeline
        //ringDetectionPipeLine = new openCvPipeLines.RingDetectionPipeLine();
        webCam.setPipeline(new openCvPipeLines.RingDetectionPipeLine());

        //webCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
    }
    private void initGamepads() //set the buttons to thier values
    {
        a1 = new Button();
        a2 = new Button();
        b1 = new Button();
        b2 = new Button();
        x1 = new Button();
        x2 = new Button();
        y1 = new Button();
        y2 = new Button();
        dpadDown1 = new Button();
        dpadDown2 = new Button();
        dpadLeft1 = new Button();
        dpadLeft2 = new Button();
        dpadRight1 = new Button();
        dpadRight2 = new Button();
        dpadUp1 = new Button();
        dpadUp2 = new Button();
        rightBumper1 = new Button();
        rightBumper2 = new Button();
        leftBumper1 = new Button();
        leftBumper2 = new Button();
        start1=new Button();
        start2=new Button();
        leftTrigger1 = new Button();
        leftTrigger2 = new Button();
        rightTrigger1 = new Button();
        rightTrigger2 = new Button();
    }
    public void updateGamepads() //update the gamepad buttons from HOMAR-FTC-Libary for tele-op
    {
        Gamepad gamepad1 = opMode.gamepad1;
        Gamepad gamepad2 = opMode.gamepad2;
        a1.input(gamepad1.a);
        a2.input(gamepad2.a);
        b1.input(gamepad1.b);
        b2.input(gamepad2.b);
        x1.input(gamepad1.x);
        x2.input(gamepad2.x);
        y1.input(gamepad1.y);
        y2.input(gamepad2.y);
        dpadUp1.input(gamepad1.dpad_up);
        dpadUp2.input(gamepad2.dpad_up);
        dpadRight1.input(gamepad1.dpad_right);
        dpadRight2.input(gamepad2.dpad_right);
        dpadUp1.input(gamepad1.dpad_up);
        dpadUp2.input(gamepad2.dpad_up);
        dpadLeft1.input(gamepad1.dpad_left);
        dpadLeft2.input(gamepad2.dpad_left);
        leftBumper1.input(gamepad1.left_bumper);
        leftBumper2.input(gamepad2.left_bumper);
        rightBumper1.input(gamepad1.right_bumper);
        rightBumper2.input(gamepad2.right_bumper);
        leftTrigger1.input(gamepad1.left_trigger>=.7);
        leftTrigger2.input(gamepad2.left_trigger>=.7);
        rightTrigger1.input(gamepad1.right_trigger>=.7);
        rightTrigger2.input(gamepad2.right_trigger>=.7);
    }

    public void startOpenCV() {
        webCam.openCameraDeviceAsync(() -> webCam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN));
    }
    public void stopOpenCV() {
        webCam.stopStreaming();
    }
    public openCvPipeLines.RingCount getRingCount() {
        return openCvPipeLines.RingDetectionPipeLine.getRingCount();
    }public void writeFile(String filename, double number)
    {
        try {
            File file = AppUtil.getInstance().getSettingsFile(filename);
            ReadWriteFile.writeFile(file, Double.toString(number));
            opMode.telemetry.log().add("saved " + number + " in " + filename);
        }
        catch(Exception e)
        {
            opMode.telemetry.log().add("Unable to write " + number + " in " + filename);
        }
    }
    public void writeFile(String filename, String str)
    {
        try {
            File file = AppUtil.getInstance().getSettingsFile(filename);
            ReadWriteFile.writeFile(file, str);
            opMode.telemetry.log().add("saved \"" + str + "\" in " + filename);
        }
        catch(Exception e)
        {
            opMode.telemetry.log().add("Unable to write \"" + str + "\" in " + filename);
        }
    }
    public double readFile(String filename)
    {
        try {
            double output=0;
            File file= AppUtil.getInstance().getSettingsFile(filename);
            output = Double.parseDouble(ReadWriteFile.readFile(file).trim());
            opMode.telemetry.log().add("read " + output + " in " + filename);
            return output;
        }
        catch (Exception e)
        {
            opMode.telemetry.log().add("Unable to read " + filename + ", returning 0");
            return 0;
        }
    }
    public String readFileString(String filename)
    {
        try {
            String output;
            File file= AppUtil.getInstance().getSettingsFile(filename);
            output = ReadWriteFile.readFile(file).trim();
            opMode.telemetry.log().add("read " + output + " in " + filename);
            return output;
        }
        catch (Exception e)
        {
            opMode.telemetry.log().add("Unable to read " + filename + ", returning 0");
            return "";
        }
    }
    public void savePos(Vector2d vector2d) {savePos(new Pose2d(vector2d.getX(), vector2d.getY(), Math.toRadians(0)));}
    public void savePos(Pose2d pose2d) {
        double x = pose2d.getX();
        double y = pose2d.getY();
        double heading = pose2d.getHeading();
        StringBuilder stringBuilder = new StringBuilder();
        String posFile = stringBuilder.append(x).append(",").append(y).append(",").append(heading).toString();
        RobotLog.v("saved POS as " + posFile + " to " + POS_FILE_NAME);
        writeFile(POS_FILE_NAME, posFile);
    }
    public Pose2d readPos() {
        String posString = readFileString(POS_FILE_NAME);
        String[] nums = posString.split(",");
        Pose2d pos = new Pose2d(Double.parseDouble(nums[0]), Double.parseDouble(nums[1]), Double.parseDouble(nums[2]));
        return pos;
    }
    public TrajectoryBuilder trajectoryBuilder() {
        return new TrajectoryBuilder(getPoseEstimate(), constraints);
    }
    public TrajectoryBuilder trajectoryBuilder(boolean reversed) {
        return new TrajectoryBuilder(getPoseEstimate(), reversed, constraints);
    }
    public TrajectoryBuilder trajectoryBuilder(boolean reversed, DriveConstraints constraints) {
        return new TrajectoryBuilder(getPoseEstimate(), reversed, constraints);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, constraints);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, constraints);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, constraints);
    }

    public void turn(double angle) {
        turn(angle, 0, 0, 0);
    }
    public void turn(double angle, double vi, double ai, double ji) {
        double heading = getPoseEstimate().getHeading();

        lastPoseOnTurn = getPoseEstimate();

        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, vi, ai, ji),
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
    public double turnTowardsAngle(Vector2d target) {return turnTowardsAngle(target,new Vector2d());}
    public double turnTowardsAngle(Vector2d target, Pose2d start) {return turnTowardsAngle(target, new Vector2d(start.getX(), start.getY()));}
    public double turnTowardsAngle(Vector2d target, Vector2d start) {
        double deltaX = target.getX() - start.getX();
        double deltaY = target.getY() - start.getY();

        double angle = Math.atan2(deltaY, deltaX);
        return (angle == Double.NaN) ? 0 : angle;
    }
    public void turnRealtiveSync(double angle)
    {
        double target=angle-getPoseEstimate().getHeading();
        //target-=Math.toRadians(90);
        if (Math.abs(target)>Math.toRadians(180)) //make the angle difference less then 180 to remove unnecessary turning
        {
            target+=(target>=0) ? Math.toRadians(-360) : Math.toRadians(360);
        }
        opMode.telemetry.log().add("inputted Angle: " + angle + " , turning to: " + target);
        RobotLog.d("Turning Realtive to heading " + angle + ", amount turning: " + target);
        turnSync(target);
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

        poseHistory.add(currentPose);

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("mode", mode);

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading", currentPose.getHeading());

        packet.put("xError", lastError.getX());
        packet.put("yError", lastError.getY());
        packet.put("headingError", lastError.getHeading());

        //draw goals
        fieldOverlay.setFill("#FF0000");
        fieldOverlay.fillCircle(FieldConstants.RED_POWER_SHOT_1X, FieldConstants.RED_POWER_SHOT_1Y, 1);
        fieldOverlay.fillCircle(FieldConstants.RED_POWER_SHOT_2X, FieldConstants.RED_POWER_SHOT_2Y, 1);
        fieldOverlay.fillCircle(FieldConstants.RED_POWER_SHOT_3X, FieldConstants.RED_POWER_SHOT_3Y, 1);

        //fieldOverlay.fillCircle(FieldConstants.RED_GOAL_X, FieldConstants.RED_GOAL_Y, 8);

        switch (mode) {
            case IDLE:
                // do nothing
                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);

                turnController.setTargetPosition(targetState.getX());

                double correction = turnController.update(currentPose.getHeading());
                if (Double.isNaN(correction)) correction = 0;

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                setDriveSignal(new DriveSignal(new Pose2d(
                        0, 0, targetOmega + correction
                ), new Pose2d(
                        0, 0, targetAlpha
                )));

               // RobotLog.v("Target Omeaga: " + targetOmega + "COrrection: " + correction);

                currentHeadingV = targetOmega + correction;
                currentHeadingA = targetAlpha;

                Pose2d newPose = lastPoseOnTurn.copy(lastPoseOnTurn.getX(), lastPoseOnTurn.getY(), targetState.getX());

                fieldOverlay.setStroke("#4CAF50");
                DashboardUtil.drawRobot(fieldOverlay, newPose);

                if (t >= turnProfile.duration()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
            case FOLLOW_TRAJECTORY: {
                DriveSignal driveSignal = follower.update(currentPose);
                setDriveSignal(driveSignal);

                Trajectory trajectory = follower.getTrajectory();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("#4CAF50");
                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.getPath());
                double t = follower.elapsedTime();
                DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));

                fieldOverlay.setStroke("#3F51B5");
                DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
        }

        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, currentPose);

        DashboardUtil.drawLine(fieldOverlay, currentPose);

        packetAction.addToPacket(packet, fieldOverlay);

        dashboard.sendTelemetryPacket(packet);

        runAtAllTimes();
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update();
        }
    }

    public void setThrowerVelocity(double ticksPerSecond) {
        if (ticksPerSecond == 0) {
            thrower2.setPower(0);
            thrower1.setPower(0);
            targetVelocity = 0;
            return;
        }
        try {
            targetVelocity = ticksPerSecond;
            thrower1.setVelocity(ticksPerSecond);
            thrower2.setVelocity(ticksPerSecond);
        } catch (Exception e) {
            RobotLog.e("SETTING tick VELOCITY FAILED");
        }
    }
    public void setThrowerVelocity(double angularRate, AngleUnit angleUnit) {
        if (angularRate == 0) {
            thrower2.setPower(0);
            thrower1.setPower(0);
            targetVelocity = 0;
            return;
        }
        try {
            targetVelocity = angularRate;
            thrower1.setVelocity(angularRate, angleUnit);
            thrower2.setVelocity(angularRate, angleUnit);
        } catch (Exception e) {
            RobotLog.e("SETTING angle VELOCITY FAILED");
        }
        if (angleUnit == AngleUnit.DEGREES) RobotLog.v("Setting Thrower Velocity to " + (angularRate/360) +"REV/s");
        if (angleUnit == AngleUnit.RADIANS) RobotLog.v("Setting Thrower Velocity to " + (angularRate/(2*Math.PI)) +"REV/s");
    }
    public double[] getThrowerVelocity() {
        try {return new double[] {thrower1.getVelocity(), thrower2.getVelocity()}; }
        catch (Exception e) {
            return new double[] {0,0};
        }
    }
    public double[] getThrowerVelocity(AngleUnit angleUnit) {
        try {
            return new double[] {thrower1.getVelocity(angleUnit), thrower2.getVelocity(angleUnit)};
        } catch (Exception e) {
            return new double[] {0,0};
        }
    }
    public void stopThrower() {
        thrower1.setPower(0);
        thrower2.setPower(0);
    }
    public boolean isReadyToThrow() {
        double[] velocities = getThrowerVelocity(AngleUnit.DEGREES);
        double minVelo = targetVelocity - leewayVelo;
        double maxVelo = targetVelocity + leewayVelo;
        boolean isMotor0AtTarget = velocities[0] > minVelo && velocities[0] < maxVelo;
        boolean isMotor1AtTarget = velocities[1] > minVelo && velocities[1] < maxVelo;

        return  isMotor0AtTarget || isMotor1AtTarget;
    }
    public void updateLightsBasedOnThrower() {
        if (targetVelocity == 0 || !isReadyToThrow()) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }
    public void launchRing() {
        try {
            indexer.setPosition(.55);
        } catch (Exception e) {
            RobotLog.e("LAUCNH RING FAILED");
        }
    }
    public void openIndexer() {
        try {
            indexer.setPosition(.2);
        } catch (Exception e) {
            RobotLog.e("OPEN INDEXER FAILED");
        }
    }
    public void midIndexer(){
        try {
            indexer.setPosition(.38);
        } catch (Exception e) {
            RobotLog.e("MID INDEXER FAILED");
        }
    }
    public void grabWobbleGoal() {

        try {
            wobbleGoalGrabber.setPosition(.15);
        } catch (Exception e) {
            RobotLog.e("GRAB WOB GOAL FAILED");
        }
    }
    public void releaseWobbleGoal() {
        try {
            wobbleGoalGrabber.setPosition(.45);
        } catch (Exception e) {
            RobotLog.e("RELEASE WOB GOAL FAILED");
        }
    }
    public void lowerWobvator() {
        try {
        wobbleGoalElevator.setPosition(.35);
        } catch (Exception e) {
            RobotLog.e("LOWER WOBVATOR FAILED");
        }
    }
    public void lowerWobvatorByNotAllTheWay() {
        try {
        wobbleGoalElevator.setPosition(.385);
        } catch (Exception e) {
            RobotLog.e("LOWER WOBVATOR but not all the way FAILED");
        }
    }
    public void raiseWobvator() {
        try {
        wobbleGoalElevator.setPosition(.42);
        } catch (Exception e) {
            RobotLog.e("RAISE WOBVATOR FAILED");
        }
    }

    public void intake() {
        intake.setPower(1);
        inTaking = true;
        outTaking = false;
    }
    public void outtake() {
        intake.setPower(-1);
        outTaking = true;
        inTaking = false;
    }
    public void stopIntake() {
        intake.setPower(0);
        outTaking = false;
        inTaking = false;
    }
    public boolean isIntaking() {
        return inTaking;
    }
    public boolean isOutTaking() {
        return outTaking;
    }

    public void updateServo(Servo servo, double gamepadStick, double speed, double max, double min)
    {
        if (Math.abs(gamepadStick) < .2) return;
        double posToAdd = gamepadStick*speed;
        opMode.telemetry.addData("changing pos by ",posToAdd);
        double servoCurrentPos = servo.getPosition();
        double targetPos = posToAdd + servoCurrentPos;
        //if ((servoCurrentPos > min && gamepadStick > 0) || (servoCurrentPos < max && gamepadStick < 0))
        //{
        if (targetPos > max) servo.setPosition(max);
        else if (targetPos < min) servo.setPosition(min);
        else {
            servo.setPosition(targetPos);
        }
        //}
    }
    public void updateServo(Servo servo, double gamepadStick, double speed)
    {
        updateServo(servo, gamepadStick, speed, 0, 1);
    }
    public void runAtAllTimes() //anything in here runs at all times during auton because this method is ran during roadRunner's state machine
    {

    }
    //convert a driveSignal to motor powers
    public double[] driveSignalToPowers(DriveSignal driveSignal)
    {
        List<Double> vel = MecanumKinematics.robotToWheelVelocities(driveSignal.getVel(), TRACK_WIDTH, WHEEL_BASE, LATERAL_MULTIPLIER);
        List<Double> accel = MecanumKinematics.robotToWheelAccelerations(driveSignal.getAccel(), TRACK_WIDTH, WHEEL_BASE, LATERAL_MULTIPLIER);
        List<Double> powers = Kinematics.calculateMotorFeedforward(vel, accel, kV,kA,kStatic);
        //convert list to array
        return powers.stream().mapToDouble(d -> d).toArray();
    }
    //Get and set methods

    public boolean isBusy() {
        return mode != Mode.IDLE;
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftFront.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, getMotorVelocityF()
            ));
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }
    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }
    public void stopRobot()
    {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

    @Override
    public double getRawExternalHeading() {
        if (USE_REALSENSE)
            return T265LocalizerRR.getHeading();
        else
            return imu.getAngularOrientation().firstAngle;
    }

    public double getCurrentHeadingA() {
        return currentHeadingA;
    }

    public double getCurrentHeadingV() {
        return currentHeadingV;
    }

    public void setPacketAction(PacketAction packetAction) {
        this.packetAction = packetAction;
    }
}
