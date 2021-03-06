package org.firstinspires.ftc.teamcode.drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.PID.VelocityPIDFController;
import org.firstinspires.ftc.teamcode.util.ThrowerUtil;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

@Config
public class Bot2 extends DriveTrain6547Realsense {

    public static double CONVEYOR_TARGET_POS = 700;

    public static PIDCoefficients X_PID = new PIDCoefficients(3, 0, 0);
    public static PIDCoefficients Y_PID = new PIDCoefficients(4, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(1, 0, 0);

    public static double MOTOR_TICKS_PER_REV = 28;
    public static double MOTOR_MAX_RPM = 5400;
    public static double MOTOR_GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed

    // Copy your feedforward gains here
    public static double kV = 0.000565; //1 / TuningController.rpmToTicksPerSecond(TuningController.MOTOR_MAX_RPM);
    public static double kA = 0.0007;
    public static double kStatic = 0;
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(.001, 0, 0.0005);

    private final VelocityPIDFController veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);

    private double lastKp = 0.0;
    private double lastKi = 0.0;
    private double lastKd = 0.0;
    private double lastKf = getMotorVelocityF();

    private double targetVelo=0;
    private double lastTargetVelo=0;

    private ElapsedTime veloTimer = new ElapsedTime();

    public DcMotorEx conveyor;
    private ColorSensor colorSensor;
    private ColorSensor colorSensor2;
    public Servo rightArm;

    private OpMode opMode;

    private double oldConveyorPower = 0;

    private boolean isLaunching;

    public Bot2(OpMode opMode) {
        super(opMode, false);
        this.opMode = opMode;
        DriveTrain6547Realsense.X_PID = X_PID;
        DriveTrain6547Realsense.Y_PID = Y_PID;
        DriveTrain6547Realsense.HEADING_PID = HEADING_PID;
        opMode.telemetry.log().add("Initing Bot 2");

        initRobot();
        initBot2Hardware();
    }

    private void initBot2Hardware() {
        conveyor = opMode.hardwareMap.get(DcMotorEx.class, "conveyor");
        conveyor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightArm = opMode.hardwareMap.get(Servo.class, "rightArm");

        colorSensor = opMode.hardwareMap.get(ColorSensor.class, "color3");
        colorSensor2 = opMode.hardwareMap.get(ColorSensor.class, "color4");

        //conveyor.setPower(1);


        raiseWobvator();
        openIndexer();
        grabWobbleGoal();
    }

    private void initThrowerMotors() {
//        MotorConfigurationType motorConfigurationType = thrower1.getMotorType().clone();
//        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
//        thrower1.setMotorType(motorConfigurationType);
//
//        MotorConfigurationType motorConfigurationType2 = thrower1.getMotorType().clone();
//        motorConfigurationType2.setAchieveableMaxRPMFraction(1.0);
//        thrower2.setMotorType(motorConfigurationType2);

        thrower1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        thrower2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        setPIDCoefficients(thrower1, MOTOR_VELO_PID);
//        setPIDCoefficients(thrower2, MOTOR_VELO_PID);
    }

    public void updateThrower() {
        veloController.setTargetVelocity(targetVelo);
        veloController.setTargetAcceleration((targetVelo - lastTargetVelo) / veloTimer.seconds());
        veloTimer.reset();

        lastTargetVelo = targetVelo;

        // Get the velocity from the motor with the encoder
        double motorPos = thrower1.getCurrentPosition();
        double motorVelo = thrower1.getVelocity();

        // Update the controller and set the power for each motor
        double power = veloController.update(motorPos, motorVelo);
        thrower1.setPower(power);
        thrower2.setPower(power);

        opMode.telemetry.addData("Thrower Velocity 1", thrower1.getVelocity());
        opMode.telemetry.addData("Thrower Velocity 2", thrower2.getVelocity());

    }

    public void setConveyor(double power) {
        conveyor.setPower(power);
    }

    public void pushRingsThroughConveyor() {
        conveyor.setPower(1);
    }

    public void stopConveyor() {
        conveyor.setPower(0);
    }

    public void loadRingInConveyor() {
        conveyor.setPower(1);
    }

    public void zeroConveyor() {
        conveyor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        conveyor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isRingAtConveyor() {
        return isRingAtConveyor(colorSensor) || isRingAtConveyor(colorSensor2);
    }
    public boolean isRingAtConveyor(ColorSensor colorSensor) {
        return colorSensor.blue() < 210;
    }

    public void lowerArms() {
        //rightArm.setPosition(0);
    }
    public void raiseArms() {
        //rightArm.setPosition(1);
    }

    public void reverseMotors() {
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    //    @Override
//    public void setThrowerVelocity(double ticksPerSecond) {
//        targetVelo = ticksPerSecond;
//        updateThrower();
//    }

//    @Override
//    public void setThrowerVelocity(double angularRate, AngleUnit angleUnit) {
//        super.setThrowerVelocity(angularRate, angleUnit);
////        if (angleUnit == AngleUnit.DEGREES) {
////            angularRate/=360; //convert to REV/s
////            angularRate*=28; //convert to ticksPerSec
////        } else if (angleUnit == AngleUnit.RADIANS) {
////            angularRate/=Math.toRadians(360); //convert to REV/s
////            angularRate*=28; //convert to ticksPerSec
////        }
////
////        setThrowerVelocity(angularRate);
//    }

    @Override
    public double getThrowerVelocityFromPosition(Pose2d currentPos) {
        //get distance based
        double targetY = ThrowerUtil.getTargetY(currentPos, FieldConstants.RED_GOAL_X);
        double dist = Math.hypot(currentPos.getX() - FieldConstants.RED_GOAL_X, currentPos.getY() - targetY);
        double ticksPerSec = getThrowerVelocityFromPosition(dist);
        return ticksPerSec;
    }

//    @Override
//    public double getThrowerVelocityFromPosition(Pose2d currentPos, AngleUnit angleUnit) {
//        double targetY = ThrowerUtil.getTargetY(currentPos, FieldConstants.RED_GOAL_X);
//        double dist = Math.hypot(currentPos.getX() - FieldConstants.RED_GOAL_X, currentPos.getY() - targetY);
//        double unitsPerSec = getThrowerVelocityFromPosition(dist, angleUnit);
//        RobotLog.v("TARGET Y: " + targetY + ", DIST: " + dist + ", UNITS PER SEC: " + unitsPerSec);
//        return unitsPerSec;
//    }
//
//    @Override
//    public double getThrowerVelocityFromPosition(double dist, AngleUnit angleUnit) {
//        return super.getThrowerVelocityFromPosition(dist, angleUnit);
//    }

    @Override
    public double getThrowerVelocityFromPosition(double dist) {
        double ticksPerSec = 35.8219*Math.pow(1.0036,dist);
        return ticksPerSec;
    }

    @Override
    public double getThrowerVelocityFromPositionPowershot(double dist) {
        return super.getThrowerVelocityFromPositionPowershot(dist);
    }

    @Override
    public double getThrowerVelocityFromPositionPowerShot(double dist, AngleUnit angleUnit) {
        return super.getThrowerVelocityFromPositionPowerShot(dist, angleUnit);
    }

    @Override
    public void update() {
        super.update();

    }

    @Override
    public void launchRing() {
        conveyor.setPower(1);
        intake();
        isLaunching = true;
    }

    @Override
    public void openIndexer() {
        //super.openIndexer();
    }

    @Override
    public void midIndexer() {
        //super.midIndexer();
    }

    @Override
    public void grabWobbleGoal() {
        wobbleGoalGrabber.setPosition(.1);
    }

    @Override
    public void openWobbleGrabberHalfway() {
        wobbleGoalGrabber.setPosition(.5);
    }

    @Override
    public void releaseWobbleGoal() {
       wobbleGoalGrabber.setPosition(.7);
    }

    @Override
    public void lowerWobvator() {
        wobbleGoalElevator.setPosition(.3);
    }

    @Override
    public void lowerWobvatorByNotAllTheWay() {
        wobbleGoalElevator.setPosition(.4);
    }

    @Override
    public void raiseWobvator() {
        wobbleGoalElevator.setPosition(.75);
    }

    public static double getMotorVelocityF() {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 * 60.0 / (MOTOR_MAX_RPM * MOTOR_TICKS_PER_REV);
    }

    public boolean isLaunching() {
        return isLaunching;
    }
    public void stopLaunch() {
        isLaunching = false;
        stopConveyor();
        stopIntake();
    }

    public double throwerTicksToRev(double ticks, DcMotorEx motor) {
        return ticks/28;
    }

}
