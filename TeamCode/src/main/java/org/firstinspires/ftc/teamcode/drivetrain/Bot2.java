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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.PID.VelocityPIDFController;
import org.firstinspires.ftc.teamcode.util.ThrowerUtil;

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
    public Servo rightArm;
    public Servo leftArm;

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
        leftArm = opMode.hardwareMap.get(Servo.class, "leftArm");

        colorSensor = opMode.hardwareMap.get(ColorSensor.class, "color2");

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
        return colorSensor.red() > 16;
    }

    public void lowerArms() {
        //rightArm.setPosition(0);
        //leftArm.setPosition(1);
    }
    public void raiseArms() {
        //rightArm.setPosition(1);
        //leftArm.setPosition(0)
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
        return super.getThrowerVelocityFromPosition(currentPos);
    }

    @Override
    public double getThrowerVelocityFromPosition(Pose2d currentPos, AngleUnit angleUnit) {
        return super.getThrowerVelocityFromPosition(currentPos, angleUnit);
    }

    @Override
    public double getThrowerVelocityFromPosition(double dist, AngleUnit angleUnit) {
        return super.getThrowerVelocityFromPosition(dist, angleUnit);
    }

    @Override
    public double getThrowerVelocityFromPosition(double dist) {
        return super.getThrowerVelocityFromPosition(dist);
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
        if (!isLaunching() && conveyor.getCurrentPosition() > CONVEYOR_TARGET_POS) {
            stopConveyor();
            zeroConveyor();
        }
    }

    @Override
    public void launchRing() {
        conveyor.setPower(1);
        //intake();
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
        wobbleGoalGrabber.setPosition(.57);
    }

    @Override
    public void openWobbleGrabberHalfway() {
        wobbleGoalElevator.setPosition(.45);
    }

    @Override
    public void releaseWobbleGoal() {
       wobbleGoalGrabber.setPosition(.3);
    }

    @Override
    public void lowerWobvator() {
        wobbleGoalElevator.setPosition(.14);
    }

    @Override
    public void lowerWobvatorByNotAllTheWay() {
        wobbleGoalElevator.setPosition(.18);
    }

    @Override
    public void raiseWobvator() {
        wobbleGoalElevator.setPosition(.33);
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
    }

    public double throwerTicksToRev(double ticks, DcMotorEx motor) {
        return ticks/28;
    }

}
