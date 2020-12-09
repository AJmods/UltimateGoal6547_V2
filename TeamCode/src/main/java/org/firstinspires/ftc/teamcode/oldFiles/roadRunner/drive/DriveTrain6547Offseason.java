package org.firstinspires.ftc.teamcode.oldFiles.roadRunner.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.oldFiles.roadRunner.localizer.StandardTwowheelLocalizerOffseason;
import org.firstinspires.ftc.teamcode.util.roadrunner.AxesSigns;
import org.firstinspires.ftc.teamcode.util.roadrunner.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.roadrunner.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.SkyStoneLoc;
import org.firstinspires.ftc.teamcode.util.homar.Button;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drivetrain.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drivetrain.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drivetrain.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drivetrain.DriveConstants.getMotorVelocityF;

/*
 * Simple mecanum drive hardware implementation for REV hardware. If your hardware configuration
 * satisfies the requirements, SampleMecanumDriveREVOptimized is highly recommended.
 */
public class DriveTrain6547Offseason extends MecanumDriveBase6547Offseason {

    private static double angleZzeroValue=0;

    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime grabberSlideTime = new ElapsedTime();

    public final String GYRO_ANGLE_FILE_NAME="gyroAngle.txt";
    public final String LIFT_MAX_FILE_NAME="liftMax.txt";
    public final String LIFT_STARTING_POS_FILE_NAME="liftStartPos.txt";
    public final String GRABBER_MAX_FILE_NAME = "grabberMax.txt";
    public final String GRABBER_MIN_FILE_NAME = "grabberMin.txt";
    public final String ROBOT_POS_FILE_NAME = "robotPos.txt";

    public DcMotorEx lift;
    public DcMotorEx intake;

    public Servo fondationGrabber;
    public Servo fondationGrabber2;
    public CRServo grabberSlide;
    public Servo grabber;
    public CRServo measuringTape;

    public ColorSensor colorSensorSideLeft;
    public ColorSensor colorSensorSideRight;
    public RevColorSensorV3 intakeColorSensor;
    public ColorSensor endColorSensor;
    public ColorSensor rightEndColorSensor;

    RevBlinkinLedDriver lights;

    public SkyStoneLoc skyStoneLoc=SkyStoneLoc.CENTER; //defualt to center

    public Button a1,a2,b1,b2,x1,x2,y1,y2,dpadUp1,dpadUp2,dpadDown1, dpadDown2, dpadLeft1, dpadLeft2,dpadRight1, dpadRight2,leftBumper1,leftBumper2,rightBumper1,rightBumper2,start1,start2, rightTrigger1, rightTrigger2, leftTrigger1, leftTrigger2;

    int[] levels = new int[] {700,3000}; //linear slide levels, 0 is bottom, 1 is to drop the skystone on top of the fondation, 2 is on top on the first stone, 3 is on top of the second stone, and so one.

    public int liftMax = 10000;
    public int liftMin = 0;
    public int liftStartingPos=0;
    private int liftLeeway = 50;
    public double grabberMin = 0;
    public double grabberMax = 1;
    private double grabberSlideTargetTime=0;

    Rev2mDistanceSensor distanceSensorX;
    Rev2mDistanceSensor distanceSensorY;

    public AnalogInput limitSwitch;
    public RevTouchSensor touchSensor;

    List<Integer> screamIds = new ArrayList<>();//screaming robot (don't put in notebook)

    ElapsedTime screamTime = new ElapsedTime(); //makes the robot scream (don't put in notebook)

    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private BNO055IMU imu;

    List<LynxModule> allHubs;

    LinearOpMode opMode;
    HardwareMap hardwareMap;

    boolean isLiftAtStartingPos;

    private boolean runIntakeUntilStone = false;

    private DcMotor sideEncoder;
    private DcMotor frontEncoder;

    public DriveTrain6547Offseason(LinearOpMode _opMode) {
        super(_opMode.hardwareMap);

        opMode = _opMode;
        hardwareMap = opMode.hardwareMap;

      //  LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BNO055IMU.class, "imu"); //get gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            if (RUN_USING_ENCODER) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            //setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
        setLocalizer(new StandardTwowheelLocalizerOffseason(hardwareMap,this)); //two wheel odometry
        //setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap)); //three wheel odometry

        initOtherHardware();

    }
    public void initOtherHardware()
    {
        limitSwitch = hardwareMap.get(AnalogInput.class, "limitSwitch");
        touchSensor = hardwareMap.get(RevTouchSensor.class, "ts2");

        lift =  hardwareMap.get(DcMotorEx.class, "lift");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        fondationGrabber = hardwareMap.get(Servo.class, "f grabber");
        fondationGrabber2 = hardwareMap.get(Servo.class, "f grabber1");
        grabberSlide = hardwareMap.get(CRServo.class, "grabberSlide");
        measuringTape = hardwareMap.get(CRServo.class, "mtape");
        grabber = hardwareMap.get(Servo.class, "grabber");

        colorSensorSideRight = hardwareMap.get(ColorSensor.class, "color sensor"); //set color sensors
        colorSensorSideLeft = hardwareMap.get(ColorSensor.class, "color sensor2");
        intakeColorSensor = hardwareMap.get(RevColorSensorV3.class, "intake color sensor");
        endColorSensor = hardwareMap.get(ColorSensor.class,"end color sensor");
        rightEndColorSensor = hardwareMap.get(ColorSensor.class, "rightEndColorSensor");

        lights = hardwareMap.get(RevBlinkinLedDriver.class,"lights");

        allHubs = hardwareMap.getAll(LynxModule.class);

        measuringTape.setDirection(DcMotorSimple.Direction.REVERSE);

        RobotLog.d("Initialized hardware");

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RobotLog.d("set  lift to brake");

        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        RobotLog.d("set hardware");

        setFondationGrabber(0); //open foundation grabber

        openGrabber();

        RobotLog.d("Initialized IMU");

        initGamepads();

        RobotLog.d("Initialized gamepads");

        setGrabber(0); //open stone grabber

        //load screaming sounds into the bot for reasons
        screamIds.add(hardwareMap.appContext.getResources().getIdentifier("scream1",   "raw", hardwareMap.appContext.getPackageName()));
        screamIds.add(hardwareMap.appContext.getResources().getIdentifier("scream2",   "raw", hardwareMap.appContext.getPackageName()));
        screamIds.add(hardwareMap.appContext.getResources().getIdentifier("scream3",   "raw", hardwareMap.appContext.getPackageName()));
        screamIds.add(hardwareMap.appContext.getResources().getIdentifier("scream4",   "raw", hardwareMap.appContext.getPackageName()));
        screamIds.add(hardwareMap.appContext.getResources().getIdentifier("scream5",   "raw", hardwareMap.appContext.getPackageName()));
        screamIds.add(hardwareMap.appContext.getResources().getIdentifier("scream6",   "raw", hardwareMap.appContext.getPackageName()));
        screamTime.reset();

        /*
        read files set in calibration
         */
        grabberMin = readFile(GRABBER_MIN_FILE_NAME);
        grabberMax = readFile(GRABBER_MAX_FILE_NAME);

        liftMax = (int) readFile(LIFT_MAX_FILE_NAME);
        liftStartingPos = (int) readFile(LIFT_STARTING_POS_FILE_NAME);

        //display files to user
        opMode.telemetry.log().add("Lift Auton Starting Pos: " + liftStartingPos + ", Lift Max: " + liftMax);

        setLiftTargetPos(liftStartingPos);
        setRunLift(false); //robot will not try to move lift unless otherwise told

        setBulkReadAuto();

        //opMode.telemetry = dashboard.getopMode.telemetry();
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
        a1.input(opMode.gamepad1.a);
        a2.input(opMode.gamepad2.a);
        b1.input(opMode.gamepad1.b);
        b2.input(opMode.gamepad2.b);
        x1.input(opMode.gamepad1.x);
        x2.input(opMode.gamepad2.x);
        y1.input(opMode.gamepad1.y);
        y2.input(opMode.gamepad2.y);
        dpadUp1.input(opMode.gamepad1.dpad_up);
        dpadUp2.input(opMode.gamepad2.dpad_up);
        dpadRight1.input(opMode.gamepad1.dpad_right);
        dpadRight2.input(opMode.gamepad2.dpad_right);
        dpadUp1.input(opMode.gamepad1.dpad_up);
        dpadUp2.input(opMode.gamepad2.dpad_up);
        dpadLeft1.input(opMode.gamepad1.dpad_left);
        dpadLeft2.input(opMode.gamepad2.dpad_left);
        leftBumper1.input(opMode.gamepad1.left_bumper);
        leftBumper2.input(opMode.gamepad2.left_bumper);
        rightBumper1.input(opMode.gamepad1.right_bumper);
        rightBumper2.input(opMode.gamepad2.right_bumper);
        leftTrigger1.input(opMode.gamepad1.left_trigger>=.7);
        leftTrigger2.input(opMode.gamepad2.left_trigger>=.7);
        rightTrigger1.input(opMode.gamepad1.right_trigger>=.7);
        rightTrigger2.input(opMode.gamepad2.right_trigger>=.7);

    }
    public void setBulkReadAuto()
    {
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
    public void setBulkReadManual()
    {
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }
    public void clearBulkReadCache()
    {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
    }
    public void setBulkReadOff()
    {
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
        }
    }
    public void writeFile(String filename, double number)
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
    public void saveRobotPos()
    {
        //Not using toString because that rounds the pos to 3 decimal places.  I want more decimal places because I want to
        Pose2d currentPos = getPoseEstimate();
        double x = currentPos.getX();
        double y = currentPos.getY();
        double angleRAD = currentPos.getHeading();
        StringBuilder stringBuilder = new StringBuilder();
        String pos = stringBuilder.append(x).append(",").append(y).append(",").append(angleRAD).toString();
        writeFile(ROBOT_POS_FILE_NAME,pos);
    }
    public Pose2d readRobotPos()
    {
        String posString = readFileString(ROBOT_POS_FILE_NAME);
        String[] numStrings = posString.split(",");

        double x = Double.valueOf(numStrings[0]);
        double y = Double.valueOf(numStrings[1]);
        double angleRAD = Double.valueOf(numStrings[2]);

        return new Pose2d(x,y,angleRAD);
    }
    public void outputTelemetry() //used for debugging.  This method is called a lot
    {
        opMode.telemetry.addData("IMU angle with zero Value", getIMUAngle());
        opMode.telemetry.addData("target pos", getLiftTargetPos());
        opMode.telemetry.addData("lift pos", lift.getCurrentPosition());
        opMode.telemetry.update();
    }
    public void setLiftPower(double pow)
    {
        lift.setPower(pow);
    }
    public void intake(double pow)
    {
        intake.setPower(-pow);
        RobotLog.d("intaking");
    }
    public void outtake(double pow)
    {
        intake.setPower(pow);
        RobotLog.d("outtaking");
    }
    public void stopIntake()
    {
        intake.setPower(0);
        setRunIntakeUntilStone(false);
        RobotLog.d("stopped intake");
    }
    public void setGrabber(double pos)
    {
        double min=.25;
        double max=.45;
        double range = max-min;
        pos*=range;
        pos+=min;
        grabber.setPosition(pos);
    }
    public void openGrabber()
    {
        setGrabber(1);
    }
    public void closeGrabber()
    {
        setGrabber(0);
    }
//    public void setGrabberSlider(double pos)
//    {
//        double min=grabberMin;
//        double max=grabberMax;
//        double range = max-min;
//        pos*=range;
//        pos+=min;
//        grabberSlide.setPosition(pos);
//    }
    public void ExtendGrabberSlide() { grabberSlide.setPower(1); }
    public void RetractGrabberSlide()
    {
        grabberSlide.setPower(-1);
    }
    public void stopGrabberSlide() { grabberSlide.setPower(0);}
    public void extendMeasuingTape() { measuringTape.setPower(1);}
    public void retractMeasuringTape() {measuringTape.setPower(-1);}
    public void stopMeasuringTape() {measuringTape.setPower(0);}
    public void moveGrabberSlideForTime(long milliseconds) {moveGrabberSlideForTime(1,milliseconds);}
    public void moveGrabberSlideForTime(double power, long milliseconds)
    {
        grabberSlide.setPower(power);
        grabberSlideTime.reset();
        grabberSlideTargetTime = milliseconds;
    }

    //updates servo for use with a gamepad stick
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
    public boolean isTouchSensorPressed()
    {
        return touchSensor.isPressed();
    }
    //set lift level
    public void setLiftLevel(int level)
    {
        setLiftLevel(level, 1);
    }
    public void setLiftLevel(int level, double power) //sets the lift based on the level the user stated
    {

        double target = levels[level];
        opMode.telemetry.log().add("target: " + target );
        opMode.telemetry.log().add("current pos: " + lift.getCurrentPosition());
        if (lift.getCurrentPosition() > target)
        {
            opMode.telemetry.log().add("going down");
            power=-Math.abs(power);
            lift.setPower(power);
            while (lift.getCurrentPosition() > target && ((LinearOpMode) opMode).opModeIsActive())
            {
                opMode.telemetry.log().add("Status","Moving Lift Down");
                opMode.telemetry.addData("lift power", lift.getPower());
                opMode.telemetry.addData("lift pos", lift.getCurrentPosition());
                outputTelemetry();
            }
            lift.setPower(0);
        }
        else
        {
            opMode.telemetry.log().add("going up");
            opMode.telemetry.log().add("target: " + target);
            power=Math.abs(power);
            lift.setPower(power);
            while (((LinearOpMode) opMode).opModeIsActive() && lift.getCurrentPosition() < target)
            {
                opMode.telemetry.addData("Status","Moving Up");
                opMode.telemetry.addData("lift power", lift.getPower());
                opMode.telemetry.addData("lift pos", lift.getCurrentPosition());
                outputTelemetry();
            }
            lift.setPower(0);
        }


    }

    @Override
    public void runAtAllTimes() //anything in here runs at all times during auton because this method is ran during roadRunner's state machine
    {
        if (runLift) setLiftToTargetPos(getLiftTargetPos(), getLiftLeeway());
        if (runIntakeUntilStone) runIntakeUntilStone();
        if (grabberSlideTime.milliseconds() > grabberSlideTargetTime) grabberSlide.setPower(0);
        opMode.telemetry.addData("Left END RED: ",endColorSensor.red());
        opMode.telemetry.addData("Right END RED", rightEndColorSensor.red());
        opMode.telemetry.update();
        //outputTelemetry();
    }
    public void runIntakeUntilStone()
    {
        runIntakeUntilStone(1);
    }
    public void runIntakeUntilStone(double power)
    {
        intake(power);
        if (isStoneAtIntake())
        {
            //If a stone is under the bot, go slower so it reaches the end
            RobotLog.d("Stone at intake of robot");
            RobotLog.d("Intake Color Sensor R:" + intakeColorSensor.red() + ", G:" + intakeColorSensor.green() + ", B:" + intakeColorSensor.blue() +", A:" + intakeColorSensor.alpha());
            stopIntake();
            return;
        }
        if (false && isStoneAtEnd())
        {
            //stone at end.
            RobotLog.d("Stone at end of robot");
            RobotLog.d("Left Color Sensor R: " + endColorSensor.red() + ", G: " + endColorSensor.green() +", B: " + endColorSensor.blue() + ", a: " + endColorSensor.alpha());
            RobotLog.d("Right Color Sensor R: " + rightEndColorSensor.red() + ", G: " + rightEndColorSensor.green() +", B: " + rightEndColorSensor.blue() + ", a: " + rightEndColorSensor.alpha());
            stopIntake();
            return;
            //closeGrabber();
        }

        intake(power);
    }
    public void grabStoneInIntake()
    {
        runtime.reset();
        while (!isStoneAtEnd() && opMode.opModeIsActive() && runtime.seconds() < .5)
        {
            intake(.5);
        }
        opMode.sleep(500);
        closeGrabber();
    }
    public boolean isLiftAtTargetPos()
    {
       return isLiftAtStartingPos;
    }

    public void setLiftToTargetPos(int targetPos, int leaway) //sets lift to target pos.
    {
        // No while loops in this so it can be used in tele-op or a state machine
        int liftPos = lift.getCurrentPosition();
        if (liftPos > targetPos + leaway)
        {
            lift.setPower(-0.5);
            isLiftAtStartingPos = false;
        }
        else if (liftPos < targetPos-leaway)
        {
            lift.setPower(0.5);
            isLiftAtStartingPos = false;
        }
        else
        {
            lift.setPower(0);
            isLiftAtStartingPos = true;
        }

    }
    public void moveLift(int modifer, int leaway)
    {
        RobotLog.d("Moving Lift to " + modifer + " from " + lift.getCurrentPosition());
        isLiftAtStartingPos = false;
        runtime.reset();
        setLiftTargetPos(getLiftStartingPos() + modifer);
        while (!isLiftAtTargetPos() && opMode.opModeIsActive()) {
            setLiftToTargetPos(getLiftTargetPos(), leaway);
            outputTelemetry();
        }
    }
    public void setFondationGrabber(double pos) //take a pos from 0-1 and convert it into the mins and max areas of the servo
    {
        double min=.2;
        double min2=.2;
        double max=.9;
        double max2 = .92;
        double range = max-min;
        double range2 = max2-min2;
        double pos2 = (pos*range2) +min;
        pos*=range;
        pos+=min;
        fondationGrabber2.setPosition(pos2);
        fondationGrabber.setPosition(Math.abs(1-pos));
        opMode.telemetry.log().add("set foundation grabber to "  + pos);

    }
    public double getFoundationGrabber()
    {
        double min=0;
        double max=.55;
        double max2=.65;
        double range = max-min;
        return (fondationGrabber2.getPosition()*range) + min;
    }
    public double getIMUAngle() //gets the gyro angle in degrees
    {
        double currentAngle=Math.toDegrees(getRawExternalHeading())+angleZzeroValue;
        while (currentAngle>180) currentAngle-=360;
        while (currentAngle<-180) currentAngle+=360;
        return currentAngle;
    }
    public double getIMUAngle(boolean extendBeyond180)
    {
        double currentAngle=Math.toDegrees(getRawExternalHeading())+angleZzeroValue;
        if (extendBeyond180) return currentAngle;
        else return getIMUAngle();
    }
    public boolean isSkystone(ColorSensor colorSensorToBeUsed) //checks if color sensor sees a skystone
    {
        return (colorSensorToBeUsed.red()*colorSensorToBeUsed.green()) / Math.pow(colorSensorToBeUsed.blue(),2) < 3;
    }
    public boolean isStoneAtEnd()
    {
        if (isStone(rightEndColorSensor) || isStone(endColorSensor))
        {
            return true;
        }
        else return false;
    }
    public boolean isStone(ColorSensor colorSensor)
    {
        return colorSensor.red()>200; //&& colorSensor.red()<255;
    }
    public boolean isStoneAtIntake()
    {
        return intakeColorSensor.alpha() > 1000;
    }
    public boolean isLimitSwitchPressed()
    {
        return limitSwitch.getVoltage() > 2.5;
    }
    public void zeroEncoders() //set all the encoders to zero
    {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //zero encoders
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //zero encoders
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //zero encoders
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //zero encoders
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void disableEncoders() //turn all encoders off
    {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //zero encoders
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //zero encoders
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //zero encoders
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //zero encoders
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public DcMotor zeroEncoder(DcMotor motor)
    {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return motor;
    }
    public void stopRobot()
    {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
    public void scream()
    {
        if (screamTime.seconds() >= 2)
        {
            int soundToPlay = (int) (Math.random() * screamIds.size());

            if (screamIds.get(soundToPlay) != 0)
            {
                SoundPlayer.getInstance().preload(hardwareMap.appContext, screamIds.get(soundToPlay));
            }
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, screamIds.get(soundToPlay));
            screamTime.reset();
        }
        else
        {
            opMode.telemetry.log().add("out of time");
        }
    }
    //--------------------------------------------------------------
    // Get Set Methods
    //--------------------------------------------------------------

    public double getAngleZzeroValue() {
        return angleZzeroValue;
    }

    public void setAngleZzeroValue(double angleZzeroValue) {
        DriveTrain6547Offseason.angleZzeroValue = angleZzeroValue;
    }

    public int getLiftLeeway() {
        return liftLeeway;
    }

    public void setLiftLeeway(int liftLeeway) {
        this.liftLeeway = liftLeeway;
    }

    public int getLiftStartingPos() {
        return liftStartingPos;
    }

    public ElapsedTime getRuntime() {
        return runtime;
    }

    public void setRunIntakeUntilStone(boolean val)
    {
        runIntakeUntilStone = val;
    }
    public boolean getRunIntakeUntilStone()
    {
        return runIntakeUntilStone;
    }
    //--------------------------------------------------------------
    //  Road Runner
    //--------------------------------------------------------------

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftFront.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, getMotorVelocityF()
            ));
        }
    }

    @
            NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double LF, double LR, double RR, double RF) {
        leftFront.setPower(LF);
        leftRear.setPower(LR);
        rightRear.setPower(RR);
        rightFront.setPower(RF);
    }
    //set motor powers but it's field realtive.  Unused
    public void setMotorPowers(double v,double v1, double v2, double v3, double angle)
    {
        angle-=getIMUAngle();
        angle+=45;
        angle = Math.toRadians(angle);
        double normalize = 1/Math.cos(Math.toRadians(45));
        v*=Math.cos(angle)*normalize;
        v1*=Math.cos(angle)*normalize;
        v2*=Math.sin(angle)*normalize;
        v3*=Math.sin(angle)*normalize;
        setMotorPowers(v,v1,v2,v3);
    }
    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    /*
    test road runner methods, currently not used
     */
    public void driveForward(double inches)
    {
        followTrajectorySync(trajectoryBuilder()
        .forward(inches)
        .build());
    }
    public void driveBackward(double inches)
    {
        followTrajectorySync(trajectoryBuilder()
        .back(inches)
        .build());
    }
    public void strafeLeft(double inches)
    {
        followTrajectorySync(trajectoryBuilder()
        .strafeLeft(inches)
        .build());
    }
    public void strafeRight(double inches)
    {
        followTrajectorySync(trajectoryBuilder()
        .strafeRight(inches)
        .build());
    }
    /*
    Road Runner turn with consideration of the gyro angle
     */
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
    public void turnRealtive(double angle)
    {
        double target=angle-Math.toRadians(getIMUAngle());
        target-=Math.toRadians(90);
        if (Math.abs(target)>Math.toRadians(180)) //make the angle difference less then 180 to remove unnecessary turning
        {
            target+=(target>=0) ? Math.toRadians(-360) : Math.toRadians(360);
        }
        opMode.telemetry.log().add("inputted Angle: " + angle + " , turning to: " + target);
        turn(target);
    }
    public void updateRobotPosRoadRunner()
    {
        updatePoseEstimate();

        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading", currentPose.getHeading());

        packet.put("xError", lastError.getX());
        packet.put("yError", lastError.getY());
        packet.put("headingError", lastError.getHeading());

        fieldOverlay.setStroke("#F44336");
        DashboardUtil.drawRobot(fieldOverlay, currentPose);

        dashboard.sendTelemetryPacket(packet);

    }
    public void turnLeft(double power) {
        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftRear.setPower(power);
        rightRear.setPower(-power);
    }
}
