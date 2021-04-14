package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drivetrain.Bot2;
import org.firstinspires.ftc.teamcode.drivetrain.DriveSpeeds;
import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense;
import org.firstinspires.ftc.teamcode.teleOp.Bot2TeleOp;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.PID.TuningController;
import org.firstinspires.ftc.teamcode.util.PID.VelocityPIDFController;
import org.firstinspires.ftc.teamcode.util.ThrowerUtil;
import org.firstinspires.ftc.teamcode.util.homar.Button;
import org.firstinspires.ftc.teamcode.util.homar.ToggleBoolean;

@Config
@TeleOp(group = "_teleOp")
public class TeleOpThrowerPID extends LinearOpMode {

    public static double TIME_TO_CONVEY = .25;
    // Copy your PID Coefficients here
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0.001, 0, 0.0001);


    // Copy your feedforward gains here
    public static double kV = .0005; //1 / TuningController.rpmToTicksPerSecond(TuningController.MOTOR_MAX_RPM);
    public static double kA = 0.0005;
    public static double kStatic = 0;

    public static double targetTicksPerSec=1500;
    public static double leeway=28;

    // Timer for calculating desired acceleration
    // Necessary for kA to have an affect
    private final ElapsedTime veloTimer = new ElapsedTime();
    private double lastTargetVelo = 0.0;

    // Our velocity controller
    private final VelocityPIDFController veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);

    public DcMotorEx conveyor;
    public DcMotorEx intake;

    DcMotorEx myMotor1;
    DcMotorEx myMotor2;

    Bot2 bot;
    Bot2TeleOp bot2TeleOp;

    public ToggleBoolean turnOnThrower = new ToggleBoolean(false);

    public ElapsedTime conveyTime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {

        bot = new Bot2(this);
        bot2TeleOp = new Bot2TeleOp(this, bot);

        // SETUP MOTORS //
        // Change my id
        myMotor1 = hardwareMap.get(DcMotorEx.class, "thrower");
        myMotor2 = hardwareMap.get(DcMotorEx.class, "thrower2");

//        // Reverse as appropriate
         myMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
         myMotor2.setDirection(DcMotorSimple.Direction.REVERSE);


        // Ensure that RUN_USING_ENCODER is not set
        myMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Turns on bulk reading
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Insert whatever other initialization stuff you do here

        telemetry.log().add("Ready to Start");
        waitForStart();

        if (isStopRequested()) return;

        // Start the veloTimer
        veloTimer.reset();

        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Button rightBumpeer2 = new Button();

        while (!isStopRequested()) {
            ///// Run the velocity controller ////

            Pose2d robotPos = bot.getPoseEstimate();
            bot.updateGamepads();
            rightBumpeer2.input(gamepad2.right_bumper);

            if (bot.rightBumper2.onPress()) {
                turnOnThrower.toggle();
                conveyTime.reset();
                telemetry.log().add("thing pressed");
            }

            if (bot.isRingAtConveyor() && bot.intake.getPower() >=0) {
                bot.loadRingInConveyor();
            } else if (!bot.isLaunching()) {
                bot.stopConveyor();
            }

            if (bot.a2.isPressed()) {
                if (isReadyToThrow()) bot.launchRing();
                else bot.stopConveyor();
            } else if (bot.a2.onRelease() && !bot.isRingAtConveyor()){
                bot.stopLaunch();
            }

            if (turnOnThrower.output() && conveyTime.seconds() > TIME_TO_CONVEY) {
                updateThrower(targetTicksPerSec);
                if (!bot.isLaunching() && !bot.isRingAtConveyor()) bot.stopConveyor();
            } else if (turnOnThrower.output() && conveyTime.seconds() < TIME_TO_CONVEY){
                bot.conveyor.setPower(-1);
                updateThrower(0);
            } else {
                updateThrower(0);
            }

            //updateThrower(targetTicksPerSec);

//            if (!turnOnThrower.output() && !bot.isLaunching() && bot.isRingAtConveyor()) {
//                bot.loadRingInConveyor();
//            } else if (!turnOnThrower.output() && !bot.isLaunching() && !bot.isRingAtConveyor()) {
//                bot.stopConveyor();
//                //bot.stopIntake();
//            }
            if (bot.dpadDown1.onPress()) {
                doRegularShots(robotPos);
                bot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            }
            bot2TeleOp.doTeleOp();
            bot.update();
            // Target velocity in ticks per second

            // Update the controller and set the power for each motor
//            if (targetVelo != 0 && bot2TeleOp.conveyTime.seconds() > Bot2TeleOp.TIME_TO_CONVEY) {
//                double power = -veloController.update(motorPos, motorVelo);
//                myMotor1.setPower(power);
//                myMotor2.setPower(power);
//                if (bot.conveyor.getPower() < 0) bot.stopConveyor();
//            } else {
//                myMotor1.setPower(0);
//                myMotor2.setPower(0);
//            }

//            if (bot2TeleOp.conveyTime.seconds() > Bot2TeleOp.TIME_TO_CONVEY) {
//                if (bot.conveyor.getPower() < 0) {
//                    bot.stopConveyor();
//                    bot.stopIntake();
//                }
//                double power = -veloController.update(motorPos, motorVelo);
//                myMotor1.setPower(power);
//                myMotor2.setPower(power);
//                bot.updateLightsBasedOnThrower();
//            } else {
//                bot.conveyor.setPower(-1);
//                bot.outtake();
//            }



//            if (gamepad1.a && isReadyToThrow()) {
//                intake();
//                forwardConveyBelt();
//            } else if (gamepad1.b) {
//                outtake();
//                backConveyVelt();
//            } else {
//                stopIntake();
//                stopConveyVelt();
//            }

            telemetry.addData("sees Ring", bot.isRingAtConveyor());
            telemetry.addData("is thrower on", turnOnThrower.output());
            telemetry.addData("IS READY TO THROW", isReadyToThrow());
            telemetry.addData("RPM", myMotor1.getVelocity(AngleUnit.DEGREES)/360);
            telemetry.addData("Target Velocity", targetTicksPerSec);
            telemetry.addData("Thrower Velocity 1", myMotor1.getVelocity());
            telemetry.addData("Thrower Velocity 2", myMotor2.getVelocity());
            telemetry.addData("current pos:", myMotor1.getCurrentPosition());
            telemetry.addData("upperBound", TuningController.rpmToTicksPerSecond(TuningController.TESTING_MAX_SPEED * 1.15));
            telemetry.addData("lowerBound", 0);
            telemetry.update();

            // Do your opmode stuff
        }
    }

    public void updateThrower(double targetVelo) {

        bot.setTargetVelocity(targetVelo);
        // Call necessary controller methods
        veloController.setTargetVelocity(targetVelo);
        veloController.setTargetAcceleration((targetVelo - lastTargetVelo) / veloTimer.seconds());
        veloTimer.reset();

        lastTargetVelo = targetVelo;

        // Get the velocity from the motor with the encoder
        if (targetVelo != 0) {
            double motorPos = -myMotor1.getCurrentPosition();
            double motorVelo = -myMotor1.getVelocity();

            double power = -veloController.update(motorPos, motorVelo);
            myMotor1.setPower(power);
            myMotor2.setPower(power);
        } else { //target Velo is 0
            myMotor1.setPower(0);
            myMotor2.setPower(0);
        }


    }

    public void doRegularShots() {
        doRegularShots(bot.getPoseEstimate());
    }
    public void doRegularShots(Pose2d robotPos) {
        bot.stopIntake();
        double angleToTurnTo = bot.turnTowardsAngle(new Vector2d(FieldConstants.RED_GOAL_X, FieldConstants.RED_GOAL_Y), robotPos);
        //bot.setThrowerVelocity(bot.getThrowerVelocityFromPosition(new Pose2d(robotPos.getX(), robotPos.getY(), angleToTurnTo)) * ThrowerUtil.GOAL_CONSTANT, AngleUnit.DEGREES);

        bot.turnRelative(angleToTurnTo);
        while (bot.mode!= DriveTrain6547Realsense.Mode.IDLE) {
            bot.update();
        }
        bot.turnRelative(angleToTurnTo);
        while (bot.mode!= DriveTrain6547Realsense.Mode.IDLE) {
            bot.update();
        }
        //bot.setThrowerVelocity(bot.getThrowerVelocityFromPosition(currentPos, AngleUnit.DEGREES) * VELO_MUTIPLIER, AngleUnit.DEGREES);

        RobotLog.v("STARTING TO DO REGLUAR SHOTS");
        for (int i = 0; i < 3 && !bot.isStickMoved(gamepad1.left_stick_x, gamepad1.left_stick_y); i++) {
            while (!isReadyToThrow() && !bot.isStickMoved(gamepad1.left_stick_x, gamepad1.left_stick_y)) {
                bot.updateThrower();
                bot.updateLightsBasedOnThrower();
            }
            bot.launchRing();
            RobotLog.v("Thrower motor 0 VELO (when launched): " + (bot.getThrowerVelocity()[0] / 360) + "REV/s");
            if (!bot.isStickMoved(gamepad1.left_stick_x, gamepad1.left_stick_y)) sleep(100);
        }

        updateThrower(0);
        turnOnThrower.input(false);
    }

    public void intake() {
        intake.setPower(1);
    }
    public void outtake() {
        intake.setPower(-1);
    }
    public void stopIntake() {
        intake.setPower(0);
    }
    public void forwardConveyBelt() {
        conveyor.setPower(1);
        //intake();
    }
    public void backConveyVelt() {
        conveyor.setPower(-1);
    }
    public void stopConveyVelt() {
        conveyor.setPower(0);
    }

    public boolean isReadyToThrow() {
        double[] velocities = new double[] {Math.abs(myMotor1.getVelocity()), Math.abs(myMotor2.getVelocity())};
        double minVelo = Math.abs(targetTicksPerSec) - leeway;
        double maxVelo = Math.abs(targetTicksPerSec) + leeway;
        boolean isMotor0AtTarget = velocities[0] > minVelo && velocities[0] < maxVelo;
        boolean isMotor1AtTarget = velocities[1] > minVelo && velocities[1] < maxVelo;

        return  isMotor0AtTarget || isMotor1AtTarget;
    }

    public double ticksToRev(double ticks, DcMotorEx motor) {
        return ticks/28;
    }
}
