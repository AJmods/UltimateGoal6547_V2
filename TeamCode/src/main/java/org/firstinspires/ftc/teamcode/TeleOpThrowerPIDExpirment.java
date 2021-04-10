package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

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

import static org.firstinspires.ftc.teamcode.testing.throwing.ThrowerExperiment.TARGET_HEIGHT;

@Config
@TeleOp(group = "_teleOp")
public class TeleOpThrowerPIDExpirment extends LinearOpMode {

    public static boolean move = false;
    public static double X=0;
    public static double Y=0;
    public static double Heading=0;
    public static double TIME_TO_CONVEY = .25;
    // Copy your PID Coefficients here
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0.001, 0, 0.0001);


    // Copy your feedforward gains here
    public static double kV = .0005; //1 / TuningController.rpmToTicksPerSecond(TuningController.MOTOR_MAX_RPM);
    public static double kA = 0.0005;
    public static double kStatic = 0;

    public static double targetTicksPerSec=1400;
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

            Pose2d pos = bot.getPoseEstimate();
            bot.updateGamepads();
            rightBumpeer2.input(gamepad2.right_bumper);

            if (rightBumpeer2.onPress()) {
                turnOnThrower.toggle();
                conveyTime.reset();
                telemetry.log().add("thing pressed");
            }

            if (gamepad1.a && isReadyToThrow()) {
                bot.conveyor.setPower(1);
                bot.intake();
            } else {
                bot.conveyor.setPower(0);
                bot.stopIntake();
            }

            updateThrower(targetTicksPerSec);

            //updateThrower(targetTicksPerSec);

            if (bot.mode == DriveTrain6547Realsense.Mode.IDLE && move) {
                try {
                    bot.followTrajectory(bot.trajectoryBuilder(false, DriveSpeeds.slow).lineToLinearHeading(new Pose2d(X, Y, Heading)).build());
                } catch (Exception e){

                }
            }

            boolean isValidAngle = ThrowerUtil.isValidAngle(pos.getX(), pos.getY(), pos.getHeading());

            double targetY = ThrowerUtil.getTargetY(pos, FieldConstants.RED_GOAL_X);
            double deltaX = pos.getX() - FieldConstants.RED_GOAL_X;
            double deltaY = pos.getY() - targetY;
            double dist = Math.hypot(deltaX, deltaY);
            double vi = ThrowerUtil.getVi(0, ThrowerUtil.INITIAL_HEIGHT, dist, TARGET_HEIGHT, ThrowerUtil.INITIAL_ANGLE);
            double targetRev = vi/ ThrowerUtil.inchesPerRev;

            bot.setPacketAction((packet, fieldOverlay) -> {

                packet.addLine("DISTANCE: " + dist);
                packet.addLine("TARGET ticks/S: " + targetTicksPerSec);

                //add data.
                packet.addLine("Theoretical Ring Velocity: " + targetRev + " ticks/s");

                packet.addLine("INITIAL ANGLE: " + ThrowerUtil.INITIAL_ANGLE);
                packet.addLine("INITIAL HEIGHT: " + ThrowerUtil.INITIAL_HEIGHT);
                //draw robot
                //  DashboardUtil.drawRobot(fieldOverlay, pos);
                //draw launcherPos
                //draw line in the direction of where the robot is facing
                //DashboardUtil.drawLine(fieldOverlay, pos);

                //draw target robot angle range
                fieldOverlay.setStroke("#000000");
                fieldOverlay.strokeLine(pos.getX(), pos.getY(), FieldConstants.RED_GOAL_X, ThrowerUtil.MIN_Y);
                fieldOverlay.strokeLine(pos.getX(), pos.getY(), FieldConstants.RED_GOAL_X, ThrowerUtil.MAX_Y);

                //mark middle launch goal with circle
                fieldOverlay.strokeCircle(FieldConstants.RED_GOAL_X, targetY, 1);

                //draw robot launch goal position
                fieldOverlay.setStrokeWidth(3);
                fieldOverlay.setStroke("FF0000");
                fieldOverlay.strokeLine(FieldConstants.RED_GOAL_X, ThrowerUtil.MIN_Y, FieldConstants.RED_GOAL_X, ThrowerUtil.MAX_Y);

            });

            telemetry.addData("Target VELO: ", targetTicksPerSec);
            telemetry.addData("CURRENT THROWER 0 VELO:", bot.getThrowerVelocity(AngleUnit.DEGREES)[0]/360);
            telemetry.addData("CURRENT THROWER 1 VELO: ", bot.getThrowerVelocity(AngleUnit.DEGREES)[1]/360);
            telemetry.addData("IS LAUNCHED: ", bot.isReadyToThrow());
            telemetry.addData("sees Ring", bot.isRingAtConveyor());
            telemetry.addData("is thrower on", turnOnThrower.output());
            telemetry.addData("IS READY TO THROW", isReadyToThrow());
            telemetry.update();

            bot.update(); //updates robot's position
            bot.updateLightsBasedOnThrower();
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
