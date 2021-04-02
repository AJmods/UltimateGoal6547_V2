package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain6547Realsense;
import org.firstinspires.ftc.teamcode.util.PID.TuningController;
import org.firstinspires.ftc.teamcode.util.PID.VelocityPIDFController;

@Config
@TeleOp
public class SampleLinkedPIDUse extends LinearOpMode {
    // Copy your PID Coefficients here
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0.003, 0, 0.000000);


    // Copy your feedforward gains here
    public static double kV = 0.00051; //1 / TuningController.rpmToTicksPerSecond(TuningController.MOTOR_MAX_RPM);
    public static double kA = 0.0001;
    public static double kStatic = 0;

    public static double targetTicksPerSec=0;
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
    @Override
    public void runOpMode() throws InterruptedException {

        // SETUP MOTORS //
        // Change my id
        myMotor1 = hardwareMap.get(DcMotorEx.class, "thrower");
        myMotor2 = hardwareMap.get(DcMotorEx.class, "thrower2");

//        // Reverse as appropriate
//         myMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
//         myMotor2.setDirection(DcMotorSimple.Direction.REVERSE);


        // Ensure that RUN_USING_ENCODER is not set
        myMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        conveyor = hardwareMap.get(DcMotorEx.class, "conveyor");
        conveyor.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        // Turns on bulk reading
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Insert whatever other initialization stuff you do here

        waitForStart();

        if (isStopRequested()) return;

        // Start the veloTimer
        veloTimer.reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (!isStopRequested()) {
            ///// Run the velocity controller ////

            // Target velocity in ticks per second
            double targetVelo = targetTicksPerSec;

            // Call necessary controller methods
            veloController.setTargetVelocity(targetVelo);
            veloController.setTargetAcceleration((targetVelo - lastTargetVelo) / veloTimer.seconds());
            veloTimer.reset();

            lastTargetVelo = targetVelo;

            // Get the velocity from the motor with the encoder
            double motorPos = myMotor1.getCurrentPosition();
            double motorVelo = myMotor1.getVelocity();

            // Update the controller and set the power for each motor
            double power = veloController.update(motorPos, motorVelo);
            myMotor1.setPower(power);
            myMotor2.setPower(power);

            if (gamepad1.a && isReadyToThrow()) {
                intake();
                forwardConveyBelt();
            } else if (gamepad1.b) {
                outtake();
                backConveyVelt();
            } else {
                stopIntake();
                stopConveyVelt();
            }

            telemetry.addData("IS READY TO THROW", isReadyToThrow());
            telemetry.addData("RPM", myMotor1.getVelocity(AngleUnit.DEGREES)/360);
            telemetry.addData("Thrower Velocity 1", myMotor1.getVelocity());
            telemetry.addData("Thrower Velocity 2", myMotor2.getVelocity());
            telemetry.addData("upperBound", TuningController.rpmToTicksPerSecond(TuningController.TESTING_MAX_SPEED * 1.15));
            telemetry.addData("lowerBound", 0);
            telemetry.update();

            // Do your opmode stuff
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
        double[] velocities = new double[] {myMotor1.getVelocity(), myMotor2.getVelocity()};
        double minVelo = targetTicksPerSec - leeway;
        double maxVelo = targetTicksPerSec + leeway;
        boolean isMotor0AtTarget = velocities[0] > minVelo && velocities[0] < maxVelo;
        boolean isMotor1AtTarget = velocities[1] > minVelo && velocities[1] < maxVelo;

        return  isMotor0AtTarget || isMotor1AtTarget;
    }

    public double ticksToRev(double ticks, DcMotorEx motor) {
        return ticks/28;
    }
}
