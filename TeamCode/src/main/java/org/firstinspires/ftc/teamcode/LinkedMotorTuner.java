package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drivetrain.Bot2;
import org.firstinspires.ftc.teamcode.util.PID.TuningController;
import org.firstinspires.ftc.teamcode.util.PID.VelocityPIDFController;

/**
 * Program for Tuning Thrower.
 */
@Config
@TeleOp
public class LinkedMotorTuner extends LinearOpMode {
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0,0,0);

    // Copy your feedforward gains here
    public static double kV = 0; //1 / TuningController.rpmToTicksPerSecond(TuningController.MOTOR_MAX_RPM);
    public static double kA = 0;
    public static double kStatic = 0;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private final ElapsedTime veloTimer = new ElapsedTime();

    DcMotorEx conveyor;
    DcMotorEx intake;

    @Override
    public void runOpMode() throws InterruptedException {
        // Change my id
        DcMotorEx myMotor1 = hardwareMap.get(DcMotorEx.class, "thrower");
        DcMotorEx myMotor2 = hardwareMap.get(DcMotorEx.class, "thrower2");

        conveyor = hardwareMap.get(DcMotorEx.class, "conveyor");
        conveyor.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        // Reverse as appropriate
//        myMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
//        myMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        myMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        VelocityPIDFController veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);
        TuningController tuningController = new TuningController();

        double lastTargetVelo = 0.0;
        double lastKv = kV;
        double lastKa = kA;
        double lastKstatic = kStatic;

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        tuningController.start();
        veloTimer.reset();

        while (!isStopRequested() && opModeIsActive()) {
            double targetVelo = tuningController.update();

            veloController.setTargetVelocity(targetVelo);
            veloController.setTargetAcceleration((targetVelo - lastTargetVelo) / veloTimer.seconds());
            veloTimer.reset();

            lastTargetVelo = targetVelo;

            telemetry.addData("targetVelocity", targetVelo);

            double motorPos = myMotor1.getCurrentPosition();
            double motorVelo = myMotor1.getVelocity();

            double power = veloController.update(motorPos, motorVelo);
            myMotor1.setPower(power);
            myMotor2.setPower(power);

            if(lastKv != kV || lastKa != kA || lastKstatic != kStatic) {
                lastKv = kV;
                lastKa = kA;
                lastKstatic = kStatic;

                veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);
            }

            if (gamepad1.a) {
                intake();
                forwardConveyBelt();
            } else if (gamepad1.b) {
                outtake();
                backConveyVelt();
            } else {
                stopIntake();
                stopConveyVelt();
            }



            telemetry.addData("RPM", myMotor1.getVelocity(AngleUnit.DEGREES)/360);
            telemetry.addData("velocity", motorVelo);
            telemetry.addData("motorTwo velo", myMotor2.getVelocity());
            telemetry.addData("error", targetVelo - motorVelo);

            telemetry.addData("upperBound", TuningController.rpmToTicksPerSecond(TuningController.TESTING_MAX_SPEED * 1.15));
            telemetry.addData("lowerBound", 0);
            telemetry.update();
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
}
