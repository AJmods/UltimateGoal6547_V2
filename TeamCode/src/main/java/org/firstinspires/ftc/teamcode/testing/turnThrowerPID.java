package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
@Disabled
public class turnThrowerPID extends LinearOpMode {

    public static double REV_PER_SEC;
    public static double P;
    public static double I;
    public static double D;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    DcMotorEx thrower1;
    DcMotorEx thrower2;

    @Override
    public void runOpMode() throws InterruptedException {
        thrower1 = hardwareMap.get(DcMotorEx.class, "thrower");
        thrower2 = hardwareMap.get(DcMotorEx.class, "thrower2");

        thrower1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        thrower2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.log().add("Ready to start");
        waitForStart();
        telemetry.clearAll();

        while (opModeIsActive()) {

            thrower1.setVelocity(REV_PER_SEC * 360, AngleUnit.DEGREES);
            thrower2.setVelocity(REV_PER_SEC * 360, AngleUnit.DEGREES);

          //  thrower1.setPIDFCoefficients(thrower1.getMode(), new PIDCoefficients());
            telemetry.addData("TARGET VELO (REV/s)", REV_PER_SEC);
            telemetry.addData("Thrower 1 VELO (REV/s)", thrower1.getVelocity(AngleUnit.DEGREES)/360);
            telemetry.addData("Thrower 2 VELO (REV/s)", thrower2.getVelocity(AngleUnit.DEGREES)/360);
            telemetry.update();
        }
    }
//    public double getMotorVelocityF() {
//        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
//        return 32767 * 60 / (6000 * TICKS_PER_REV);
//    }
}
