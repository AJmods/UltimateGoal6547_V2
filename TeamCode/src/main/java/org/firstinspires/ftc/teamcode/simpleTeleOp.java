package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class simpleTeleOp extends LinearOpMode {

    DcMotor leftFront;
    DcMotor rightRear;
    DcMotor rightFront;
    DcMotor leftRear;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        waitForStart();

        while (opModeIsActive()) {
            double ly = gamepad1.left_stick_y;
            double ry = gamepad1.right_stick_y;
            leftFront.setPower(ly);
            leftRear.setPower(ly);
            rightFront.setPower(-ry);
            rightRear.setPower(-ry);
        }
    }
}
