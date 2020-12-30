package org.firstinspires.ftc.teamcode.testing.throwing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drivetrain.localizer.T265LocalizerRR;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.roadrunner.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.ThrowerUtil;

@TeleOp(name = "Throw Test Realsense")
@Config
@Disabled
public class throwTestRealsense extends LinearOpMode {

    public static boolean USE_CALCULATED_VELOCITY = false;
    public static double MIN_Y = -33;
    public static double MAX_Y = -39;
    public static double REV_PER_SEC = 0;
    public static double INITIAL_ANGLE = 35;
    public static double INITIAL_HEIGHT = 10;
    private final double RED_GOAL_X = FieldConstants.RED_GOAL_X;
    private final double wheelDiameter = 3.77953; //96 mm
    private final double inchesPerRev = Math.PI * wheelDiameter;


    FtcDashboard dashboard = FtcDashboard.getInstance();

    Localizer realSense = new T265LocalizerRR(hardwareMap);

    DcMotorEx thrower1;
    DcMotorEx thrower2;

    @Override
    public void runOpMode() throws InterruptedException {

        thrower1 = hardwareMap.get(DcMotorEx.class, "thrower");
        thrower2 = hardwareMap.get(DcMotorEx.class, "thrower1");

        telemetry.log().add("Ready to start");
        waitForStart();

        while (opModeIsActive()) {

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            Pose2d pos = realSense.getPoseEstimate();

            boolean isValidAngle = isValidAngle(pos.getX(), pos.getY(), pos.getHeading());
            packet.addLine("VALID ANGLE: " + isValidAngle);

            if (isValidAngle) {
                double targetY = getTargetY(pos, RED_GOAL_X);
                double deltaX = pos.getX() - RED_GOAL_X;
                double deltaY = pos.getY() - targetY;
                double dist = Math.hypot(deltaX, deltaY);
                double vi = ThrowerUtil.getVi(0, INITIAL_HEIGHT, dist, FieldConstants.RED_GOAL_HEIGHT, 35);
                double targetRev = vi/inchesPerRev;

                if (USE_CALCULATED_VELOCITY) {
                    thrower1.setVelocity(targetRev*360, AngleUnit.DEGREES);
                    thrower2.setVelocity(targetRev*360, AngleUnit.DEGREES);
                } else {
                    thrower1.setVelocity(REV_PER_SEC * 360, AngleUnit.DEGREES);
                    thrower2.setVelocity(REV_PER_SEC * 360, AngleUnit.DEGREES);
                    double drawsConstant = REV_PER_SEC/targetRev;
                    packet.addLine("Drew's Constant: " + drawsConstant + " (Current Motor Rev/s)/(Target Rev/s)");
                }


                //stroke launcher targetPos
                fieldOverlay.strokeCircle(RED_GOAL_X, targetY, 1);
                //add data.
                packet.addLine("Target Ring Velocity: " + vi + " inches/s, " + targetRev + " rev/s, " + (vi*28) + " ticks/s");
                packet.addLine("Thrower1 velocity: " + (thrower1.getVelocity(AngleUnit.DEGREES)/360*inchesPerRev) + " inches/s, " + (thrower1.getVelocity(AngleUnit.DEGREES)/360) + " rev/s, " + thrower1.getVelocity() + " ticks/s");
                packet.addLine("Thrower2 velocity: " + (thrower2.getVelocity(AngleUnit.DEGREES)/360*inchesPerRev) + " inches/s, " + (thrower2.getVelocity(AngleUnit.DEGREES)/360) + " rev/s, " + thrower2.getVelocity() + " ticks/s");
            } else {
                double minAngle = Math.toDegrees(Math.atan2(MIN_Y - pos.getY(), RED_GOAL_X - pos.getX()));
                double maxAngle = Math.toDegrees(Math.atan2(MAX_Y - pos.getY(), RED_GOAL_X - pos.getX()));
                packet.addLine("INVALID ANGLE! Must be Between " + Math.round(Math.toDegrees(minAngle)) + " and " + Math.round(Math.toDegrees(maxAngle)) + " Degrees.");
            }
            //draw robot
            DashboardUtil.drawRobot(fieldOverlay, pos);
            //draw line in the direction of where the robot is facing
            DashboardUtil.drawLine(fieldOverlay, pos);

            //draw target robot angle range
            fieldOverlay.strokeLine(pos.getX(), pos.getY(), RED_GOAL_X, MIN_Y);
            fieldOverlay.strokeLine(pos.getX(), pos.getY(), RED_GOAL_X, MAX_Y);

            //draw robot launch goal position
            fieldOverlay.strokeLine(RED_GOAL_X, MIN_Y, RED_GOAL_X, MAX_Y);
            dashboard.sendTelemetryPacket(packet);
            realSense.update();
        }

    }
    double getTargetY(Pose2d currentPos, double targetX) {
        double slope = Math.tan(currentPos.getHeading());
        //y = mx + b
        //b = y - mx
        double yIntercept = currentPos.getY() - (currentPos.getX() * slope);

        return (slope * targetX) + yIntercept;
    }
    double getTargetX(Pose2d currentPos, double targetY) {
        double slope = Math.tan(currentPos.getHeading());

        double yIntercept = currentPos.getY() - (currentPos.getX() * slope);
        return (targetY - yIntercept) / slope;
    }
    boolean isValidAngle(double x, double y, double angle) {
       double minAngle = Math.toDegrees(Math.atan2(MIN_Y - y, RED_GOAL_X - x));
       double maxAngle = Math.toDegrees(Math.atan2(MAX_Y - y, RED_GOAL_X - x));

       return (maxAngle > minAngle && minAngle < angle && maxAngle > angle) || (minAngle > maxAngle && maxAngle < angle && minAngle > angle);
    }

}
