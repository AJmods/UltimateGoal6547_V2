package org.firstinspires.ftc.teamcode.testing.openCV;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.pipeline.openCvPipeLines;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


/**
 * Created by maryjaneb  on 11/13/2016.
 *
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 *
 * monitor: 640 x 480
 *YES
 */
@TeleOp(name= "opencvRingDetectorTest")
@Disabled
public class RingOpenCVTest extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    OpenCvCamera phoneCam;

    private final int rows = 320;
    private final int cols = 240;

    openCvPipeLines.RingDetectionPipeLine ringDetectionPipeLine;

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        phoneCam.openCameraDevice();//open camera
        //set pipeline
        ringDetectionPipeLine = new openCvPipeLines.RingDetectionPipeLine();
        phoneCam.setPipeline(new openCvPipeLines.RingDetectionPipeLine());

        //phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        phoneCam.openCameraDeviceAsync(() -> phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPSIDE_DOWN));
        //width, height
        //width = height in this case, because camera is in portrait mode.

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {

            telemetry.addData("RING COUNT: ", openCvPipeLines.RingDetectionPipeLine.getRingCount());
            telemetry.addData("RING COUNT: ", openCvPipeLines.RingDetectionPipeLine.getRingCount());
            telemetry.update();
            sleep(100);
            //call movement functions

            }
        phoneCam.stopStreaming();
            }
    }
