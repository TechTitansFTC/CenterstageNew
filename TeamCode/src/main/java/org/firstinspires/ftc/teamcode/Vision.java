package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;

public class Vision extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                // custom intrinsics .setLensIntrinsics()
                // set num threads .setNumThreads() default is 3
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .build();

        CameraName webcam = hardwareMap.get(CameraName.class, "Webcam 1");
        
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(webcam)
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true) // camera monitoring, default true
                // setAutoStopLiveView - auto disables live view if you disable a processor, save resources
                // setCameraMonitorViewId - only needed if several cameras
                // setStreamFormat - mjpeg or yuy2 bandwith vs quality tradeoff, mjep @ 640 x 480 good, YUY2 default
                .build();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            /*
            * tagProcessor methods
            * .getFreshDetections() - returns a list of new readings
            * .setDecimation() - allows you to set a custom decimation value, higher frame rate but lower range
            * .setPoseSolver() - allows you to choose from several different ways to calculating the tag to camera pose
            * .getPerTagAvgPoseSolveTime() - how long it takes to retrieve the active tag pose using the active pose solve
            * */


            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);



                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("yaw", tag.ftcPose.yaw);
            }

            telemetry.update();
        }
    }
}
