package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvPipeline;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

@TeleOp(name = "GreenDetectionOpMode")
public class GreenDetectionOpMode extends LinearOpMode {

    private OpenCvWebcam webcam;
    private Servo servo;

    @Override
    public void runOpMode() throws InterruptedException {

        int camViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1"),
                camViewId
        );

        servo = hardwareMap.get(Servo.class, "servo");

        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        GreenPipeline pipeline = new GreenPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        telemetry.addLine("Place object at 5 inches (12.7 cm) for calibration");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (pipeline.isCalibrated()) {
                telemetry.addData("Calibrated Focal Length", pipeline.getFocalLength());
            } else {
                telemetry.addLine("Calibrating...");
            }

            double distanceInches = pipeline.getDistanceCm() / 2.54;
            telemetry.addData("Estimated Distance (inches)", String.format("%.2f", distanceInches));
            telemetry.update();

            double distanceMeters = distanceInches * 0.0254;

            // Only aim cannon if object detected
            if (pipeline.getDistanceCm() > 0) {
                CannonUtils.aimCannon(servo, distanceMeters);
            }

            sleep(50);
        }
    }

    // -------------------- PIPELINE --------------------
    static class GreenPipeline extends OpenCvPipeline {

        private double distanceCm = -1;
        private double focalLength = -1;
        private boolean calibrated = false;

        // Real-world object width (cm)
        private static final double KNOWN_WIDTH_CM = 5.0;

        // Known calibration distance (cm) — 5 inches = 12.7 cm
        private static final double KNOWN_DISTANCE_CM = 12.7;

        // HSV range for green shades (#5c9c2e and #72860a)
        private static final Scalar LOWER_HSV = new Scalar(30, 70, 40);
        private static final Scalar UPPER_HSV = new Scalar(75, 255, 255);

        // Ignore small contours (px²)
        private static final double MIN_CONTOUR_AREA = 500.0;

        @Override
        public Mat processFrame(Mat input) {

            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            Mat mask = new Mat();
            Core.inRange(hsv, LOWER_HSV, UPPER_HSV, mask);

            // Find contours
            List<org.opencv.core.MatOfPoint> contours = new java.util.ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            if (!contours.isEmpty()) {
                double maxArea = 0;
                Rect bestRect = null;

                for (org.opencv.core.MatOfPoint contour : contours) {
                    Rect rect = Imgproc.boundingRect(contour);

                    // Skip tiny contours
                    if (rect.area() < MIN_CONTOUR_AREA) {
                        continue;
                    }

                    if (rect.area() > maxArea) {
                        maxArea = rect.area();
                        bestRect = rect;
                    }
                }

                if (bestRect != null) {
                    // Draw bounding box
                    Imgproc.rectangle(input, bestRect, new Scalar(0, 255, 0), 2);

                    double perceivedWidth = bestRect.width;

                    // Auto-calibration
                    if (!calibrated) {
                        focalLength = (perceivedWidth * KNOWN_DISTANCE_CM) / KNOWN_WIDTH_CM;
                        calibrated = true;
                    }

                    // Distance calculation
                    if (focalLength > 0) {
                        distanceCm = (KNOWN_WIDTH_CM * focalLength) / perceivedWidth;

                        // Draw label + distance above the box
                        double distanceInches = distanceCm / 2.54;
                        Imgproc.putText(
                                input,
                                String.format("Green Object: %.2f in", distanceInches),
                                new Point(bestRect.x, bestRect.y - 10),
                                Imgproc.FONT_HERSHEY_SIMPLEX,
                                0.7,
                                new Scalar(0, 255, 0),
                                2
                        );
                    }
                }
            }

            return input;
        }

        public double getDistanceCm() { return distanceCm; }
        public boolean isCalibrated() { return calibrated; }
        public double getFocalLength() { return focalLength; }
    }

}
