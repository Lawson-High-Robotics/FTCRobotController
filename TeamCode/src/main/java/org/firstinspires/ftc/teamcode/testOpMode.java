package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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

@TeleOp(name = "testOpMode")
public class testOpMode extends LinearOpMode {

    private OpenCvWebcam webcam;
    private DcMotor armLeft;
    private DcMotor armRight;

    // Arm control constants
    private static final double ARM_MIN_ANGLE = 0;    // degrees, fully down
    private static final double ARM_MAX_ANGLE = 90;   // degrees, fully up
    private static final int ENCODER_COUNTS_PER_REV = 560; // depends on your motor
    private static final double GEAR_RATIO = 1.0; // motor-to-arm ratio
    private static final double DISTANCE_TO_ANGLE_SCALE = 5.0; // tweak this for mapping distance→angle

    @Override
    public void runOpMode() throws InterruptedException {

        int camViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1"),
                camViewId
        );

        armLeft = hardwareMap.get(DcMotor.class, "arm_left");
        armRight = hardwareMap.get(DcMotor.class, "arm_right");

        // Reverse one motor if necessary
        armLeft.setDirection(DcMotor.Direction.FORWARD);
        armRight.setDirection(DcMotor.Direction.REVERSE);

        // Initialize motors for RUN_TO_POSITION
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

            double distanceCm = pipeline.getDistanceCm();
            double distanceInches = distanceCm / 2.54;
            telemetry.addData("Estimated Distance (inches)", String.format("%.2f", distanceInches));

            if (distanceCm > 0) {
                // Map distance to arm angle
                double targetAngle = Math.min(Math.max(distanceCm * DISTANCE_TO_ANGLE_SCALE, ARM_MIN_ANGLE), ARM_MAX_ANGLE);
                int targetCounts = angleToCounts(targetAngle);

                armLeft.setTargetPosition(targetCounts);
                armRight.setTargetPosition(targetCounts); // opposite direction already set by motor reversal

                armLeft.setPower(0.5);
                armRight.setPower(0.5);
            } else {
                armLeft.setPower(0);
                armRight.setPower(0);
            }

            telemetry.update();
            sleep(50);
        }
    }

    private int angleToCounts(double angle) {
        return (int)((angle / 360.0) * ENCODER_COUNTS_PER_REV * GEAR_RATIO);
    }

    // -------------------- PIPELINE --------------------
    static class GreenPipeline extends OpenCvPipeline {

        private double distanceCm = -1;
        private double focalLength = -1;
        private boolean calibrated = false;

        private static final double KNOWN_WIDTH_CM = 5.0;
        private static final double KNOWN_DISTANCE_CM = 12.7;

        // HSV range for green shades (#5c9c2e and #72860a)
        private static final Scalar LOWER_HSV = new Scalar(30, 70, 40);
        private static final Scalar UPPER_HSV = new Scalar(75, 255, 255);

        private static final double MIN_CONTOUR_AREA = 500.0;

        @Override
        public Mat processFrame(Mat input) {
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            Mat mask = new Mat();
            Core.inRange(hsv, LOWER_HSV, UPPER_HSV, mask);

            List<org.opencv.core.MatOfPoint> contours = new java.util.ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            if (!contours.isEmpty()) {
                double maxArea = 0;
                Rect bestRect = null;

                for (org.opencv.core.MatOfPoint contour : contours) {
                    Rect rect = Imgproc.boundingRect(contour);
                    if (rect.area() < MIN_CONTOUR_AREA) continue;
                    if (rect.area() > maxArea) {
                        maxArea = rect.area();
                        bestRect = rect;
                    }
                }

                if (bestRect != null) {
                    Imgproc.rectangle(input, bestRect, new Scalar(0, 255, 0), 2);
                    double perceivedWidth = bestRect.width;

                    if (!calibrated) {
                        focalLength = (perceivedWidth * KNOWN_DISTANCE_CM) / KNOWN_WIDTH_CM;
                        calibrated = true;
                    }

                    if (focalLength > 0) {
                        distanceCm = (KNOWN_WIDTH_CM * focalLength) / perceivedWidth;
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
            } else {
                distanceCm = -1; // no object detected
            }

            return input;
        }

        public double getDistanceCm() { return distanceCm; }
        public boolean isCalibrated() { return calibrated; }
        public double getFocalLength() { return focalLength; }
    }
}
