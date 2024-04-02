package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Disabled
public class SleeveDetection extends OpenCvPipeline {
    /*
    GREEN  = Parking Left
    BROWN   = Parking Middle
    MAGENTA = Parking Right
     */

    public enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    // TOPLEFT anchor point for the bounding box
    private static Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(145, 168);

    // Width and height for the bounding box
    public static int REGION_WIDTH = 30;
    public static int REGION_HEIGHT = 50;

    // Lower and upper boundaries for colors
    private static final Scalar
            // green scalar
            lower_green_bounds  = new Scalar(0, 144, 0, 255),
            upper_green_bounds  = new Scalar(40, 255, 30, 255),
            // brown scalar
            lower_brown_bounds    = new Scalar(129, 103, 60, 255),
            upper_brown_bounds    = new Scalar(200, 130, 76, 255),
            // magenta scalar
            lower_magenta_bounds = new Scalar(170, 0, 80, 255),
            upper_magenta_bounds = new Scalar(255, 42, 178, 255);

    // Color definitions
    private final Scalar
            GREEN  = new Scalar(31, 234, 0),
            BROWN   = new Scalar(150, 110, 80),
            MAGENTA = new Scalar(255, 37, 189);

    // Percent and mat definitions
    private double grePercent, broPercent, magPercent;
    private Mat greMat = new Mat(), broMat = new Mat(), magMat = new Mat(), blurredMat = new Mat();

    // Anchor point definitions
    Point sleeve_pointA = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point sleeve_pointB = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    // Running variable storing the parking position
    private volatile ParkingPosition position = ParkingPosition.LEFT;

    @Override
    public Mat processFrame(Mat input) {
        // Noise reduction
        Imgproc.blur(input, blurredMat, new Size(5, 5));
        blurredMat = blurredMat.submat(new Rect(sleeve_pointA, sleeve_pointB));

        // Apply Morphology
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMat, blurredMat, Imgproc.MORPH_CLOSE, kernel);

        // Gets channels from given source mat
        Core.inRange(blurredMat, lower_green_bounds, upper_green_bounds, greMat);
        Core.inRange(blurredMat, lower_brown_bounds, upper_brown_bounds, broMat);
        Core.inRange(blurredMat, lower_magenta_bounds, upper_magenta_bounds, magMat);

        // Gets color specific values
        grePercent = Core.countNonZero(greMat);
        broPercent = Core.countNonZero(broMat);
        magPercent = Core.countNonZero(magMat);

        // Calculates the highest amount of pixels being covered on each side
        double maxPercent = Math.max(grePercent, Math.max(broPercent, magPercent));

        // Checks all percentages, will highlight bounding box in camera preview
        // based on what color is being detected
        if (maxPercent == grePercent) {
            position = ParkingPosition.LEFT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    GREEN,
                    2
            );
        } else if (maxPercent == broPercent) {
            position = ParkingPosition.CENTER;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    BROWN,
                    2
            );
        } else if (maxPercent == magPercent) {
            position = ParkingPosition.RIGHT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    MAGENTA,
                    2
            );
        }

        // Memory cleanup
        blurredMat.release();
        greMat.release();
        broMat.release();
        magMat.release();

        return input;
    }

    // Returns an enum being the current position where the robot will park
    public ParkingPosition getPosition() {
        return position;
    }
}