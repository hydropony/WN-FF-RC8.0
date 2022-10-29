package org.firstinspires.ftc.teamcode.modules;

import static org.firstinspires.ftc.teamcode.modules.PipelineConfig.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

//@Config
public class TeamElementDetectionPipeline extends OpenCvPipeline {
    public enum Position {
        LEFT,
        MIDDLE,
        RIGHT,
        NONE
    }
    public static Position position;
    private Mat mat;
    private Mat ret;
    private boolean isRed = true;
    Telemetry telemetry;
    boolean debug = true;
    private LinearOpMode opMode;

    /** width of the camera in use, defaulted to 320 as that is most common in examples **/
    public static int CAMERA_WIDTH = 320;

    /** Horizon value in use, anything above this value (less than the value) since
     * (0, 0) is the top left of the camera frame **/
    public static int HORIZON =  (int)((100.0 / 320.0) * CAMERA_WIDTH);

    /** algorithmically calculated minimum width for width check based on camera width **/

    public TeamElementDetectionPipeline(Telemetry tele, boolean d, boolean isRed, LinearOpMode opmode) {
        position = Position.NONE;
        ret = new Mat();
        mat = new Mat();
        telemetry = tele;
        debug = d;
        this.isRed = isRed;
        this.opMode = opmode;
    }

//    @Override
//    public Mat processFrame(Mat input) {
//        ret.release();
//        ret = new Mat();
//
//        try {
//            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HLS_FULL);
//            /**checking if any pixel is within the orange bounds to make a black and white mask**/
//            Mat mask = new Mat(mat.rows(), mat.cols(), CvType.CV_8UC1); // variable to store mask i
//            Core.inRange(mat, new Scalar(lhl, lsl, lvl), new Scalar(hhl, hsl, hvl), mask);
//
//            /**applying to input and putting it on ret in black or yellow**/
//            Core.bitwise_and(input, input, ret, mask);
//
//            /**applying GaussianBlur to reduce noise when finding contours**/
//            Imgproc.GaussianBlur(mask, mask, new Size(5.0, 15.0), 0.00);
//
//            /**finding contours on mask**/
//            List<MatOfPoint> contours = new ArrayList<>();
//            Mat hierarchy = new Mat();
//            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
//
//            /**drawing contours to ret in green**/
//            Imgproc.drawContours(ret, contours, -1, new Scalar(0, 255.0, 0.0), 3);
//
//            /**finding widths of each contour, comparing, and storing the widest**/
//            double  maxWidth = 0;
//            Rect maxRect = new Rect();
//            Iterator<MatOfPoint> iterator = contours.iterator();
//            while (iterator.hasNext() && !opMode.isStopRequested()) {
//                MatOfPoint c = iterator.next();
//
//                MatOfPoint2f copy = new MatOfPoint2f(c.toArray());
//                Rect rect = Imgproc.boundingRect(copy);
//
//                double w = rect.width;
//                // checking if the rectangle is below the horizon
//                if (w > maxWidth && rect.y > HORIZON && rect.x > 15 && rect.x < 220) { //rect.y + height
//                    maxWidth = w;
//                    maxRect = rect;
//                }
//                c.release(); // releasing the buffer of the contour, since after use, it is no longer needed
//                copy.release(); // releasing the buffer of the copy of the contour, since after use, it is no longer needed
//            }
//
//            /**drawing widest bounding rectangle to ret in blue**/
//            Imgproc.rectangle(ret, maxRect, new Scalar(0, 0.0, 255.0), 2);
//
//            /** drawing a red line to show the horizon (any above the horizon is not checked to be a ring stack **/
//            Imgproc.line(
//                    ret,
//                    new Point(
//                            .0,
//                            (double)HORIZON
//                    ),
//                    new Point(
//                            (double)CAMERA_WIDTH,
//                            (double)HORIZON
//                    ),
//                    new Scalar(
//                            .0,
//                            100.0,
//                            100.0)
//            );
//
//            if (debug && telemetry != null)
//                telemetry.addData("Vision: maxW", maxWidth);
//
//            /** checking if widest width is greater than equal to minimum width
//             * using Kotlin if expression (Java ternary) to set height variable
//             *
//             * height = maxWidth >= MIN_WIDTH ? aspectRatio > BOUND_RATIO ? FOUR : ONE : ZERO
//             **/
//            if (maxWidth >= MIN_WIDTH) {
////                Double aspectRatio = (double)maxRect.height / (double)maxRect.width;
//                if (maxRect.x >= 190)
//                    position = Position.RIGHT;
//                else if (maxRect.x >= 90 && maxRect.x < 190)
//                    position = Position.MIDDLE;
//                else if (maxRect.x < 90 && maxRect.x > 30)
//                    position = Position.LEFT;
//                else
//                    position = Position.RIGHT;
//                if (debug) {
////                    telemetry.addData("Vision: Aspect Ratio", aspectRatio);
//                    telemetry.addData("Vision: maxrect X", maxRect.x);
//                    telemetry.addData("Vision: maxrect Y", maxRect.y);
//                }
//            }
//            else {
//                position = Position.NONE;
//            }
//
//            if (debug && telemetry != null)
//                telemetry.addData("Vision: Position", position);
//
//            // releasing all mats after use
//            mat.release();
//            mask.release();
//            hierarchy.release();
//        }
//        catch (Exception e) {
//            if (telemetry != null)
//                telemetry.addData("[ERROR]", e);
////            e.stackTrace.toList().stream().forEach { x -> telemetry.addLine(x.toString()); }
//            e.printStackTrace();
//        }
//        if (telemetry != null)
//            telemetry.update();
//
//        return ret; //ret
//    }

//    @Override
//    public Mat processFrame(Mat input) {
//        ret.release();
//        ret = new Mat();
//
//        try {
//            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV_FULL);
//            /**checking if any pixel is within the orange bounds to make a black and white mask**/
//            Mat mask = new Mat(mat.rows(), mat.cols(), CvType.CV_8UC1); // variable to store mask in
//            if (isRed)
//                Core.inRange(mat, new Scalar(redlh, redls, redlv), new Scalar(redhh, redhs, redhv), mask);
//            else
//                Core.inRange(mat, new Scalar(bluelh, bluels, bluelv), new Scalar(bluehh, bluehs, bluehv), mask);
//
//            /**applying to input and putting it on ret in black or yellow**/
//            Core.bitwise_and(input, input, ret, mask);
//
//            /**applying GaussianBlur to reduce noise when finding contours**/
//            Imgproc.GaussianBlur(mask, mask, new Size(5.0, 15.0), 0.00);
//
//            /**finding contours on mask**/
//            List<MatOfPoint> contours = new ArrayList<>();
//            Mat hierarchy = new Mat();
//            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
//
//            /**drawing contours to ret in green**/
//            Imgproc.drawContours(ret, contours, -1, new Scalar(0, 255.0, 0.0), 3);
//
//            /**finding widths of each contour, comparing, and storing the widest**/
//            double maxWidth1 = 0;
//            double maxWidth2 = 0;
//            Rect maxRect1 = new Rect();
//            Rect maxRect2 = new Rect();
//            Iterator<MatOfPoint> iterator = contours.iterator();
//            while (iterator.hasNext() && !opMode.isStopRequested()) {
//                MatOfPoint c = iterator.next();
//
//                MatOfPoint2f copy = new MatOfPoint2f(c.toArray());
//                Rect rect = Imgproc.boundingRect(copy);
//
//                double w = rect.width;
//                // checking if the rectangle is below the horizon
//                if (w > maxWidth1 && rect.y > HORIZON && rect.x > 15 && rect.x < 220) { //rect.y + height
//                    maxWidth1 = w;
//                    maxRect1 = rect;
//                }
//                else if (w > maxWidth2 && rect.y > HORIZON && rect.x > 15 && rect.x < 220) {
//                    maxWidth2 = w;
//                    maxRect2 = rect;
//                }
//
//                c.release(); // releasing the buffer of the contour, since after use, it is no longer needed
//                copy.release(); // releasing the buffer of the copy of the contour, since after use, it is no longer needed
//            }
//
//            /**drawing widest bounding rectangle to ret in blue**/
//            Imgproc.rectangle(ret, maxRect1, new Scalar(0, 0.0, 255.0), 2);
//            Imgproc.rectangle(ret, maxRect2, new Scalar(0, 0.0, 255.0), 2);
//
//            /** drawing a red line to show the horizon (any above the horizon is not checked to be a ring stack **/
//            Imgproc.line(
//                    ret,
//                    new Point(
//                            .0,
//                            (double)HORIZON
//                    ),
//                    new Point(
//                            (double)CAMERA_WIDTH,
//                            (double)HORIZON
//                    ),
//                    new Scalar(
//                            .0,
//                            100.0,
//                            100.0)
//            );
//
//            if (debug && telemetry != null) {
//                telemetry.addData("Vision: maxW1", maxWidth1);
//                telemetry.addData("Vision: maxW2", maxWidth2);
//            }
//
//            /** checking if widest width is greater than equal to minimum width
//             * using Kotlin if expression (Java ternary) to set height variable
//             *
//             * height = maxWidth >= MIN_WIDTH ? aspectRatio > BOUND_RATIO ? FOUR : ONE : ZERO
//             **/
//            if (maxWidth1 >= MIN_WIDTH && maxWidth2 > MIN_WIDTH) {
////                Double aspectRatio = (double)maxRect.height / (double)maxRect.width;
//                if (maxRect1.x <= 190 && maxRect2.x <= 190)
//                    position = Position.RIGHT;
//                else if ((maxRect1.x < 90 && maxRect2.x > 190) || (maxRect2.x < 90 && maxRect1.x > 190))
//                    position = Position.MIDDLE;
//                else if ((maxRect1.x > 90 && maxRect2.x > 190) || (maxRect2.x > 90 && maxRect1.x > 190))
//                    position = Position.LEFT;
//                else
//                    position = Position.RIGHT;
//                if (debug) {
////                    telemetry.addData("Vision: Aspect Ratio", aspectRatio);
//                    telemetry.addData("Vision: maxrect1 X", maxRect1.x);
//                    telemetry.addData("Vision: maxrect1 Y", maxRect1.y);
//                    telemetry.addData("Vision: maxrect2 X", maxRect2.x);
//                    telemetry.addData("Vision: maxrect2 Y", maxRect2.y);
//                }
//            }
//            else {
//                position = Position.NONE;
//            }
//
//            if (debug && telemetry != null)
//                telemetry.addData("Vision: Position", position);
//
//            // releasing all mats after use
//            mat.release();
//            mask.release();
//            hierarchy.release();
//        }
//        catch (Exception e) {
//            if (telemetry != null)
//                telemetry.addData("[ERROR]", e);
////            e.stackTrace.toList().stream().forEach { x -> telemetry.addLine(x.toString()); }
//            e.printStackTrace();
//        }
//        if (telemetry != null)
//            telemetry.update();
//
//        return ret; //ret
//    }

    @Override
    public Mat processFrame(Mat input) {
        ret.release();
        ret = new Mat();

        try {
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
            /**checking if any pixel is within the orange bounds to make a black and white mask**/
            Mat mask1 = new Mat(mat.rows(), mat.cols(), CvType.CV_8UC1); // variable to store mask in
            Mat mask2 = new Mat(mat.rows(), mat.cols(), CvType.CV_8UC1);
            Mat mask = new Mat(mat.rows(), mat.cols(), CvType.CV_8UC1);
            if (isRed) {
                Core.inRange(mat, new Scalar(redlh, redls, redlv), new Scalar(redhh, redhs, redhv), mask1);
                Core.inRange(mat, new Scalar(170, redls, redlv), new Scalar(180, redhs, redhv), mask2);
                Core.bitwise_or(mask1, mask2, mask);
            }
            else
                Core.inRange(mat, new Scalar(bluelh, bluels, bluelv), new Scalar(bluehh, bluehs, bluehv), mask);

            /**applying to input and putting it on ret in black or yellow**/
            Core.bitwise_and(input, input, ret, mask);

            /**applying GaussianBlur to reduce noise when finding contours**/
            Imgproc.GaussianBlur(mask, mask, new Size(5.0, 15.0), 0.00);

            /**finding contours on mask**/
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

            /**drawing contours to ret in green**/
            Imgproc.drawContours(ret, contours, -1, new Scalar(0, 255.0, 0.0), 3);

            /**finding widths of each contour, comparing, and storing the widest**/
            double maxWidth1 = 0;
            double maxWidth2 = 0;
            Rect maxRect1 = new Rect();
            Rect maxRect2 = new Rect();
            ListIterator<MatOfPoint> iterator = contours.listIterator();
            while (iterator.hasNext()) { //iterator.hasNext()
//                MatOfPoint c;
//                if (iterator.hasNext())
//                    c = iterator.next();
//                else if (iterator.hasPrevious())
//                    c = iterator.previous();
//                else
//                    return ret;
                MatOfPoint c = iterator.next();

                MatOfPoint2f copy = new MatOfPoint2f(c.toArray());
                Rect rect = Imgproc.boundingRect(copy);
                Imgproc.rectangle(ret, rect, new Scalar(255.0, 0.0, 0.0), 2);

                double w = rect.width;
                // checking if the rectangle is below the horizon
                if (w > MIN_WIDTH && w > maxWidth1 && rect.y > HORIZON) { //&& rect.x > 15 && rect.x < 220 && rect.height < maxHeight && rect.width < maxHeight
                    maxWidth2 = maxWidth1;
                    maxRect2 = maxRect1;
                    maxWidth1 = w;
                    maxRect1 = rect;
                }
                else if (w <= maxWidth1 && w > MIN_WIDTH && w >= maxWidth2 && rect.y > HORIZON) { //&& rect.x > 15 && rect.x < 220 && rect.height < maxHeight && rect.width < maxHeight
                    maxWidth2 = w;
                    maxRect2 = rect;
                }

                if (rect.y > HORIZON) {
                    telemetry.addData("x", rect.x);
                    telemetry.addData("width", rect.width);
                }

                c.release(); // releasing the buffer of the contour, since after use, it is no longer needed
                copy.release(); // releasing the buffer of the copy of the contour, since after use, it is no longer needed
            }

            /**drawing widest bounding rectangle to ret in blue**/
            Imgproc.rectangle(ret, maxRect1, new Scalar(0, 0.0, 255.0), 2);
            Imgproc.rectangle(ret, maxRect2, new Scalar(0, 0.0, 255.0), 2);

            /** drawing a red line to show the horizon (any above the horizon is not checked to be a ring stack **/
            Imgproc.line(
                    ret,
                    new Point(
                            .0,
                            (double)HORIZON
                    ),
                    new Point(
                            (double)CAMERA_WIDTH,
                            (double)HORIZON
                    ),
                    new Scalar(
                            .0,
                            100.0,
                            100.0)
            );

//            if (debug && telemetry != null) {
//                telemetry.addData("Vision: maxW1", maxWidth1);
//                telemetry.addData("Vision: maxW2", maxWidth2);
//            }

            /** checking if widest width is greater than equal to minimum width
             * using Kotlin if expression (Java ternary) to set height variable
             *
             * height = maxWidth >= MIN_WIDTH ? aspectRatio > BOUND_RATIO ? FOUR : ONE : ZERO
             **/
            if (maxWidth1 >= MIN_WIDTH && maxWidth2 > MIN_WIDTH) {
//                Double aspectRatio = (double)maxRect.height / (double)maxRect.width;
                if (maxRect1.x <= 190 && maxRect2.x <= 190)
                    position = Position.RIGHT;
                else if ((maxRect1.x < 90 && maxRect2.x > 190) || (maxRect2.x < 90 && maxRect1.x > 190))
                    position = Position.MIDDLE;
                else if ((maxRect1.x > 90 && maxRect2.x > 190) || (maxRect2.x > 90 && maxRect1.x > 190))
                    position = Position.LEFT;
                else
                    position = Position.RIGHT;
                if (debug) {
//                    telemetry.addData("Vision: Aspect Ratio", aspectRatio);
                    telemetry.addData("Vision: maxrect1 X", maxRect1.x);
                    telemetry.addData("Vision: maxrect1 Y", maxRect1.y);
                    telemetry.addData("Vision: maxrect2 X", maxRect2.x);
                    telemetry.addData("Vision: maxrect2 Y", maxRect2.y);
                }
            }
            else {
                position = Position.NONE;
            }

            if (debug && telemetry != null)
                telemetry.addData("Vision: Position", position);

            // releasing all mats after use
            mat.release();
            mask.release();
            hierarchy.release();
        }
        catch (Exception e) {
            if (telemetry != null)
                telemetry.addData("[ERROR]", e);
//            e.stackTrace.toList().stream().forEach { x -> telemetry.addLine(x.toString()); }
            e.printStackTrace();
        }
        if (telemetry != null)
            telemetry.update();

        return ret; //ret
    }

    public Position getPosition() {
        return position;
    }
}
