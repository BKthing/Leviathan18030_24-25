package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.opencv.core.Core.bitwise_and;
import static org.opencv.core.Core.bitwise_not;
import static org.opencv.core.Core.bitwise_or;
import static org.opencv.core.Core.inRange;
import static org.opencv.imgproc.Imgproc.COLOR_BGR2HSV;
import static org.opencv.imgproc.Imgproc.COLOR_GRAY2BGR;
import static org.opencv.imgproc.Imgproc.COLOR_HSV2BGR;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.utils.Converters;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;

public class TestOpenCV extends OpenCvPipeline {

    Mat outputLayer = new Mat();
    Mat output = new Mat();

    Mat blurred = new Mat();
    Mat mask = new Mat();

    Mat hsvMat = new Mat();

    Mat morphOutput = new Mat();

    Mat white2Blue = new Mat();


    ArrayList<double[]> framelist;

    private final Scalar
            blue = new Scalar(0, 0, 255),
            red = new Scalar(255, 0, 0),
            yellow = new Scalar(255, 255, 0);

    public TestOpenCV() {
        framelist = new ArrayList<>();
    }


    //bgr
    @Override
    public Mat processFrame(Mat input) {

        // following https://opencv-java-tutorials.readthedocs.io/en/latest/08-object-detection.html

        Imgproc.blur(input, blurred, new Size(7, 7));
//
        Imgproc.cvtColor(blurred, hsvMat, Imgproc.COLOR_RGB2HSV);

        if (hsvMat.empty()) {
            return input;
        }


        Scalar lowB = new Scalar(105, 20, 33);
        Scalar highB = new Scalar(130, 255, 200);

        Core.inRange(hsvMat, lowB, highB, mask);

        Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(24, 24));
        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(12, 12));

        Imgproc.erode(mask, morphOutput, erodeElement);
        Imgproc.erode(mask, morphOutput, erodeElement);

        Imgproc.dilate(mask, morphOutput, dilateElement);
        Imgproc.dilate(mask, morphOutput, dilateElement);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(morphOutput, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);//mask

//        input.copyTo(output);

//        try {
//            Imgproc.applyColorMap(morphOutput, outputLayer, white2Blue);
////            Imgproc.cvtColor(morphOutput, outputLayer, Imgproc.COLOR_HSV2BGR);
//        } catch (Exception e) {
//            throw new RuntimeException("other new exeption");
//        }


        Imgproc.cvtColor(mask, outputLayer, COLOR_GRAY2BGR);
        input.copyTo(output);

        output.setTo(new Scalar(0, 0, 255), mask);


        return output;
//
//        if (outputLayer.channels() != input.channels()) {
//            throw new RuntimeException("issuueue");
//        }

//        try {
//            Core.addWeighted(input, .5, outputLayer, .5, 0, output);
//        } catch (Exception e) {
//            throw new RuntimeException("other issueeueue");
//        }
//
//        if (hierarchy.size().height > 0 && hierarchy.size().width > 0) { //
//            for (int i = 0; i >= 0; i = (int) hierarchy.get(0, i)[0]) {
//                Imgproc.drawContours(output, contours, i, blue);
//            }
//        }



//        return outputLayer;
//
//        // would you be able to repeat with yellow and red?
//
//        Scalar lowY;
//        Scalar highY;
//
//        Scalar lowR;
//        Scalar highR;
//
//        return output;
    }
}
