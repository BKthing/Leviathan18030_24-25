package org.firstinspires.ftc.teamcode.teleops;

import static org.opencv.core.Core.bitwise_and;
import static org.opencv.core.Core.bitwise_not;
import static org.opencv.core.Core.bitwise_or;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;

public class TestOpenCV extends OpenCvPipeline {

    Scalar lowB = new Scalar(110, 20, 33);
    Scalar highB = new Scalar(130, 255, 80);

    Scalar lowR = new Scalar(0, 60, 33);
    Scalar highR = new Scalar(30, 255, 80);

    Scalar lowY = new Scalar(35, 20, 33);
    Scalar highY = new Scalar(71, 255, 80);

    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(24, 24));
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(12, 12));


    Mat blurred = new Mat();
    Mat maskBlue = new Mat();
    Mat maskRed = new Mat();
    Mat maskYellow = new Mat();

    Mat hsvMat = new Mat();

    Mat morphOutputBlue = new Mat();
    Mat morphOutputRed = new Mat();
    Mat morphOutputYellow = new Mat();


    Mat output = new Mat();


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

        Imgproc.cvtColor(blurred, hsvMat, Imgproc.COLOR_RGB2HSV);

        if (hsvMat.empty()) {
            return input;
        }

        input.copyTo(output);


        //Blue
        Core.inRange(hsvMat, lowB, highB, maskBlue);

        Imgproc.erode(maskBlue, morphOutputBlue, erodeElement);
        Imgproc.erode(maskBlue, morphOutputBlue, erodeElement);

        Imgproc.dilate(maskBlue, morphOutputBlue, dilateElement);
        Imgproc.dilate(maskBlue, morphOutputBlue, dilateElement);

        List<MatOfPoint> contoursBlue = new ArrayList<>();
        Mat hierarchyBlue = new Mat();

        Imgproc.findContours(morphOutputBlue, contoursBlue, hierarchyBlue, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);//mask

        if (hierarchyBlue.size().height > 0 && hierarchyBlue.size().width > 0) { //
            for (int i = 0; i >= 0; i = (int) hierarchyBlue.get(0, i)[0]) {
                Imgproc.drawContours(output, contoursBlue, i, new Scalar(0, 0, 0), 3);
            }
        }

        output.setTo(blue, maskBlue);


        //Red
        Core.inRange(hsvMat, lowR, highR, maskRed);

        Imgproc.erode(maskRed, morphOutputRed, erodeElement);
        Imgproc.erode(maskRed, morphOutputRed, erodeElement);

        Imgproc.dilate(maskRed, morphOutputRed, dilateElement);
        Imgproc.dilate(maskRed, morphOutputRed, dilateElement);

        List<MatOfPoint> contoursRed = new ArrayList<>();
        Mat hierarchyRed = new Mat();

        Imgproc.findContours(morphOutputRed, contoursRed, hierarchyRed, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);//mask

        if (hierarchyRed.size().height > 0 && hierarchyRed.size().width > 0) { //
            for (int i = 0; i >= 0; i = (int) hierarchyRed.get(0, i)[0]) {
                Imgproc.drawContours(output, contoursRed, i, new Scalar(0, 0, 0), 3);
            }
        }

        output.setTo(red, maskRed);


        //Yellow
        Core.inRange(hsvMat, lowY, highY, maskYellow);

        Imgproc.erode(maskYellow, morphOutputYellow, erodeElement);
        Imgproc.erode(maskYellow, morphOutputYellow, erodeElement);

        Imgproc.dilate(maskYellow, morphOutputYellow, dilateElement);
        Imgproc.dilate(maskYellow, morphOutputYellow, dilateElement);

        List<MatOfPoint> contoursYellow = new ArrayList<>();
        Mat hierarchyYellow = new Mat();

        Imgproc.findContours(morphOutputYellow, contoursYellow, hierarchyYellow, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);//mask

        if (hierarchyYellow.size().height > 0 && hierarchyYellow.size().width > 0) { //
            for (int i = 0; i >= 0; i = (int) hierarchyYellow.get(0, i)[0]) {
                Imgproc.drawContours(output, contoursYellow, i, new Scalar(0, 0, 0), 3);
            }
        }

        output.setTo(yellow, maskYellow);


        return output;


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
