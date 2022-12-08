package org.firstinspires.ftc.team16909.autonomous;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class ColorDetectionPipeline extends OpenCvPipeline
{

    int width = 50;
    int height = 50;
    Point TOP_LEFT_BOUND = new Point(150,100);
    Point BOTTOM_RIGHT_BOUND = new Point(TOP_LEFT_BOUND.x + width, TOP_LEFT_BOUND.y + height);
    Rect window = new Rect(TOP_LEFT_BOUND, BOTTOM_RIGHT_BOUND);
    public int meanCol;
    int [] colDistances = new int[3];
    int lavender = 286; //(286, 78%, 78%) #de9bf3
    int darkGreen = 62; //(123, 62%, 17%) #104613
    int lightBlue = 92; //(178, 43%, 90%) #dbf0f0
    int[] cols = {lavender, darkGreen, lightBlue};
    String[] locations = {"left", "middle", "right"};
    String[] colors = {"lavender", "darkGreen", "lightBlue"};
    Scalar[] RGBCONVERSION = {new Scalar(222, 155, 243), new Scalar(16, 70, 19), new Scalar(219, 240, 240)};
    Mat submat = new Mat();
    Mat hsv = new Mat();


    private volatile int parkPoint = 0;


    public void init(Mat initFrame)
    {
        RGB2HSV(initFrame);
    }

    public void RGB2HSV(Mat input)
    {
        submat = input.submat(window);
        Imgproc.cvtColor(submat, hsv, Imgproc.COLOR_RGB2HSV);
    }

    @Override
    public Mat processFrame(Mat input)
    {
        RGB2HSV(input);

        meanCol = (int) Core.mean(hsv).val[0];


        for (int i = 0; i < cols.length; i++)
        {
            colDistances[i] = Math.abs(meanCol - cols[i]);
        }

        parkPoint = 0;

        for(int i = 1; i < colDistances.length; i++)
        {
            if (colDistances[i] < colDistances[parkPoint])
            {
                parkPoint = i;
            }
        }


        Imgproc.rectangle(input, window, RGBCONVERSION[parkPoint], 5);


        return input;

    }

    public String getParkPoint()
    {
        return locations[parkPoint];
    }

    public String getColor()
    {
        return colors[parkPoint];
    }
}
