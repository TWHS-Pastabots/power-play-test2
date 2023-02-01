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
    boolean isRight;
    int width = 25;
    int height = 45;
    Point TOP_LEFT_BOUND = new Point(280,125 );
    Point BOTTOM_RIGHT_BOUND = new Point(TOP_LEFT_BOUND.x + width, TOP_LEFT_BOUND.y + height);
    Rect window = new Rect(TOP_LEFT_BOUND, BOTTOM_RIGHT_BOUND);
    public int meanCol;
    int [] colDistances = new int[3];
    int yellow = 37; //(60, 100%, 50%) #de9bf3
    int magenta = 160; //(300, 100%, 50%) #104613
    int cyan = 103; //(180, 100%, 50%) #dbf0f0
    int[] cols = {yellow, magenta, cyan};
    String[] locations = {"left", "middle", "right"};
    String[] colors = {"yellow", "magenta", "cyan"};
    Scalar[] RGBCONVERSION = {new Scalar(255, 255, 0), new Scalar(255, 0, 255), new Scalar(0, 255, 255)};
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

        if (isRight)
        {
            Imgproc.rectangle(input, new Rect(75, 50, 5,100), new Scalar(0, 0, 0), 2);
        }

        else
        {
            Imgproc.rectangle(input, new Rect(75, 30, 5,100), new Scalar(0, 0, 0), 2);
        }


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
