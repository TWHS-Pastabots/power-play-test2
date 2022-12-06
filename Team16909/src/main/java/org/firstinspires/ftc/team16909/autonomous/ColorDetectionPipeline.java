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
    Point TOP_LEFT_BOUND = new Point(50,50);
    Point BOTTOM_RIGHT_BOUND = new Point(TOP_LEFT_BOUND.x + width, TOP_LEFT_BOUND.y + height);
    int lavender = 286; //(286, 78%, 78%)
    int darkGreen = 62; //(123, 62%, 17%)
    int lightBlue = 92; //(178, 43%, 90%)
    int[] cols = {lavender, darkGreen, lightBlue};
    public enum color
    {
        LAVENDER,
        DARKGREEN,
        LIGHTBLUE
    }

    private volatile color chosenColor;

    @Override
    public Mat processFrame(Mat input)
    {
        Mat mat = new Mat();
        Mat hue = new Mat();

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(mat, hue, 0);


        if(mat.empty())
        {
            return input;
        }

        Rect window = new Rect(TOP_LEFT_BOUND, BOTTOM_RIGHT_BOUND);

        Mat submat = mat.submat(window);

        int meanCol = (int) Core.mean(submat).val[0];
        int closestCol = 999;


        for(int col : cols)
        {
            if (Math.abs(col - meanCol) < Math.abs(closestCol - meanCol))
            {
                closestCol = col;

                if (col == lavender)
                {
                    chosenColor = color.LAVENDER;
                }

                else if (col == darkGreen)
                {
                    chosenColor = color.DARKGREEN;
                }

                else if (col == lightBlue)
                {
                    chosenColor = color.LIGHTBLUE;
                }
            }
        }

        Imgproc.rectangle(input, window, new Scalar(closestCol, 39, 5));

        return input;
    }

    public color getChosenColor()
    {
        return chosenColor;
    }
}
