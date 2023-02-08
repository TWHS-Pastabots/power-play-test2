package org.firstinspires.ftc.team15021.auton;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class AltSignalPipeline extends OpenCvPipeline
{

    private final Rect SIGNAL_ZONE = new Rect(176, 72, 5, 5);

    private boolean signalGreen;
    private boolean signalPurple;
    private boolean signalOrange;
    private volatile ans color;

    int avg;

    Mat hsv = new Mat();
    Mat H = new Mat();
    Mat test = new Mat();
    public enum ans
    {
        GREEN,
        PURPLE,
        ORANGE
    }
    void extractToHsv(Mat input)
    {
        Imgproc.cvtColor(input, hsv, COLOR_RGB2HSV);
        Core.extractChannel(hsv, H, 0);
    }
    @Override
    public void init(Mat initFrame)
    {
        extractToHsv(initFrame);
    }
    @Override
    public Mat processFrame(Mat input)
    {
        extractToHsv(input);
        test = input.submat(SIGNAL_ZONE);

        avg = (int) Core.mean(test).val[0];

        signalGreen = (-1 < (int) Core.mean(test).val[0])&& ((int) Core.mean(test).val[0] < 76);
        signalPurple = (75 < (int) Core.mean(test).val[0])&& ((int) Core.mean(test).val[0] < 150);
        signalOrange = (150 < (int) Core.mean(test).val[0])&& ((int) Core.mean(test).val[0] < 20000);

        if(signalGreen||signalPurple||signalOrange)
        {
            if (signalGreen)
                Imgproc.rectangle(input, SIGNAL_ZONE, new Scalar(0, 255, 0), 5);
            if (signalPurple)
                Imgproc.rectangle(input, SIGNAL_ZONE, new Scalar(255, 0, 0), 5);
            if (signalOrange)
                Imgproc.rectangle(input, SIGNAL_ZONE, new Scalar(0, 0, 255), 5);
        }
        else
        {
            Imgproc.rectangle(input, SIGNAL_ZONE, new Scalar(255, 255, 255), 5);
        }
        processSignalColor();
        hsv.release();
        H.release();
        test.release();
        return input;

    }

    private void processSignalColor()
    {
        boolean[] colors = new boolean[3];
        colors[0] = signalGreen;
        colors[1] = signalPurple;
        colors[2] = signalOrange;
        for (int i = 0; i<colors.length; i++)
        {
            if(colors[i]&&i==0)
                color = ans.GREEN;
            if(colors[i]&&i==1)
                color = ans.PURPLE;
            if(colors[i]&&i==2)
                color = ans.ORANGE;
        }

    }
    public ans getAnalysis(){return color;}
}
