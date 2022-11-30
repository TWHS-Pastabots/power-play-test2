package org.firstinspires.ftc.team15021.auton;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SignalPipeline extends OpenCvPipeline
{

    private final Rect SIGNAL_ZONE = new Rect(176, 90, 40, 40);

    private boolean signalGreen;
    private boolean signalPurple;
    private boolean signalOrange;
    private volatile ans color;

    int HELP;

    Mat hsv = new Mat();
    Mat H = new Mat();
    Mat donkey = new Mat();
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
        donkey = input.submat(SIGNAL_ZONE);

        HELP = (int) Core.mean(donkey).val[0];
        /*
        signalGreen = (50 < (int) Core.mean(donkey).val[0])&& ((int) Core.mean(donkey).val[0] < 70);
        signalPurple = (140 < (int) Core.mean(donkey).val[0])&& ((int) Core.mean(donkey).val[0] < 160);
        signalOrange = (10 < (int) Core.mean(donkey).val[0])&& ((int) Core.mean(donkey).val[0] < 20);
        */
        signalGreen = (20 < (int) Core.mean(donkey).val[0])&& ((int) Core.mean(donkey).val[0] < 40);
        signalPurple = (105 < (int) Core.mean(donkey).val[0])&& ((int) Core.mean(donkey).val[0] < 135);
        signalOrange = (150 < (int) Core.mean(donkey).val[0])&& ((int) Core.mean(donkey).val[0] < 190);

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
