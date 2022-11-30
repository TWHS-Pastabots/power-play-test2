package org.firstinspires.ftc.team15021.auton;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team15021.hardware.RavioliHardware;

public class util
{
    private RavioliHardware ravioliHardware;

    public util (RavioliHardware ravioliHardware)
    {
        this.ravioliHardware = ravioliHardware;
    }

   /* public void moveArm01(int position)
    {

        ravioliHardware.armMotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ravioliHardware.armMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while(!(position-5< ravioliHardware.armMotor0.getCurrentPosition() &&
                ravioliHardware.armMotor1.getCurrentPosition()< position+5))
        {
            if(position<ravioliHardware.armMotor0.getCurrentPosition())
            {
                ravioliHardware.armMotor0.setPower(-.25);
                ravioliHardware.armMotor1.setPower(-.25);
            }
            if(position>ravioliHardware.armMotor0.getCurrentPosition())
            {
                ravioliHardware.armMotor0.setPower(.25);
                ravioliHardware.armMotor1.setPower(.25);
            }
        }
        ravioliHardware.armMotor0.setPower(0);
        ravioliHardware.armMotor1.setPower(0);

    }*/
    public void moveArm01(int position)
    {
        ravioliHardware.armMotor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ravioliHardware.armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ravioliHardware.armMotor0.setTargetPosition(ravioliHardware.armMotor0.getCurrentPosition()+position);
        ravioliHardware.armMotor1.setTargetPosition(ravioliHardware.armMotor1.getCurrentPosition()+position);

        ravioliHardware.armMotor0.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        ravioliHardware.armMotor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        ravioliHardware.armMotor0.setPower(1);
        ravioliHardware.armMotor1.setPower(1);

    }
    public void moveArm2(int position)
    {
        ravioliHardware.armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ravioliHardware.armMotor2.setTargetPosition(ravioliHardware.armMotor2.getCurrentPosition()+position);

        ravioliHardware.armMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        ravioliHardware.armMotor2.setPower(.5);

    }


    public String getPos()
    {
        return (ravioliHardware.armMotor0.getCurrentPosition()+ " + " + ravioliHardware.armMotor1.getCurrentPosition());
    }


    public void openClaw()
    {
        ravioliHardware.servo1.setPosition(0);
    }

    public void closeClaw()
    {
        ravioliHardware.servo1.setPosition(1);
    }

    public void tiltDown() { ravioliHardware.servo0.setPosition(1); }

    public void tiltUp() { ravioliHardware.servo0.setPosition(.4); }

    public void resetClaw() {ravioliHardware.servo0.scaleRange(0,1);}


    public void wait(int waitTime, Telemetry tele)
    {
        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        while (time.time() < waitTime)
        {
            tele.addData("Time Left", (waitTime-time.time())/1000.0 );
            tele.addData("Time Limit", waitTime/1000);
            tele.update();
            continue;
        }
    }
}
