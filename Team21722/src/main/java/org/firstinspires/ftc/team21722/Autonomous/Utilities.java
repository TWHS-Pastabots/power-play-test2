package org.firstinspires.ftc.team21722.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team21722.Hardware.MacaroniHardware;

public class Utilities
{
    private MacaroniHardware macaroniHardware;


    public Utilities(MacaroniHardware macaroniHardware){this.macaroniHardware= macaroniHardware;}

   public void moveArm(int e)
   {
        macaroniHardware.liftArm.setTargetPosition(macaroniHardware.liftArm.getCurrentPosition() + e);
        macaroniHardware.liftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        macaroniHardware.liftArm.setPower(1);
   }





    public void openClaw(){macaroniHardware.clawServo.setPosition(.10);}

    public void closeClaw(){macaroniHardware.clawServo.setPosition(.70);}

//    public void wait (int waitTime)
//    {
//        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//
//        while(time.time() < waitTime)
//        {
//            continue;
//        }
//    }
    public void wait(int waitTime, Telemetry telemetry)
    {
        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        time.reset();
        while (time.time() < waitTime)
        {
            telemetry.addData("Status", "Waiting");
            telemetry.addData("Wait Time", waitTime / 1000);
            telemetry.addData("Time Left", (waitTime - time.time()) / 1000);
            telemetry.update();
        }
    }
}
