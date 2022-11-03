package org.firstinspires.ftc.team16910.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team16910.Hardware.SpaghettiHardware;

public class Functions
{
    //CHANGE THESE VALUES LATER
    public final int[] juncHeight = {100,200,300};
    final double openClaw = 0.5;
    final double closeClaw = 0;
    private SpaghettiHardware hardware;

    Functions(SpaghettiHardware hardware)
    {
        this.hardware = hardware;
    }

    public void moveLift(int position)
    {
        //VALUE WILL BE CHANGED
  /*  hardware.liftMotor.setTargetPosition(position);
    hardware.liftMotor2.setTargetPosition(position);

    hardware.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    hardware.liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    hardware.liftMotor.setPower(1);
    hardware.liftMotor2.setPower(1);
    */




    }

  /*  public void clawGrasp() {
        if (hardware.clawServo.getPosition() == openClaw) {
            hardware.clawServo.setPosition(closeClaw);

        } else //if claw is closed
        {
            hardware.clawServo.setPosition(openClaw);
        }
    }

   */

    public void wait(int waitTime, Telemetry telemetry) {
        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        time.reset();
        while (time.time() < waitTime) {
            telemetry.addData("Status", "Waiting");
            telemetry.addData("Wait Time", waitTime / 1000);
            telemetry.addData("Time Left", (waitTime - time.time()) / 1000);
            telemetry.update();
        }
    }


}
