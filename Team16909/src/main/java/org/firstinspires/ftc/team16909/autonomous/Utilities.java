package org.firstinspires.ftc.team16909.autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team16909.hardware.FettucineHardware;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Utilities
{
    FettucineHardware hardware;
    ElapsedTime time = new ElapsedTime();
    public Utilities(FettucineHardware hardware)
    {
        this.hardware = hardware;
        resetEncoderModes(hardware);
    }

    public void resetEncoderModes(FettucineHardware hardware)
    {
        for (DcMotorEx motor : hardware.driveMotors)
        {
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        for (DcMotorEx motor : hardware.liftMotors)
        {
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        hardware.armMotorOne.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void moveLift(int position)
    {
        hardware.liftMotorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.liftMotorTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hardware.liftMotorOne.setTargetPosition(hardware.liftMotorOne.getCurrentPosition() + position);
        hardware.liftMotorTwo.setTargetPosition(hardware.liftMotorTwo.getCurrentPosition() + position);

        hardware.liftMotorOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.liftMotorTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hardware.liftMotorOne.setPower(1);
        hardware.liftMotorTwo.setPower(1);

    }

    public void moveArm(int position)
    {
        hardware.armMotorOne.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        hardware.armMotorOne.setTargetPosition(hardware.armMotorOne.getCurrentPosition() + position);
        hardware.armMotorOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.armMotorOne.setPower(.65);

        wait((int)(position * 1.5));
        hardware.armServoOne.setPower(0.0);

    }

    public void wait(int millis, Telemetry telemetry)
    {
        ElapsedTime waitTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (waitTime.time() < millis)
        {
            telemetry.addData("Status", "Waiting");
            telemetry.addData("Time Left", "" + (millis - waitTime.time()));
            telemetry.update();
        }
    }

    public void wait(int millis)
    {
        ElapsedTime waitTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (waitTime.time() < millis)
        {
            continue;
        }
    }

    public void outtake()
    {
        hardware.grabberServo.setPosition(0);
    }

    public void intake()
    {
        hardware.grabberServo.setPosition(1);
    }

    public void moveHand(int ms, int dir)
    {
        hardware.armServoTwo.setPower(-dir);
        hardware.armServoOne.setPower(dir);
        this.wait(ms);
        hardware.armServoOne.setPower(0);
        hardware.armServoTwo.setPower(0);
    }
}
