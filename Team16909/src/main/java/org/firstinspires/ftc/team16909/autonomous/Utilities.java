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
    public Utilities(HardwareMap hardwareMap)
    {
        hardware.init(hardwareMap);
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
}
