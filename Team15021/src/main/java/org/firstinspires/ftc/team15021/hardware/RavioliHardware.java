package org.firstinspires.ftc.team15021.hardware;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.Assert;

public class RavioliHardware
{
    public DcMotorEx leftFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx rightRear = null;
    public DcMotorEx[] motors;
    public DcMotorEx armMotor0 = null;
    public DcMotorEx armMotor1 = null;
    public DcMotorEx armMotor2 = null;
    public DcMotorEx armMotor3 = null;
    public Servo servo0 = null;
    public Servo servo1 = null;



    public void init(HardwareMap hardwareMap)
    {
        Assert.assertNotNull(hardwareMap);

        initializeDriveMotors(hardwareMap);
        initializeArmMotors(hardwareMap);
        initializeClawServos(hardwareMap);
    }

    private void initializeDriveMotors(HardwareMap hardwareMap)
    {
        leftFront = hardwareMap.get(DcMotorEx.class, RavioliIds.LEFT_FRONT_MOTOR);
        rightFront = hardwareMap.get(DcMotorEx.class, RavioliIds.RIGHT_FRONT_MOTOR);
        leftRear = hardwareMap.get(DcMotorEx.class, RavioliIds.LEFT_REAR_MOTOR);
        rightRear = hardwareMap.get(DcMotorEx.class, RavioliIds.RIGHT_REAR_MOTOR);

        motors = new DcMotorEx[]{leftFront, leftRear, rightFront, rightRear};

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx motor : motors)
        {
            motor.setPower(0.0);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

    }
    private void initializeArmMotors(HardwareMap hardwareMap)
    {
        armMotor0 = hardwareMap.get(DcMotorEx.class, RavioliIds.ARM_MOTOR_ZERO);
        armMotor1 = hardwareMap.get(DcMotorEx.class, RavioliIds.ARM_MOTOR_ONE);
        armMotor2 = hardwareMap.get(DcMotorEx.class, RavioliIds.ARM_MOTOR_TWO);
        armMotor3 = hardwareMap.get(DcMotorEx.class, RavioliIds.ARM_MOTOR_THREE);

        armMotor0.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor3.setDirection(DcMotorSimple.Direction.FORWARD);

        armMotor0.setPower(0.0);
        armMotor1.setPower(0.0);
        armMotor2.setPower(0.0);
        armMotor3.setPower(0.0);

        armMotor0.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armMotor3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        armMotor0.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        armMotor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private void initializeClawServos(HardwareMap hardwareMap)
    {
        servo0 = hardwareMap.get(Servo.class, RavioliIds.SERVO_MOTOR_ZERO);
        servo1 = hardwareMap.get(Servo.class, RavioliIds.SERVO_MOTOR_ONE);

        servo1.scaleRange(.45, .6);

        servo0.setDirection(Servo.Direction.REVERSE);
        servo1.setDirection(Servo.Direction.REVERSE);

    }
}
