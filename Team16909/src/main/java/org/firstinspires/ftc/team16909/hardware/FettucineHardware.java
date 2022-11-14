package org.firstinspires.ftc.team16909.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.internal.system.Assert;

public class FettucineHardware
{
    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightRear = null;
    public DcMotorEx liftMotorOne = null;
    public DcMotorEx liftMotorTwo = null;
    public DcMotorEx armMotorOne = null;
    public CRServo armServoOne = null;
    public CRServo armServoTwo = null;
    public DcMotorEx[] driveMotors;
    public DcMotorEx[] liftMotors;

    public void init(HardwareMap hardwareMap)
    {
        Assert.assertNotNull(hardwareMap);
        initializeDriveMotors(hardwareMap);
        initializeLiftMotors(hardwareMap);
        initializeArmMotors(hardwareMap);
    }

    public void initializeDriveMotors(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, FettucineIds.LEFT_FRONT_MOTOR);
        rightFront = hardwareMap.get(DcMotorEx.class, FettucineIds.RIGHT_FRONT_MOTOR);
        leftRear = hardwareMap.get(DcMotorEx.class, FettucineIds.LEFT_REAR_MOTOR);
        rightRear = hardwareMap.get(DcMotorEx.class, FettucineIds.RIGHT_REAR_MOTOR);

        driveMotors = new DcMotorEx[] {leftFront, leftRear, rightFront, rightRear};

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx motor : driveMotors) {
            motor.setPower(0.0);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void initializeLiftMotors(HardwareMap hardwareMap)
    {
        liftMotorOne = hardwareMap.get(DcMotorEx.class, FettucineIds.LIFT_MOTOR_ONE);
        liftMotorTwo = hardwareMap.get(DcMotorEx.class, FettucineIds.LIFT_MOTOR_TWO);

        liftMotorOne.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotorTwo.setDirection(DcMotorSimple.Direction.REVERSE);


        liftMotors = new DcMotorEx[] {liftMotorOne, liftMotorTwo};

        for (DcMotorEx motor : liftMotors)
        {
            motor.setPower(0.0);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void initializeArmMotors (HardwareMap hardwareMap)
    {
        armMotorOne = hardwareMap.get(DcMotorEx.class, FettucineIds.ARM_MOTOR_ONE);
        armMotorOne.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotorOne.setPower(0.0);
        armMotorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorOne.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        armServoOne = hardwareMap.get(CRServo.class, FettucineIds.ARM_SERVO_ONE);
        armServoTwo = hardwareMap.get(CRServo.class, FettucineIds.ARM_SERVO_TWO);

        armServoOne.resetDeviceConfigurationForOpMode();
        armServoTwo.resetDeviceConfigurationForOpMode();
    }
}
