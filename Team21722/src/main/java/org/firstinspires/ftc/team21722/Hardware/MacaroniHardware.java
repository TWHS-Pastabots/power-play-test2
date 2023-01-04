package org.firstinspires.ftc.team21722.Hardware;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.internal.system.Assert;


public class MacaroniHardware {
    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightRear = null;
    public DcMotorEx[] motors;

    //supplementary motor
    public DcMotorEx liftArm = null;
    public Servo clawServo = null;


    public void init(HardwareMap hardwaremap)
    {
        Assert.assertNotNull(hardwaremap);
        initializeDriveMotors(hardwaremap);
        initializeSupplementMotor(hardwaremap);
    }

    public void initializeDriveMotors(HardwareMap hardwaremap) {
        leftFront = hardwaremap.get(DcMotorEx.class, MacaroniIds.LEFT_FRONT_MOTOR);
        rightFront = hardwaremap.get(DcMotorEx.class, MacaroniIds.RIGHT_FRONT_MOTOR);
        leftRear = hardwaremap.get(DcMotorEx.class, MacaroniIds.LEFT_REAR_MOTOR);
        rightRear = hardwaremap.get(DcMotorEx.class, MacaroniIds.RIGHT_REAR_MOTOR);


        motors = new DcMotorEx[]{leftFront, rightRear, leftRear, rightFront};


        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx motor : motors) {
            motor.setPower(0.0);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }


    }

    public void initializeSupplementMotor(HardwareMap hardwaremap)
    {
        liftArm = hardwaremap.get(DcMotorEx.class, MacaroniIds.LIFT_ARM_MOTOR);

        liftArm.setDirection(DcMotorSimple.Direction.FORWARD);

        liftArm.setPower(0.0);
        liftArm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        clawServo = hardwaremap.get(Servo.class, MacaroniIds.CLAW_SERVO);




    }
}
