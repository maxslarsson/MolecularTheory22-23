package org.firstinspires.ftc.teamcode.intakeandlift;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Lift {
    public static PIDCoefficients INTAKE_PID = new PIDCoefficients(.009, 0, 0.0002);
    public static double kV = 0;
    public static double kA = 0;
    public static double kStatic = 0;

    public static double MAX_VEL = 1400;
    public static double MAX_ACCEL = 1400;
    public static double MAX_JERK = 0;  // Jerk isn't used if it's 0, but it might end up being necessary

    public ElapsedTime timer;
    public PIDFController leftController;
    public PIDFController rightController;
    public MotionProfile leftMotionProfile;
    public MotionProfile rightMotionProfile;

    public Servo clawServo;
    public DcMotorEx leftMotor;
    public DcMotorEx rightMotor;

    public Lift(HardwareMap hardwareMap) {
        timer = new ElapsedTime();

        clawServo = hardwareMap.get(Servo.class, "liftClawServo");
        leftMotor = hardwareMap.get(DcMotorEx.class, "liftLeftMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "liftRightMotor");

        leftMotor.setDirection(DcMotorEx.Direction.REVERSE);

        MotorConfigurationType leftMotorConfigurationType = leftMotor.getMotorType().clone();
        leftMotorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        leftMotor.setMotorType(leftMotorConfigurationType);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MotorConfigurationType rightMotorConfigurationType = rightMotor.getMotorType().clone();
        rightMotorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        rightMotor.setMotorType(rightMotorConfigurationType);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftController = new PIDFController(INTAKE_PID, kV, kA, kStatic);
        rightController = new PIDFController(INTAKE_PID, kV, kA, kStatic);
    }

    public void setClawPosition(double position) {
        clawServo.setPosition(position);
    }

    public boolean finishedFollowingMotionProfiles() {
        return timer.time() >= leftMotionProfile.duration() && timer.time() >= rightMotionProfile.duration();
    }

    public void followMotionProfiles(double targetPosition) {
        followMotionProfilesAsync(targetPosition);

        while (!Thread.currentThread().isInterrupted() && !finishedFollowingMotionProfiles()) {
            stepController();
        }
    }

    public void followMotionProfilesAsync(double targetPosition) {
        // Add bounds so that the lift can not go too high or too low
        if (targetPosition < 0) {
            targetPosition = 0;
        }

        leftMotionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(leftMotor.getCurrentPosition(), leftMotor.getVelocity()),
                new MotionState(targetPosition, 0, 0),
                MAX_VEL,
                MAX_ACCEL,
                MAX_JERK
        );

        rightMotionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(rightMotor.getCurrentPosition(), rightMotor.getVelocity()),
                new MotionState(targetPosition, 0, 0),
                MAX_VEL,
                MAX_ACCEL,
                MAX_JERK
        );

        timer.reset();
    }

    public void setPower(double power) {
        leftMotionProfile = null;
        rightMotionProfile = null;

        if (power < 0) {
            power = 0;
        }

        if (leftMotor.getCurrentPosition() <= 0) {
            leftMotor.setPower(0);
        } else {
            leftMotor.setPower(power);
        }

        leftController.setTargetPosition(leftMotor.getCurrentPosition());
        leftController.setTargetVelocity(0);
        leftController.setTargetAcceleration(0);

        if (rightMotor.getCurrentPosition() <= 0) {
            rightMotor.setPower(0);
        } else {
            rightMotor.setPower(power);
        }

        rightController.setTargetPosition(rightMotor.getCurrentPosition());
        rightController.setTargetVelocity(0);
        rightController.setTargetAcceleration(0);
    }

    public void stepController() {
        if (leftMotionProfile != null) {
            MotionState state = leftMotionProfile.get(timer.time());
            leftController.setTargetPosition(state.getX());
            leftController.setTargetVelocity(state.getV());
            leftController.setTargetAcceleration(state.getA());
        }

        if (rightMotionProfile != null) {
            MotionState state = rightMotionProfile.get(timer.time());
            rightController.setTargetPosition(state.getX());
            rightController.setTargetVelocity(state.getV());
            rightController.setTargetAcceleration(state.getA());
        }

        double leftPower = leftController.update(leftMotor.getCurrentPosition(), leftMotor.getVelocity());
        double rightPower = rightController.update(rightMotor.getCurrentPosition(), rightMotor.getVelocity());

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }
}
