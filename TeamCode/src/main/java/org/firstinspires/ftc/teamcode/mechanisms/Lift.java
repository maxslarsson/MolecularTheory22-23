package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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

@Config
public class Lift {
    public static PIDCoefficients INTAKE_PID = new PIDCoefficients(.009, 0, 0.0002);
    public static double kV = 0;
    public static double kA = 0;
    public static double kStatic = 0;
    public static double kG = 0.0002;

    public static double MAX_VEL = 2000;
    public static double MAX_ACCEL = 2000;
    public static double MAX_JERK = 0;  // Jerk isn't used if it's 0, but it might end up being necessary

    public ElapsedTime timer;
    public PIDFController leftController;
    public PIDFController rightController;
    public MotionProfile motionProfile;

    public Servo clawServo;
    public Servo clawRotationServo;
    public DcMotorEx leftMotor;
    public DcMotorEx rightMotor;

    public Lift(HardwareMap hardwareMap) {
        timer = new ElapsedTime();
        //win or else sad claudia
        clawServo = hardwareMap.get(Servo.class, "liftClawServo");
        clawRotationServo = hardwareMap.get(Servo.class, "liftClawRotationServo");
        leftMotor = hardwareMap.get(DcMotorEx.class, "liftLeftMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "liftRightMotor");

        rightMotor.setDirection(DcMotorEx.Direction.REVERSE);

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

    public void setClawRotation(double position) {
        clawRotationServo.setPosition(position);
    }

    public void setClawPosition(double position) {
        clawServo.setPosition(position);
    }

    public boolean finishedFollowingMotionProfile() {
        return timer.time() >= motionProfile.duration();
    }

    public double getCurrentMotorPosition() {
        return (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2.0;
    }

    public double getCurrentMotorVelocity() {
        return (leftMotor.getVelocity() + rightMotor.getVelocity()) / 2.0;
    }

    public double getGravityAdjustment() {
        return getCurrentMotorPosition() * kG;
    }

    public void followMotionProfile(double targetPosition) {
        followMotionProfileAsync(targetPosition);

        while (!Thread.currentThread().isInterrupted() && !finishedFollowingMotionProfile()) {
            stepController();
        }
    }

    public void followMotionProfileAsync(double targetPosition) {
        // Add bounds so that the lift can not go too high or too low
        if (targetPosition < 0) {
            targetPosition = 0;
        }

        double currentPosition = getCurrentMotorPosition();
        double currentVelocity = getCurrentMotorVelocity();
        motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(currentPosition, currentVelocity),
                new MotionState(targetPosition, 0, 0),
                MAX_VEL,
                MAX_ACCEL,
                MAX_JERK
        );

        timer.reset();
    }

    public void setPower(double power) {
        motionProfile = null;

        if (leftMotor.getCurrentPosition() <= 0 && power < 0) {
            leftMotor.setPower(0);
        } else {
            leftMotor.setPower(power + getGravityAdjustment());
        }

        if (rightMotor.getCurrentPosition() <= 0 && power < 0) {
            rightMotor.setPower(0);
        } else {
            rightMotor.setPower(power + getGravityAdjustment());
        }

        leftController.setTargetPosition(getCurrentMotorPosition());
        leftController.setTargetVelocity(0);
        leftController.setTargetAcceleration(0);

        rightController.setTargetPosition(getCurrentMotorPosition());
        rightController.setTargetVelocity(0);
        rightController.setTargetAcceleration(0);
    }

    public void stepController() {
        if (motionProfile != null) {
            MotionState state = motionProfile.get(timer.time());

            leftController.setTargetPosition(state.getX());
            leftController.setTargetVelocity(state.getV());
            leftController.setTargetAcceleration(state.getA());

            rightController.setTargetPosition(state.getX());
            rightController.setTargetVelocity(state.getV());
            rightController.setTargetAcceleration(state.getA());
        }

        double leftLiftPower = leftController.update(leftMotor.getCurrentPosition(), leftMotor.getVelocity());
        double rightLiftPower = rightController.update(rightMotor.getCurrentPosition(), rightMotor.getVelocity());

        // TODO: Comment out but make sure both approach 0
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("leftLiftPower", leftLiftPower);
        packet.put("rightLiftPower", rightLiftPower);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        leftMotor.setPower(leftLiftPower + getGravityAdjustment());
        rightMotor.setPower(rightLiftPower + getGravityAdjustment());
    }
}
