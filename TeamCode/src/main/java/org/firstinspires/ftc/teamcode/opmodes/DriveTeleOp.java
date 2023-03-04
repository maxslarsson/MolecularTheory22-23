package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeConstants;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.LiftConstants;
import org.firstinspires.ftc.teamcode.sequence.Sequence;
import org.firstinspires.ftc.teamcode.states.ClawPosition;
import org.firstinspires.ftc.teamcode.states.DrivingDirection;
import org.firstinspires.ftc.teamcode.states.DrivingMode;
import org.firstinspires.ftc.teamcode.states.LiftClawRotation;
import org.firstinspires.ftc.teamcode.states.IntakeArmPosition;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.List;

@Config
@TeleOp(name = "Drive TeleOp")
public class DriveTeleOp extends OpMode {
    //scalars set by claudia:
    public static double SPRINT_SPEED_SCALAR = 1;
    public static double SPRINT_ROTATION_SCALAR = 0.75;
    public static double NORMAL_SPEED_SCALAR = 0.78;
    public static double NORMAL_ROTATION_SCALAR = 0.65;
    public static double SLOW_MODE_SPEED_SCALAR = 0.6;
    public static double SLOW_MODE_ROTATION_SCALAR = 0.6;

    public static double DRIVER_CANCEL_SPRINT_THRESHOLD = 0.85;

    public static double INTAKE_SPEED_SCALAR = 1;
    public static double INTAKE_DOWN_SCALAR = 0.4;
    public static double GUNNER_STICK_THRESHOLD = 0.04;

    public static int DRIVER_RUMBLE_DURATION_MS = 300;

    public SampleMecanumDrive drive;
    public Intake intake;
    public Lift lift;
    private Sequence clawTransferSequence;
    private Sequence placeConeSequence;

    private Gamepad previousGamepad1;
    private Gamepad previousGamepad2;
    private List<LynxModule> allHubs;

    private DrivingDirection drivingDirection = DrivingDirection.INTAKE;
    private DrivingMode drivingMode = DrivingMode.NORMAL;
    private IntakeArmPosition intakeArmPosition = IntakeArmPosition.OUT;
    private ClawPosition intakeClawPosition = ClawPosition.OPEN;
    private LiftClawRotation liftClawRotation = LiftClawRotation.IN;
    private ClawPosition liftClawPosition = ClawPosition.CLOSED;

    @Override
    public void init() {
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(PoseStorage.currentPose);

        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);

        clawTransferSequence = new Sequence()
                .run(() -> drivingDirection = DrivingDirection.LIFT)
                .run(() -> gamepad1.rumble(DRIVER_RUMBLE_DURATION_MS))
                .run(() -> intakeArmPosition = IntakeArmPosition.IN)
                .run(() -> liftClawRotation = LiftClawRotation.IN)
                .run(() -> liftClawPosition = ClawPosition.OPEN)
                .waitSeconds(0.65)
                .run(() -> liftClawPosition = ClawPosition.CLOSED)
                .waitSeconds(0.07)
                .run(() -> intakeClawPosition = ClawPosition.OPEN)
                .waitSeconds(0.07)
                .run(() -> liftClawRotation = LiftClawRotation.INTERMEDIATE)
                .waitSeconds(0.07)
                .run(() -> intakeArmPosition = IntakeArmPosition.DRIVE);

        placeConeSequence = new Sequence()
                .run(() -> liftClawRotation = LiftClawRotation.OUT)
                .waitSeconds(0.07)
                .run(() -> liftClawPosition = ClawPosition.OPEN)
                .waitSeconds(0.07)
                .run(() -> liftClawRotation = LiftClawRotation.INTERMEDIATE)
                .run(() -> drivingDirection = DrivingDirection.INTAKE)
                .run(() -> gamepad1.rumble(DRIVER_RUMBLE_DURATION_MS))
                .waitSeconds(0.07)
                .run(() -> liftClawPosition = ClawPosition.CLOSED)
                .waitSeconds(0.07)
                .run(() -> liftClawRotation = LiftClawRotation.IN)
                .run(() -> intakeArmPosition = IntakeArmPosition.OUT);

        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void loop() {
        // Will run one bulk read per cycle, because the caches are being handled manually and cleared once a loop
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // ---------------
        // Driver controls
        // ---------------
        if (!previousGamepad1.left_stick_button && gamepad1.left_stick_button) {
            drivingMode = DrivingMode.SPRINT;
        }

        if (-gamepad1.left_stick_y < DRIVER_CANCEL_SPRINT_THRESHOLD) {
            drivingMode = DrivingMode.NORMAL;
        }

        if (!previousGamepad1.a && gamepad1.a) {
            if (drivingMode == DrivingMode.NORMAL) {
                drivingMode = DrivingMode.SLOW;
            } else if (drivingMode == DrivingMode.SLOW) {
                drivingMode = DrivingMode.NORMAL;
            }
        }

        if (!previousGamepad1.b && gamepad1.b) {
            if (drivingDirection == DrivingDirection.INTAKE) {
                drivingDirection = DrivingDirection.LIFT;
                gamepad1.rumble(DRIVER_RUMBLE_DURATION_MS);
            } else if (drivingDirection == DrivingDirection.LIFT) {
                drivingDirection = DrivingDirection.INTAKE;
                gamepad1.rumble(DRIVER_RUMBLE_DURATION_MS);
            }
        }

        Vector2d translationalInput = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x);

        if (drivingDirection == DrivingDirection.LIFT) {
            translationalInput = translationalInput.times(-1);
        }

        Pose2d input = new Pose2d();

        if (drivingMode == DrivingMode.SPRINT) {
            input = new Pose2d(translationalInput.times(SPRINT_SPEED_SCALAR), -gamepad1.right_stick_x * SPRINT_ROTATION_SCALAR);
        } else if (drivingMode == DrivingMode.NORMAL) {
            input = new Pose2d(translationalInput.times(NORMAL_SPEED_SCALAR), -gamepad1.right_stick_x * NORMAL_ROTATION_SCALAR);
        } else if (drivingMode == DrivingMode.SLOW) {
            input = new Pose2d(translationalInput.times(SLOW_MODE_SPEED_SCALAR), -gamepad1.right_stick_x * SLOW_MODE_ROTATION_SCALAR);
        }

        drive.setWeightedDrivePower(input);

        // ---------------
        // Gunner controls
        // ---------------
        if (!clawTransferSequence.isDone()) {
            lift.freeFloat();
        } else if (Math.abs(gamepad2.left_stick_y) > GUNNER_STICK_THRESHOLD) {
            double liftPower = -gamepad2.left_stick_y * INTAKE_SPEED_SCALAR;
            if (liftPower < 0) liftPower *= INTAKE_DOWN_SCALAR;
            lift.setPower(liftPower);
        } else {
            lift.stepController();
        }

        if (!previousGamepad2.a && gamepad2.a && placeConeSequence.isDone()) {
            if (liftClawRotation == LiftClawRotation.INTERMEDIATE) {
                placeConeSequence.start();
            } else {
                if (intakeClawPosition == ClawPosition.OPEN) {
                    intakeClawPosition = ClawPosition.CLOSED;
                } else if (intakeClawPosition == ClawPosition.CLOSED) {
                    intakeClawPosition = ClawPosition.OPEN;
                }
            }
        }

        // Use B to toggle arm position
        if (!previousGamepad2.b && gamepad2.b && clawTransferSequence.isDone() && intakeClawPosition == ClawPosition.CLOSED) {
            clawTransferSequence.start();
        }

        // Set intake arm position
        if (intakeArmPosition == IntakeArmPosition.IN) {
            intake.setArmPosition(IntakeConstants.ARM_IN_POSITION);
        } else if (intakeArmPosition == IntakeArmPosition.DRIVE) {
            intake.setArmPosition(IntakeConstants.ARM_DRIVE_POSITION);
        } else if (intakeArmPosition == IntakeArmPosition.OUT) {
            intake.setArmPosition(IntakeConstants.ARM_OUT_POSITION);
        }

        // Set intake claw position
        if (intakeClawPosition == ClawPosition.OPEN) {
            intake.setClawPosition(IntakeConstants.CLAW_OPEN_POSITION);
        } else if (intakeClawPosition == ClawPosition.CLOSED) {
            intake.setClawPosition(IntakeConstants.CLAW_CLOSED_POSITION);
        }

        // Set lift claw position
        if (liftClawPosition == ClawPosition.OPEN) {
            lift.setClawPosition(LiftConstants.CLAW_OPEN_POSITION);
        } else if (liftClawPosition == ClawPosition.CLOSED) {
            lift.setClawPosition(LiftConstants.CLAW_CLOSED_POSITION);
        }

        // Set lift claw rotation
        if (liftClawRotation == LiftClawRotation.IN) {
            lift.setClawRotation(LiftConstants.CLAW_IN_ROTATION);
        } else if (liftClawRotation == LiftClawRotation.INTERMEDIATE) {
            lift.setClawRotation(LiftConstants.CLAW_INTERMEDIATE_ROTATION);
        } else if (liftClawRotation == LiftClawRotation.OUT) {
            lift.setClawRotation(LiftConstants.CLAW_OUT_ROTATION);
        }

        // Lift motion profile presets
        if (!previousGamepad2.dpad_up && gamepad2.dpad_up) {
            lift.followMotionProfileAsync(LiftConstants.HIGH_JUNCTION_HEIGHT);
        }
        if (!previousGamepad2.dpad_right && gamepad2.dpad_right) {
            lift.followMotionProfileAsync(LiftConstants.MEDIUM_JUNCTION_HEIGHT);
        }
        if (!previousGamepad2.dpad_left && gamepad2.dpad_left) {
            lift.followMotionProfileAsync(LiftConstants.LOW_JUNCTION_HEIGHT);
        }
        if (!previousGamepad2.dpad_down && gamepad2.dpad_down) {
            lift.followMotionProfileAsync(LiftConstants.GROUND_JUNCTION_HEIGHT);
        }

        // ---------------
        // Print telemetry
        // ---------------
        telemetry.addData("Driving direction", drivingDirection);
        telemetry.addData("Driving mode", drivingMode);
        telemetry.addLine();
        telemetry.addData("Intake claw position", intakeClawPosition);
        telemetry.addData("Intake arm position", intakeArmPosition);
        telemetry.addData("Lift left motor position", lift.leftMotor.getCurrentPosition());
        telemetry.addData("Lift right motor position", lift.rightMotor.getCurrentPosition());
        telemetry.addData("Lift claw position", liftClawPosition);
        telemetry.addData("Lift claw rotation", liftClawRotation);

        // ---------
        // Clean up
        // ---------
        drive.update();
        clawTransferSequence.update();
        placeConeSequence.update();

        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);
    }
}
