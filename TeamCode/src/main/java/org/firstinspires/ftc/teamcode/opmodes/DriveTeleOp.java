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
import org.firstinspires.ftc.teamcode.mechanisms.MechanismConstants;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.sequence.Sequence;
import org.firstinspires.ftc.teamcode.states.ClawPosition;
import org.firstinspires.ftc.teamcode.states.IntakeArmPosition;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.List;

@Config
@TeleOp(name = "Drive TeleOp")
public class DriveTeleOp extends OpMode {
    //scalars set by claudia:
    public static double DRIVER_SPEED_SCALAR = 0.67;
    public static double DRIVER_ROTATION_SCALAR = 0.9;
    public static double DRIVER_SLOW_MODE_SCALAR = 0.50;
    public static double DRIVER_CANCEL_SPRINT_THRESHOLD = 0.85;

    public static double INTAKE_SPEED_SCALAR = 0.85;
    public static double GUNNER_STICK_THRESHOLD = 0.04;

    public Gamepad previousGamepad1 = new Gamepad();
    public Gamepad previousGamepad2 = new Gamepad();

    public SampleMecanumDrive drive;
    public Intake intake;
    public Lift lift;
    private Sequence clawTransferSequence;

    private List<LynxModule> allHubs;

    private boolean intakeIsForward = false;
    private boolean sprintMode = false;
    private boolean slowMode = false;

    private boolean liftIsEnabled = true;

    private IntakeArmPosition intakeArmPosition = IntakeArmPosition.ARM_DRIVE;
    private ClawPosition intakeClawPosition = ClawPosition.OPEN;
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
                .goLambda(() -> liftIsEnabled = false)
                .goLambda(() -> lift.followMotionProfileAsync(MechanismConstants.LIFT_HEIGHT_FOR_TRANSFER))
                .waitUntilTrue(() -> Math.abs(lift.getCurrentMotorPosition() - MechanismConstants.LIFT_HEIGHT_FOR_TRANSFER) < MechanismConstants.LIFT_HEIGHT_ERROR)
                .goLambda(() -> intakeArmPosition = IntakeArmPosition.ARM_UP)
                .goLambda(() -> lift.setClawPosition(MechanismConstants.LIFT_CLAW_OPEN_POSITION))
                .waitSeconds(2)
                .goLambda(() -> lift.setClawPosition(MechanismConstants.LIFT_CLAW_CLOSED_POSITION))
                .goLambda(() -> intake.setClawPosition(MechanismConstants.INTAKE_CLAW_OPEN_POSITION))
                .waitSeconds(.25)
                .goLambda(() -> intakeArmPosition = IntakeArmPosition.ARM_DRIVE)
                .waitSeconds(.5)
                .goLambda(() -> liftIsEnabled = true);

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
            sprintMode = true;
        }

        if (-gamepad1.left_stick_y < DRIVER_CANCEL_SPRINT_THRESHOLD) {
            sprintMode = false;
        }

        if (!previousGamepad1.a && gamepad1.a) {
            intakeIsForward = !intakeIsForward;
        }

        if (!previousGamepad1.b && gamepad1.b) {
            slowMode = !slowMode;
        }

        Vector2d translationalInput = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x);

        if (!intakeIsForward) {
            translationalInput = translationalInput.times(-1);
        }

        if (!sprintMode) {
            translationalInput = translationalInput.times(DRIVER_SPEED_SCALAR);
        }

        Pose2d input = new Pose2d(translationalInput, -gamepad1.right_stick_x * DRIVER_ROTATION_SCALAR);

        if (slowMode && !sprintMode) {
            input = input.times(DRIVER_SLOW_MODE_SCALAR);
        }

        drive.setWeightedDrivePower(input);

        // ---------------
        // Gunner controls
        // ---------------
        if (Math.abs(gamepad2.left_stick_y) > GUNNER_STICK_THRESHOLD && liftIsEnabled) {
            lift.setPower(-gamepad2.left_stick_y * INTAKE_SPEED_SCALAR);
        } else {
            lift.stepController();
        }

        boolean usingLiftMechanism = lift.getCurrentMotorPosition() > MechanismConstants.LIFT_HEIGHT_TO_CHANGE_TO_LIFT_MECHANISM;

        if (!previousGamepad2.a && gamepad2.a) {
            if (usingLiftMechanism) {
                switch (liftClawPosition) {
                    case OPEN:
                        liftClawPosition = ClawPosition.CLOSED;
                        break;
                    case CLOSED:
                        liftClawPosition = ClawPosition.OPEN;
                        break;
                }
            } else {
                switch (intakeClawPosition) {
                    case OPEN:
                        intakeClawPosition = ClawPosition.CLOSED;
                        break;
                    case CLOSED:
                        intakeClawPosition = ClawPosition.OPEN;
                        break;
                }
            }
        }

        // Use B to toggle arm position
        if (!previousGamepad2.b && gamepad2.b) {
            if (intakeArmPosition == IntakeArmPosition.ARM_UP) {
                intakeArmPosition = IntakeArmPosition.ARM_DOWN;
            } else if (intakeArmPosition == IntakeArmPosition.ARM_DOWN) {
                if (usingLiftMechanism) {
                    intakeArmPosition = IntakeArmPosition.ARM_DRIVE;
                } else {
                    intakeArmPosition = IntakeArmPosition.ARM_UP;
                }
            } else if (intakeArmPosition == IntakeArmPosition.ARM_DRIVE) {
                intakeArmPosition = IntakeArmPosition.ARM_DOWN;
            }
        }

        // Set intake arm position
        if (intakeArmPosition == IntakeArmPosition.ARM_UP) {
            intake.setArmPosition(MechanismConstants.INTAKE_ARM_UP_POSITION);
        } else if (intakeArmPosition == IntakeArmPosition.ARM_DOWN) {
            intake.setArmPosition(MechanismConstants.INTAKE_ARM_DOWN_POSITION);
        } else if (intakeArmPosition == IntakeArmPosition.ARM_DRIVE) {
            intake.setArmPosition(MechanismConstants.INTAKE_ARM_DRIVE_POSITION);
        }

        // Set intake claw position
        if (intakeClawPosition == ClawPosition.OPEN) {
            intake.setClawPosition(MechanismConstants.INTAKE_CLAW_OPEN_POSITION);
        } else if (intakeClawPosition == ClawPosition.CLOSED) {
            intake.setClawPosition(MechanismConstants.INTAKE_CLAW_CLOSED_POSITION);
        }

        // Set lift claw position
        if (liftClawPosition == ClawPosition.OPEN) {
            lift.setClawPosition(MechanismConstants.LIFT_CLAW_OPEN_POSITION);
        } else if (liftClawPosition == ClawPosition.CLOSED) {
            lift.setClawPosition(MechanismConstants.LIFT_CLAW_CLOSED_POSITION);
        }

        // Set lift claw rotation based on height of lift
        if (usingLiftMechanism) {
            lift.setClawRotation(MechanismConstants.LIFT_CLAW_PLACING_CONE_ROTATION);
        } else {
            lift.setClawRotation(MechanismConstants.LIFT_CLAW_PICKING_UP_CONE_POSITION);
            if (intakeArmPosition == IntakeArmPosition.ARM_DRIVE) {
                intakeArmPosition = IntakeArmPosition.ARM_DOWN;
            }
        }

        // Lift motion profile presets
        if (!previousGamepad2.dpad_up && gamepad2.dpad_up) {
            lift.followMotionProfileAsync(MechanismConstants.HIGH_JUNCTION_HEIGHT);
        }
        if (!previousGamepad2.dpad_right && gamepad2.dpad_right) {
            lift.followMotionProfileAsync(MechanismConstants.MEDIUM_JUNCTION_HEIGHT);
        }
        if (!previousGamepad2.dpad_left && gamepad2.dpad_left) {
            lift.followMotionProfileAsync(MechanismConstants.LOW_JUNCTION_HEIGHT);
        }
        if (!previousGamepad2.dpad_down && gamepad2.dpad_down) {
            lift.followMotionProfileAsync(MechanismConstants.GROUND_JUNCTION_HEIGHT);
        }

        // ---------------
        // Print telemetry
        // ---------------
        telemetry.addData("Intake is forward", intakeIsForward);
        telemetry.addData("Slow mode", slowMode);
        telemetry.addLine();
        telemetry.addData("Lift position", lift.getCurrentMotorPosition());
        telemetry.addData("Intake arm position", intakeArmPosition);

        // ---------
        // Clean up
        // ---------
        drive.update();
        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);
    }
}
