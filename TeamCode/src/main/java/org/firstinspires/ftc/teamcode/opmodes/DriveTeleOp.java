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
import org.firstinspires.ftc.teamcode.intakeandlift.Intake;
import org.firstinspires.ftc.teamcode.intakeandlift.IntakeConstants;
import org.firstinspires.ftc.teamcode.intakeandlift.Lift;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.List;

@Config
@TeleOp(name = "Drive TeleOp")
public class DriveTeleOp extends OpMode {
    public static double DRIVER_SPEED_SCALAR = 0.7;
    public static double DRIVER_ROTATION_SCALAR = 0.05;
    public static double DRIVER_SLOW_MODE_SCALAR = 0.50;
    public static double DRIVER_CANCEL_SPRINT_THRESHOLD = 0.85;

    public static double INTAKE_SPEED_SCALAR = 0.45;
    public static double GUNNER_STICK_THRESHOLD = 0.04;

    public Gamepad previousGamepad1 = new Gamepad();
    public Gamepad previousGamepad2 = new Gamepad();

    public SampleMecanumDrive drive;
    public Intake intake;
    public Lift lift;

    private List<LynxModule> allHubs;

    private boolean driverSprintMode = false;
    private boolean driverSlowMode = false;

    private boolean intakeArmDown = true;
    private boolean intakeClawClosed = true;
    private boolean liftClawClosed = true;

    @Override
    public void init() {
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(PoseStorage.currentPose);

        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);

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
            driverSprintMode = true;
        }

        if (-gamepad1.left_stick_y < DRIVER_CANCEL_SPRINT_THRESHOLD) {
            driverSprintMode = false;
        }

        if (!previousGamepad1.a && gamepad1.a) {
            driverSlowMode = !driverSlowMode;
        }

        telemetry.addData("Sprint mode", driverSprintMode);
        telemetry.addData("Slow mode", driverSlowMode);

        Vector2d translationalInput = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x);

        if (!driverSprintMode) {
            translationalInput = translationalInput.times(DRIVER_SPEED_SCALAR);
        }

        Pose2d input = new Pose2d(translationalInput, -gamepad1.right_stick_x * DRIVER_ROTATION_SCALAR);

        if (driverSlowMode && !driverSprintMode) {
            input = input.times(DRIVER_SLOW_MODE_SCALAR);
        }

        drive.setWeightedDrivePower(input);

        // ---------------
        // Gunner controls
        // ---------------
        if (Math.abs(gamepad2.left_stick_y) > GUNNER_STICK_THRESHOLD) {
            lift.setPower(-gamepad2.left_stick_y * INTAKE_SPEED_SCALAR);
        } else {
            lift.stepController();
        }

        if (!previousGamepad2.a && gamepad2.a) {
            intakeClawClosed = !intakeClawClosed;
        }

        if (!previousGamepad2.b && gamepad2.b) {
            intakeArmDown = !intakeArmDown;
        }

        if (!previousGamepad2.y && gamepad2.y) {
            liftClawClosed = !liftClawClosed;
        }

        intake.setClawPosition(intakeClawClosed ? IntakeConstants.INTAKE_CLAW_CLOSED_POSITION : IntakeConstants.INTAKE_CLAW_OPEN_POSITION);
        intake.setArmPosition(intakeArmDown ? IntakeConstants.INTAKE_ARM_DOWN_POSITION : IntakeConstants.INTAKE_ARM_UP_POSITION);
        lift.setClawPosition(liftClawClosed ? IntakeConstants.LIFT_CLAW_CLOSED_POSITION : IntakeConstants.LIFT_CLAW_OPEN_POSITION);

        // Auto-rotate claw if lift is past the threshold position
        if (lift.getCurrentMotorPosition() > IntakeConstants.LIFT_HEIGHT_TO_ROTATE_CLAW) {
            lift.setClawRotation(IntakeConstants.LIFT_CLAW_PLACING_CONE_ROTATION);
        } else {
            lift.setClawRotation(IntakeConstants.LIFT_CLAW_PICKING_UP_CONE_POSITION);
        }

        if (!previousGamepad2.dpad_up && gamepad2.dpad_up) {
            lift.followMotionProfileAsync(IntakeConstants.HIGH_JUNCTION_HEIGHT);
        }

        if (!previousGamepad2.dpad_right && gamepad2.dpad_right) {
            lift.followMotionProfileAsync(IntakeConstants.MEDIUM_JUNCTION_HEIGHT);
        }

        if (!previousGamepad2.dpad_left && gamepad2.dpad_left) {
            lift.followMotionProfileAsync(IntakeConstants.LOW_JUNCTION_HEIGHT);
        }

        if (!previousGamepad2.dpad_down && gamepad2.dpad_down) {
            lift.followMotionProfileAsync(IntakeConstants.GROUND_JUNCTION_HEIGHT);
        }

        // ---------
        // Clean up
        // ---------
        drive.update();
        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);
    }
}
