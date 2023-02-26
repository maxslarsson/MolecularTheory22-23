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

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.List;

@Config
@TeleOp
public class DriverTest extends OpMode {
    public static double DRIVER_SPEED_SCALAR = 0.7;
    public static double DRIVER_ROTATION_SCALAR = 0.05;
    public static double DRIVER_SLOW_MODE_SCALAR = 0.50;
    public static double DRIVER_CANCEL_SPRINT_THRESHOLD = 0.85;


    private boolean driverSprintMode = false;
    private boolean driverSlowMode = false;

    public Gamepad previousGamepad1 = new Gamepad();
    public Gamepad previousGamepad2 = new Gamepad();

    public SampleMecanumDrive drive;

    private List<LynxModule> allHubs;

    @Override
    public void init() {
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);

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

        drive.update();

        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);
    }
}
