package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.asyncsequence.AsyncSequence;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeConstants;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.LiftConstants;
import org.firstinspires.ftc.teamcode.states.ClawPosition;
import org.firstinspires.ftc.teamcode.states.DrivingDirection;
import org.firstinspires.ftc.teamcode.states.IntakeArmPosition;
import org.firstinspires.ftc.teamcode.states.LiftClawRotation;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.vision.CameraController;

@Config
@Autonomous(preselectTeleOp = "Drive TeleOp")
public class RightCycleAuto extends LinearOpMode {
    public static Pose2d START_POSE = new Pose2d(36, -61.5, Math.toRadians(90));
    public static Pose2d PLACE_PRELOADED_CONE_POSE = new Pose2d(24,  -8.5, Math.toRadians(270));
    public static Pose2d PLACE_CONE_POSE = new Pose2d(26,  -3.8, Math.toRadians(90));
    public static Pose2d STACK_POSE = new Pose2d(60, -12, Math.toRadians(0));


    public static int CONES_TO_PLACE = 2;
    private int placedCones = 0;

    private enum State {
        PLACE_PRELOADED_CONE,
        PRELOADED_TO_STACK,
        STACK_TO_PLACE,
        PLACE_TO_STACK,
        PARK,
    }

    State currentState = State.PLACE_PRELOADED_CONE;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        CameraController cameraController = new CameraController(hardwareMap);
        AprilTagPipeline aprilTagPipeline = new AprilTagPipeline();
        cameraController.setPipeline(aprilTagPipeline);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(START_POSE);

        Intake intake = new Intake(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        // ------------
        // Set defaults
        // ------------
        intake.setClawPosition(IntakeConstants.CLAW_OPEN_POSITION);
        intake.setArmPosition(IntakeConstants.ARM_DRIVE_POSITION);
        lift.setClawPosition(LiftConstants.CLAW_CLOSED_POSITION);
        lift.setClawRotation(LiftConstants.CLAW_IN_ROTATION);

        // ----------------------
        // Create async sequences
        // ----------------------
        AsyncSequence clawTransferAsyncSequence = new AsyncSequence()
                .run(() -> intake.setArmPosition(IntakeConstants.ARM_IN_POSITION))
                .run(() -> lift.setClawRotation(LiftConstants.CLAW_IN_ROTATION))
                .run(() -> lift.setClawPosition(LiftConstants.CLAW_OPEN_POSITION))
                .waitSeconds(0.7)
                .run(() -> lift.setClawPosition(LiftConstants.CLAW_CLOSED_POSITION))
                .waitSeconds(0.1)
                .run(() -> intake.setClawPosition(IntakeConstants.CLAW_OPEN_POSITION))
                .waitSeconds(0.05)
                .run(() -> lift.setClawRotation(LiftConstants.CLAW_INTERMEDIATE_ROTATION))
                .waitSeconds(0.05)
                .run(() -> intake.setArmPosition(IntakeConstants.ARM_DRIVE_POSITION))
                .run(() -> lift.followMotionProfileAsync(LiftConstants.HIGH_JUNCTION_HEIGHT));

        AsyncSequence placeConeAsyncSequence = new AsyncSequence()
                .run(() -> lift.setClawRotation(LiftConstants.CLAW_OUT_ROTATION))
                .waitSeconds(0.07)
                .run(() -> lift.setClawPosition(LiftConstants.CLAW_OPEN_POSITION))
                .waitSeconds(0.07)
                .run(() -> lift.setClawRotation(LiftConstants.CLAW_INTERMEDIATE_ROTATION))
                .waitSeconds(0.07)
                .run(() -> lift.setClawPosition(LiftConstants.CLAW_CLOSED_POSITION))
                .waitSeconds(0.07)
                .run(() -> lift.setClawRotation(LiftConstants.CLAW_IN_ROTATION))
                .run(() -> intake.setArmPosition(IntakeConstants.ARM_OUT_POSITION));

        // ------------------
        // Build trajectories
        // ------------------
        TrajectorySequence placePreloadedCone = drive.trajectorySequenceBuilder(START_POSE)
                .lineToSplineHeading(new Pose2d(18, -60, Math.toRadians(90)))
                .splineToSplineHeading(new Pose2d(12, -36, Math.toRadians(0)), Math.toRadians(90))
                .addTemporalMarker(() -> lift.setClawRotation(LiftConstants.CLAW_OUT_ROTATION))
                .addTemporalMarker(() -> lift.followMotionProfileAsync(LiftConstants.HIGH_JUNCTION_HEIGHT))
                .splineToSplineHeading(PLACE_PRELOADED_CONE_POSE, Math.toRadians(0))
                .addTemporalMarker(() -> placeConeAsyncSequence.start())
                .build();

        TrajectorySequence preloadedToStack = drive.trajectorySequenceBuilder(PLACE_PRELOADED_CONE_POSE)
                .addTemporalMarker(() -> lift.followMotionProfileAsync(0))
                .addTemporalMarker(() -> intake.setArmPosition(IntakeConstants.STACK_HEIGHTS[5-placedCones-1]))
                .lineToLinearHeading(STACK_POSE)
                .build();

        // The claw should be at right height before this trajectory is run
        TrajectorySequence stackToPlace = drive.trajectorySequenceBuilder(STACK_POSE)
                .addTemporalMarker(() -> clawTransferAsyncSequence.start())
                .waitSeconds(0.2)
                .setReversed(true)
                .splineTo(PLACE_CONE_POSE.vec(), PLACE_CONE_POSE.getHeading() - Math.toRadians(180))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> placeConeAsyncSequence.start())
                .build();

        TrajectorySequence placeToStack = drive.trajectorySequenceBuilder(PLACE_CONE_POSE)
                .addTemporalMarker(() -> lift.followMotionProfileAsync(0))
                .addTemporalMarker(() -> intake.setArmPosition(IntakeConstants.STACK_HEIGHTS[5-placedCones-1]))
                .splineTo(STACK_POSE.vec(), STACK_POSE.getHeading())
                .build();

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Parking Position", aprilTagPipeline.getParkingPosition());
            telemetry.update();
            sleep(25);
        }

        TrajectorySequence placeToParking;

        switch (aprilTagPipeline.getParkingPosition()) {
            case ZONE1:
                placeToParking = drive.trajectorySequenceBuilder(PLACE_CONE_POSE)
                        .setTangent(PLACE_CONE_POSE.getHeading() - Math.toRadians(90))
                        .splineTo(new Vector2d(12, -12), Math.toRadians(180))
                        .build();
                break;
            default: // ZONE2 and NO_TAGS_SEEN
                placeToParking = drive.trajectorySequenceBuilder(PLACE_CONE_POSE)
                        .splineTo(new Vector2d(36, -12), Math.toRadians(270))
                        .build();
                break;
            case ZONE3:
                placeToParking = drive.trajectorySequenceBuilder(PLACE_CONE_POSE)
                        .splineTo(new Vector2d(60, -12), Math.toRadians(0))
                        .build();
                break;
        }

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case PLACE_PRELOADED_CONE:
                    if (!drive.isBusy() && placeConeAsyncSequence.isDone()) {
                        drive.followTrajectorySequenceAsync(placePreloadedCone);
                        currentState = State.PRELOADED_TO_STACK;
                    }
                    break;
                case PRELOADED_TO_STACK:
                    if (!drive.isBusy() && placeConeAsyncSequence.isDone()) {
                        drive.followTrajectorySequenceAsync(preloadedToStack);
                        currentState = State.STACK_TO_PLACE;
                    }
                    break;
                case STACK_TO_PLACE:
                    if (!drive.isBusy() && placeConeAsyncSequence.isDone()) {
                        drive.followTrajectorySequenceAsync(stackToPlace);
                        currentState = State.PLACE_TO_STACK;
                    }
                    break;
                case PLACE_TO_STACK:
                    if (!drive.isBusy() && placeConeAsyncSequence.isDone()) {
                        if (placedCones == CONES_TO_PLACE) {
                            drive.followTrajectorySequenceAsync(placeToParking);
                            currentState = State.PARK;
                        } else {
                            drive.followTrajectorySequenceAsync(placeToStack);
                            currentState = State.STACK_TO_PLACE;
                            placedCones++;
                        }
                    }
                    break;
                case PARK:
                    if (!drive.isBusy() && placeConeAsyncSequence.isDone()) {
                        requestOpModeStop();
                    }
                    break;
            }

            drive.update();
            lift.stepController();
            clawTransferAsyncSequence.update();
            placeConeAsyncSequence.update();

            PoseStorage.currentPose = drive.getPoseEstimate();

            telemetry.update();
        }
    }
}