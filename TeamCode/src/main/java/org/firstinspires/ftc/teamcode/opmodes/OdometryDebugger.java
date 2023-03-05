package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class OdometryDebugger extends OpMode {
    public StandardTrackingWheelLocalizer threeWheelOdometryLocalizer;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();
        threeWheelOdometryLocalizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);
    }

    @Override
    public void loop() {
        telemetry.addData("Left Encoder Position (ticks)", threeWheelOdometryLocalizer.leftEncoder.getCurrentPosition());
        telemetry.addData("Right Encoder Position (ticks)", threeWheelOdometryLocalizer.rightEncoder.getCurrentPosition());
        telemetry.addData("Back Encoder Position (ticks)", threeWheelOdometryLocalizer.backEncoder.getCurrentPosition());

        telemetry.addLine();

        List<Double> encoderPositions = threeWheelOdometryLocalizer.getWheelPositions();
        telemetry.addData("Left Encoder Position (in)", encoderPositions.get(0));
        telemetry.addData("Right Encoder Position (in)", encoderPositions.get(1));
        telemetry.addData("Back Encoder Position (in)", encoderPositions.get(2));

        telemetry.addLine();

        telemetry.addData("Left Encoder Velocity (ticks/s)", threeWheelOdometryLocalizer.leftEncoder.getRawVelocity());
        telemetry.addData("Right Encoder Velocity (ticks/s)", threeWheelOdometryLocalizer.rightEncoder.getRawVelocity());
        telemetry.addData("Back Encoder Velocity (ticks/s)", threeWheelOdometryLocalizer.backEncoder.getRawVelocity());

        telemetry.addLine();

        List<Double> encoderVelocities = threeWheelOdometryLocalizer.getWheelVelocities();
        telemetry.addData("Left Encoder Velocity (in/s)", encoderVelocities.get(0));
        telemetry.addData("Right Encoder Velocity (in/s)", encoderVelocities.get(1));
        telemetry.addData("Back Encoder Velocity (in/s)", encoderVelocities.get(2));
    }
}