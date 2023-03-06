package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@Config
@TeleOp(group = "debug")
public class MotorDebugger extends OpMode {
    public static double MOTOR_POWER = 1;

    public SampleMecanumDrive drive;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad1.x) {
            drive.setMotorPowers(MOTOR_POWER, 0, 0, 0);
            telemetry.addLine("Running Motor: Front Left");
        } else if(gamepad1.y) {
            drive.setMotorPowers(0, 0, 0, MOTOR_POWER);
            telemetry.addLine("Running Motor: Front Right");
        } else if(gamepad1.b) {
            drive.setMotorPowers(0, 0, MOTOR_POWER, 0);
            telemetry.addLine("Running Motor: Rear Right");
        } else if(gamepad1.a) {
            drive.setMotorPowers(0, MOTOR_POWER, 0, 0);
            telemetry.addLine("Running Motor: Rear Left");
        } else {
            drive.setMotorPowers(0, 0, 0, 0);
            telemetry.addLine("Running Motor: None");
        }

        List<Double> motorPositions = drive.getWheelPositions();
        telemetry.addData("Left Front Motor Position (in)", motorPositions.get(0));
        telemetry.addData("Left Rear Motor Position (in)", motorPositions.get(1));
        telemetry.addData("Right Rear Motor Position (in)", motorPositions.get(2));
        telemetry.addData("Right Front Motor Position (in)", motorPositions.get(3));

        telemetry.addLine();

        List<Double> motorVelocities = drive.getWheelVelocities();
        telemetry.addData("Left Front Motor Velocity (in/s)", motorVelocities.get(0));
        telemetry.addData("Left Rear Motor Velocity (in/s)", motorVelocities.get(1));
        telemetry.addData("Right Rear Motor Velocity (in/s)", motorVelocities.get(2));
        telemetry.addData("Right Front Motor Velocity (in/s)", motorVelocities.get(3));
    }
}