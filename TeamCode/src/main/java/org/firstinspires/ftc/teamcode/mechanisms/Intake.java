package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake {
    public Servo clawServo;
    public Servo leftServo;
    public Servo rightServo;

    public Intake(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "intakeClawServo");
        leftServo = hardwareMap.get(Servo.class, "intakeLeftServo");
        rightServo = hardwareMap.get(Servo.class, "intakeRightServo");

        rightServo.setDirection(Servo.Direction.REVERSE);
    }

    public void setClawPosition(double position) {
        clawServo.setPosition(position);
    }

    public void setArmPosition(double position) {
        leftServo.setPosition(position);
        rightServo.setPosition(position);
    }
}
