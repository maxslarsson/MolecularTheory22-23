package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IntakeConstants {
    public static double CLAW_OPEN_POSITION = 0.33;
    public static double CLAW_CLOSED_POSITION = 0.48;
    public static double ARM_IN_POSITION = 0.05;
    public static double ARM_DRIVE_POSITION = 0.25;
    public static double ARM_OUT_POSITION = 0.58;

    public static double[] STACK_HEIGHTS = {
            0.58, // 1st cone
            0.54, // 2nd cone
            0.5, // 3rd cone
            0.46, // 4th cone
            0.42, // 5th cone
    };
}
