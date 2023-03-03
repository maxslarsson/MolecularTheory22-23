package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;

@Config
public class MechanismConstants {
    public static double[] STACK_HEIGHTS = {
            3, // 1st cone
            150, // 2nd cone
            202, // 3rd cone
            305, // 4th cone
            403, // 5th cone
    };

    public static double HIGH_JUNCTION_HEIGHT = 2200;
    public static double MEDIUM_JUNCTION_HEIGHT = 1600;
    public static double LOW_JUNCTION_HEIGHT = 1000;
    public static double GROUND_JUNCTION_HEIGHT = 140;
    public static double BOTTOM = 0;

    public static double ON_JUNCTION_HEIGHT_CHANGE = 300;

    public static double INTAKE_CLAW_OPEN_POSITION = 0.33;
    public static double INTAKE_CLAW_CLOSED_POSITION = 0.48;
    public static double INTAKE_ARM_DOWN_POSITION = 0.68;
    public static double INTAKE_ARM_UP_POSITION = 0.23;
    public static double RIGHT_INTAKE_SERVO_OFFSET = -0.1;

    public static double LIFT_CLAW_OPEN_POSITION = 0.5;
    public static double LIFT_CLAW_CLOSED_POSITION = 0.35;
    public static double LIFT_CLAW_PLACING_CONE_ROTATION = 0.25;
    public static double LIFT_CLAW_PICKING_UP_CONE_POSITION = 0.75;
    public static double LIFT_HEIGHT_TO_ROTATE_CLAW = 400;
}
