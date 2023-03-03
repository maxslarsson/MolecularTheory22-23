package org.firstinspires.ftc.teamcode.sequence;

@FunctionalInterface
public interface RunFunctionSegment extends BaseSegment {
    void run();
    default boolean shouldContinueToNextSegment() {
        run();
        return true;
    }
}