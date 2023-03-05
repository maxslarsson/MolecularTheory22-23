package org.firstinspires.ftc.teamcode.asyncsequence;

@FunctionalInterface
public interface RunFunctionSegment extends BaseSegment {
    void run();
    default boolean shouldContinueToNextSegment() {
        run();
        return true;
    }
}