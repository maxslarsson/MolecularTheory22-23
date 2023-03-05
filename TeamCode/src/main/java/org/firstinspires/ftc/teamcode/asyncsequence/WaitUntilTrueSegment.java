package org.firstinspires.ftc.teamcode.asyncsequence;

@FunctionalInterface
public interface WaitUntilTrueSegment extends BaseSegment {
    boolean waitUntilTrue();
    default boolean shouldContinueToNextSegment() {
        return waitUntilTrue();
    }
}