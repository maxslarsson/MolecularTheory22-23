package org.firstinspires.ftc.teamcode.sequence;

@FunctionalInterface
public interface WaitUntilTrueSegment extends BaseSegment {
    boolean waitUntilTrue();
    default boolean shouldContinueToNextSegment() {
        return waitUntilTrue();
    }
}