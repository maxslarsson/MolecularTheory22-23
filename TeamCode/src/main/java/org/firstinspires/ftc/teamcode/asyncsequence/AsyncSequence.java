package org.firstinspires.ftc.teamcode.asyncsequence;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

public class AsyncSequence {
    private ElapsedTime segmentRuntime;
    private List<BaseSegment> segments;
    private int currentIndex = Integer.MAX_VALUE;

    public AsyncSequence() {
        segmentRuntime = new ElapsedTime();
        segments = new ArrayList<>();
    }

    public AsyncSequence run(RunFunctionSegment lambda) {
        segments.add(lambda);
        return this;
    }

    public AsyncSequence waitUntilTrue(WaitUntilTrueSegment lambda) {
        segments.add(lambda);
        return this;
    }

    public AsyncSequence waitSeconds(double seconds) {
        segments.add(() -> segmentRuntime.seconds() >= seconds);
        return this;
    }

    public void start() {
        currentIndex = 0;
        segmentRuntime.reset();
    }

    public boolean isDone() {
        return currentIndex >= segments.size();
    }

    public void update() {
        //coded by claudia:
        boolean shouldContinue = true;
        while (shouldContinue) {
            if (isDone()) {
                return;
            }

            BaseSegment currentSequence = segments.get(currentIndex);
            shouldContinue = currentSequence.shouldContinueToNextSegment();
            if (shouldContinue) {
                currentIndex += 1;
                segmentRuntime.reset();
            }
        }
    }
}
