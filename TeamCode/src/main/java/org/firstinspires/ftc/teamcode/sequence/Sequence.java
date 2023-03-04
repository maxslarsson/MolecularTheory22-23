package org.firstinspires.ftc.teamcode.sequence;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

public class Sequence {
    private ElapsedTime segmentRuntime;
    private List<BaseSegment> segments;
    private int currentIndex = Integer.MAX_VALUE;

    public Sequence() {
        segmentRuntime = new ElapsedTime();
        segments = new ArrayList<>();
    }

    public Sequence run(RunFunctionSegment lambda) {
        segments.add(lambda);
        return this;
    }

    public Sequence waitUntilTrue(WaitUntilTrueSegment lambda) {
        segments.add(lambda);
        return this;
    }

    public Sequence waitSeconds(double seconds) {
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
        if (isDone()) {
            return;
        }

        boolean shouldContinue = true;
        while (shouldContinue) {
            BaseSegment currentSequence = segments.get(currentIndex);
            shouldContinue = currentSequence.shouldContinueToNextSegment();
            if (shouldContinue) {
                currentIndex += 1;
                segmentRuntime.reset();
            }
        }
    }
}
