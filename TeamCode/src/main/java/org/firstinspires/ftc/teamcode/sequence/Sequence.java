package org.firstinspires.ftc.teamcode.sequence;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

public class Sequence {
    private ElapsedTime segmentsRuntime;
    private List<BaseSegment> segments;
    private int currentIndex = Integer.MAX_VALUE;

    public Sequence() {
        segmentsRuntime = new ElapsedTime();
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
        segments.add(() -> segmentsRuntime.seconds() >= seconds);
        return this;
    }

    public void start() {
        currentIndex = 0;
        segmentsRuntime.reset();
    }

    public boolean isDone() {
        return currentIndex >= segments.size();
    }

    public void update() {
        //coded by claudia:
        if (isDone()) {
            return;
        }

        BaseSegment currentSequence = segments.get(currentIndex);
        boolean shouldContinueToNextSequence = currentSequence.shouldContinueToNextSegment();
        if (shouldContinueToNextSequence) {
            currentIndex += 1;
            segmentsRuntime.reset();
        }
    }
}
