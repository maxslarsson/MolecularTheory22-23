package org.firstinspires.ftc.teamcode.trajectorysequence.trajectorysequencesegment;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;

import java.util.List;

public final class WaitTrajectorySequenceSegment extends TrajectorySequenceSegment {
    public WaitTrajectorySequenceSegment(Pose2d pose, double seconds, List<TrajectoryMarker> markers) {
        super(seconds, pose, pose, markers);
    }
}
