package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

public class PathFollower {
    final int MAX_VELOCITY = 4; //max path following velocity (m/s)
    final int MAX_ACCEL = 3; //max path following acceleration
    static PathPlannerTrajectory path;
    static FollowPathWithEvents followCommand;
    //TODO: Move command factory from drivetrain to here.
    //Implement events.
}
