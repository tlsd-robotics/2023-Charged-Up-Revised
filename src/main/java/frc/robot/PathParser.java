package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

interface TrajectoryCommandFactory {
    Command getTrajectoryCommand(PathPlannerTrajectory traj, boolean firstPath);
}

public class PathParser {
    private PathParser() {}

    static Command commands[];

    CommandBase commandFromTrajectoryPlaceholder(PathPlannerTrajectory trajectory) {
        return null;
    }

    public static CommandBase getQuickAuto(String [] pathInstructions, double maxSpeed, double maxAcceleration, TrajectoryCommandFactory pathFollower, HashMap<String, Command> EventMap) {
        ArrayList<Command> commands = new ArrayList<Command>();
        ArrayList<PathPoint> points = new ArrayList<PathPoint>();

        for (int i = 0; i < pathInstructions.length; i++) {
            PathNode node = new PathNode(pathInstructions[i]);
            if (node.event != "") {
                points.add(new PathPoint(new Translation2d(node.x, node.y), Rotation2d.fromDegrees(node.angle)));
                PathPlannerTrajectory trajectory = PathPlanner.generatePath(new PathConstraints(maxSpeed, maxAcceleration), points);
                commands.add(pathFollower.getTrajectoryCommand(trajectory, false));
                commands.add(EventMap.get(node.event)); //TODO: Handle Null Command?
                points = new ArrayList<PathPoint>();
            }
            else {
                points.add(new PathPoint(new Translation2d(node.x, node.y), Rotation2d.fromDegrees(node.angle)));
            }
        }

        return null;
    }
}

class PathNode {
    double x;
    double y;
    double angle;
    String event;
    PathNode(double x, double y, double angle, String event) {
        this.x = x;
        this.y = y;
        this.angle = angle;
        this.event = event;
    }
    PathNode(String rawInstruction) {
        //TODO: Add parser for raw string.
    }
}