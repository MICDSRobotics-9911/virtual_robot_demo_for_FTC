package org.firstinspires.ftc.teamcode.SampleEducationalPrograms.advanced.PIDToPoint;

import org.firstinspires.ftc.teamcode.SampleEducationalPrograms.advanced.MathUtils.Pose2d;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.stream.Collectors;

public class PurePursuitPath {
    private LinkedList<Waypoint> waypoints = new LinkedList<>();
    private int targetIndex = 1;
    private boolean finished;
    public static final double radius = 10;

    public PurePursuitPath(Waypoint... ws) {
        if (ws.length < 2) throw new IllegalArgumentException();
        Collections.addAll(waypoints, ws);
    }
    public PurePursuitPath(ArrayList<Pose2d> ws) {
        if (ws.size() < 2) throw new IllegalArgumentException();
        ArrayList<Waypoint> ws2 = new ArrayList<>();
        for (int i = 0; i < ws.size(); i++) {
            ws2.add(new Waypoint(ws.get(i), radius));
        }
        waypoints = new LinkedList<>(ws2);
    }

    public Pose2d update(Pose2d robot) {
        Waypoint prev = waypoints.get(targetIndex - 1);
        Waypoint target = waypoints.get(targetIndex);

        double distance = robot.distanceTo(target.getPoint());

        if (distance > target.getRadius()) {
            Point intersection = Util.lineCircleIntersection(prev.getPoint(), target.getPoint(), robot, target.getRadius());
            Pose2d targetPose;

            targetPose = new Pose2d(intersection, ((Pose2d) target.getPoint()).getHeading());

            return targetPose;
        } else {
            if (targetIndex == waypoints.size() - 1) {
                finished = true;
                return (Pose2d) waypoints.getLast().getPoint();
            } else {
                targetIndex++;
                return update(robot);
            }
        }
    }

    public boolean isFinished() {
        return finished;
    }

    public double getRadius() {
        return waypoints.get(targetIndex).getRadius();
    }
}
