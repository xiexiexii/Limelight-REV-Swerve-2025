package frc.robot.subsystems.PathPlannerFixes;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Replanner {
    /**
     * Should the path be replanned at the start of path following if the robot is not already at the
     * starting point?
     */
    public final boolean enableInitialReplanning;
    /**
     * Should the path be replanned if the error grows too large or if a large error spike happens
     * while following the path?
     */
    public final boolean enableDynamicReplanning;
    /** The total error threshold, in meters, that will cause the path to be replanned */
    public final double dynamicReplanningTotalErrorThreshold;
    /** The error spike threshold, in meters, that will cause the path to be replanned */
    public final double dynamicReplanningErrorSpikeThreshold;

    /**
     * Create a path replanning configuration
     *
     * @param enableInitialReplanning Should the path be replanned at the start of path following if
     *     the robot is not already at the starting point?
     * @param enableDynamicReplanning Should the path be replanned if the error grows too large or if
     *     a large error spike happens while following the path?
     * @param dynamicReplanningTotalErrorThreshold The total error threshold, in meters, that will
     *     cause the path to be replanned
     * @param dynamicReplanningErrorSpikeThreshold The error spike threshold, in meters, that will
     *     cause the path to be replanned
     */
    public Replanner(
        boolean enableInitialReplanning,
        boolean enableDynamicReplanning,
        double dynamicReplanningTotalErrorThreshold,
        double dynamicReplanningErrorSpikeThreshold) {
        this.enableInitialReplanning = enableInitialReplanning;
        this.enableDynamicReplanning = enableDynamicReplanning;
        this.dynamicReplanningTotalErrorThreshold = dynamicReplanningTotalErrorThreshold;
        this.dynamicReplanningErrorSpikeThreshold = dynamicReplanningErrorSpikeThreshold;
    }

    /**
     * Create a path replanning configuration with default dynamic replanning error thresholds
     *
     * @param enableInitialReplanning Should the path be replanned at the start of path following if
     *     the robot is not already at the starting point?
     * @param enableDynamicReplanning Should the path be replanned if the error grows too large or if
     *     a large error spike happens while following the path?
     */
    public Replanner(boolean enableInitialReplanning, boolean enableDynamicReplanning) {
        this(enableInitialReplanning, enableDynamicReplanning, 1.0, 0.25);
    }

    public PathPlannerPath replan(PathPlannerPath path, Pose2d startingPose, ChassisSpeeds currentSpeeds) {
        ChassisSpeeds currentFieldRelativeSpeeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                currentSpeeds, startingPose.getRotation().unaryMinus());

        Translation2d robotNextControl = null;
        double linearVel =
            Math.hypot(
                currentFieldRelativeSpeeds.vxMetersPerSecond,
                currentFieldRelativeSpeeds.vyMetersPerSecond);
        if (linearVel > 0.1) {
        double stoppingDistance =
            Math.pow(linearVel, 2) / (2 * path.getGlobalConstraints().maxAccelerationMPSSq());

        Rotation2d heading =
            new Rotation2d(
                currentFieldRelativeSpeeds.vxMetersPerSecond,
                currentFieldRelativeSpeeds.vyMetersPerSecond);
        robotNextControl =
            startingPose.getTranslation().plus(new Translation2d(stoppingDistance / 2.0, heading));
        }

        int closestPointIdx = 0;
        Translation2d comparePoint =
            (robotNextControl != null) ? robotNextControl : startingPose.getTranslation();
        double closestDist = positionDelta(comparePoint, getPoint(path, closestPointIdx).position);

        for (int i = 1; i < path.numPoints(); i++) {
        double d = positionDelta(comparePoint, getPoint(path, i).position);

        if (d < closestDist) {
            closestPointIdx = i;
            closestDist = d;
        }
        }

        if (closestPointIdx == path.numPoints() - 1) {
        Rotation2d heading = getPoint(path, path.numPoints() - 1).position.minus(comparePoint).getAngle();

        if (robotNextControl == null) {
            robotNextControl =
                startingPose.getTranslation().plus(new Translation2d(closestDist / 3.0, heading));
        }

        Rotation2d endPrevControlHeading =
            getPoint(path, path.numPoints() - 1).position.minus(robotNextControl).getAngle();

        Translation2d endPrevControl =
            getPoint(path, path.numPoints() - 1)
                .position
                .minus(new Translation2d(closestDist / 3.0, endPrevControlHeading));
        // Throw out rotation targets, event markers, and constraint zones since we are skipping all
        // of the path
        // CHECK -- liam
        return PathPlannerPath.fromPathPoints(
              Arrays.asList(new PathPoint(startingPose.getTranslation()),
              new PathPoint(robotNextControl),
              new PathPoint(endPrevControl),
              getPoint(path, path.numPoints()-1)),
              path.getGlobalConstraints(),
              path.getGoalEndState());
        } else if ((closestPointIdx == 0 && robotNextControl == null)
            || (Math.abs(closestDist - startingPose.getTranslation().getDistance(getPoint(path, 0).position))
                    <= 0.25
                && linearVel < 0.1)) {
        double distToStart = startingPose.getTranslation().getDistance(getPoint(path, 0).position);

        Rotation2d heading = getPoint(path, 0).position.minus(startingPose.getTranslation()).getAngle();
        robotNextControl =
            startingPose.getTranslation().plus(new Translation2d(distToStart / 3.0, heading));

        Rotation2d joinHeading =
            path.getAllPathPoints().get(0).position.minus(path.getAllPathPoints().get(1).position).getAngle();
        Translation2d joinPrevControl =
            getPoint(path, 0).position.plus(new Translation2d(distToStart / 2.0, joinHeading));

        if (path.getWaypoints().isEmpty()) {
            // We don't have any bezier points to reference
            PathSegment joinSegment =
                new PathSegment(
                    startingPose.getTranslation(),
                    robotNextControl,
                    joinPrevControl,
                    getPoint(path, 0).position,
                    false);
            List<PathPoint> replannedPoints = new ArrayList<>();
            replannedPoints.addAll(joinSegment.getSegmentPoints());
            replannedPoints.addAll(path.getAllPathPoints());

            return PathPlannerPath.fromPathPoints(replannedPoints, path.getGlobalConstraints(), path.getGoalEndState());
        } else {
            // We can use the bezier points
            List<Waypoint> replannedBezier = new ArrayList<>();
            replannedBezier.addAll(
                PathPlannerPath.waypointsFromPoses(
                    startingPose, 
                    new Pose2d(robotNextControl,path.getGoalEndState().rotation()), 
                    new Pose2d(joinPrevControl,path.getGoalEndState().rotation())));
            replannedBezier.addAll(path.getWaypoints());
            // keep all rotations, markers, and zones and increment waypoint pos by 1
            return new PathPlannerPath(
                replannedBezier,
                path.getRotationTargets().stream()
                    .map(
                        (target) ->
                            new RotationTarget(
                                target.position() + 1,
                                target.rotation()))
                    .collect(Collectors.toList()),
                Collections.emptyList(),
                path.getConstraintZones().stream()
                    .map(
                        (zone) ->
                            new ConstraintsZone(
                                zone.minPosition() + 1,
                                zone.maxPosition() + 1,
                                zone.constraints()))
                    .collect(Collectors.toList()),
                path.getEventMarkers().stream()
                    .map(
                        (marker) ->
                            new EventMarker("", marker.position() + 1, marker.command())) // CHECK -- liam
                    .collect(Collectors.toList()),
                path.getGlobalConstraints(),
                path.getIdealStartingState(),
                path.getGoalEndState(),
                false);
        }
        }

        int joinAnchorIdx = path.numPoints() - 1;
        for (int i = closestPointIdx; i < path.numPoints(); i++) {
        if (getPoint(path, i).distanceAlongPath
            >= getPoint(path, closestPointIdx).distanceAlongPath + closestDist) {
            joinAnchorIdx = i;
            break;
        }
        }

        Translation2d joinPrevControl = getPoint(path, closestPointIdx).position;
        Translation2d joinAnchor = getPoint(path, joinAnchorIdx).position;

        if (robotNextControl == null) {
        double robotToJoinDelta = startingPose.getTranslation().getDistance(joinAnchor);
        Rotation2d heading = joinPrevControl.minus(startingPose.getTranslation()).getAngle();
        robotNextControl =
            startingPose.getTranslation().plus(new Translation2d(robotToJoinDelta / 3.0, heading));
        }

        if (joinAnchorIdx == path.numPoints() - 1) {
        // Throw out rotation targets, event markers, and constraint zones since we are skipping all
        // of the path
        return new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(
                startingPose, 
                new Pose2d(robotNextControl,startingPose.getRotation()), 
                new Pose2d(joinPrevControl,getPoint(path, closestPointIdx).rotationTarget.rotation()), 
                new Pose2d(joinAnchor,getPoint(path, joinAnchorIdx).rotationTarget.rotation())), // CHECK -- liam
            path.getGlobalConstraints(),
            path.getIdealStartingState(),
            path.getGoalEndState(),
            path.isReversed());
        }

        if (path.getWaypoints().isEmpty()) {
        // We don't have any bezier points to reference
        PathSegment joinSegment =
            new PathSegment(
                startingPose.getTranslation(), robotNextControl, joinPrevControl, joinAnchor, false);
        List<PathPoint> replannedPoints = new ArrayList<>();
        replannedPoints.addAll(joinSegment.getSegmentPoints());
        replannedPoints.addAll(path.getAllPathPoints().subList(joinAnchorIdx, path.getAllPathPoints().size()));

        return PathPlannerPath.fromPathPoints(replannedPoints, path.getGlobalConstraints(), path.getGoalEndState());
        }

        // We can reference bezier points
        int nextWaypointIdx = (int) Math.ceil((joinAnchorIdx + 1) * PathSegment.RESOLUTION);
        int bezierPointIdx = nextWaypointIdx;
        double waypointDelta = joinAnchor.getDistance(path.getWaypoints().get(bezierPointIdx).anchor());

        Rotation2d joinHeading = joinAnchor.minus(joinPrevControl).getAngle();
        Translation2d joinNextControl =
            joinAnchor.plus(new Translation2d(waypointDelta / 3.0, joinHeading));

        Rotation2d nextWaypointHeading;
        if (bezierPointIdx == path.getWaypoints().size() - 1) {
        nextWaypointHeading =
            path.getWaypoints().get(bezierPointIdx - 1).anchor().minus(path.getWaypoints().get(bezierPointIdx).anchor()).getAngle();
        } else {
        nextWaypointHeading =
            path.getWaypoints().get(bezierPointIdx).anchor().minus(path.getWaypoints().get(bezierPointIdx + 1).anchor()).getAngle();
        }

        Translation2d nextWaypointPrevControl =
            path.getWaypoints()
                .get(bezierPointIdx)
                .anchor()
                .plus(new Translation2d(Math.max(waypointDelta / 3.0, 0.15), nextWaypointHeading));

        List<Waypoint> replannedBezier = new ArrayList<>();
        replannedBezier.addAll(
            PathPlannerPath.waypointsFromPoses(
                // CHECK -- liam
                startingPose,
                new Pose2d(robotNextControl,startingPose.getRotation()), 
                new Pose2d(joinPrevControl,rotationValue(getPoint(path, closestPointIdx).rotationTarget,startingPose)), 
                new Pose2d(joinAnchor,rotationValue(getPoint(path, joinAnchorIdx).rotationTarget,startingPose)),
                new Pose2d(joinNextControl, rotationValue(getPoint(path, joinAnchorIdx).rotationTarget,startingPose)),
                new Pose2d(nextWaypointPrevControl, rotationValue(getPoint(path, bezierPointIdx).rotationTarget,startingPose))));
        replannedBezier.addAll(path.getWaypoints().subList(bezierPointIdx, path.getWaypoints().size()));

        double segment1Length = 0;
        Translation2d lastSegment1Pos = startingPose.getTranslation();
        double segment2Length = 0;
        Translation2d lastSegment2Pos = joinAnchor;

        for (double t = PathSegment.RESOLUTION; t < 1.0; t += PathSegment.RESOLUTION) {
        Translation2d p1 =
            GeometryUtil.cubicLerp(
                startingPose.getTranslation(), robotNextControl, joinPrevControl, joinAnchor, t);
        Translation2d p2 =
            GeometryUtil.cubicLerp(
                joinAnchor,
                joinNextControl,
                nextWaypointPrevControl,
                path.getWaypoints().get(bezierPointIdx).anchor(),
                t);

        segment1Length += positionDelta(lastSegment1Pos, p1);
        segment2Length += positionDelta(lastSegment2Pos, p2);

        lastSegment1Pos = p1;
        lastSegment2Pos = p2;
        }

        double segment1Pct = segment1Length / (segment1Length + segment2Length);

        List<RotationTarget> mappedTargets = new ArrayList<>();
        List<ConstraintsZone> mappedZones = new ArrayList<>();
        List<EventMarker> mappedMarkers = new ArrayList<>();

        for (RotationTarget t : path.getRotationTargets()) {
        if (t.position() >= nextWaypointIdx) {
            mappedTargets.add(
                new RotationTarget(
                    t.position() - nextWaypointIdx + 2, t.rotation()));
        } else if (t.position() >= nextWaypointIdx - 1) {
            double pct = t.position() - (nextWaypointIdx - 1);
            mappedTargets.add(
                new RotationTarget(mapPct(pct, segment1Pct), t.rotation()));
        }
        }

        for (ConstraintsZone z : path.getConstraintZones()) {
        double minPos = 0;
        double maxPos = 0;

        if (z.minPosition() >= nextWaypointIdx) {
            minPos = z.minPosition() - nextWaypointIdx + 2;
        } else if (z.minPosition() >= nextWaypointIdx - 1) {
            double pct = z.minPosition() - (nextWaypointIdx - 1);
            minPos = mapPct(pct, segment1Pct);
        }

        if (z.maxPosition() >= nextWaypointIdx) {
            maxPos = z.maxPosition() - nextWaypointIdx + 2;
        } else if (z.maxPosition() >= nextWaypointIdx - 1) {
            double pct = z.maxPosition() - (nextWaypointIdx - 1);
            maxPos = mapPct(pct, segment1Pct);
        }

        if (maxPos > 0) {
            mappedZones.add(new ConstraintsZone(minPos, maxPos, z.constraints()));
        }
        }

        for (EventMarker m : path.getEventMarkers()) {
        if (m.endPosition() >= nextWaypointIdx) {
            mappedMarkers.add(
                new EventMarker("", m.endPosition() - nextWaypointIdx + 2, m.command())); // CHECK -- liam
        } else if (m.endPosition() >= nextWaypointIdx - 1) {
            double pct = m.endPosition() - (nextWaypointIdx - 1);
            mappedMarkers.add(new EventMarker("", mapPct(pct, segment1Pct), m.command())); // CHECK -- liam
        }
        }

        // Throw out everything before nextWaypointIdx - 1, map everything from nextWaypointIdx -
        // 1 to nextWaypointIdx on to the 2 joining segments (waypoint rel pos within old segment = %
        // along distance of both new segments)
        return new PathPlannerPath(
            replannedBezier,
            mappedTargets,
            Collections.emptyList(),
            mappedZones,
            mappedMarkers,
            path.getGlobalConstraints(),
            path.getIdealStartingState(),
            path.getGoalEndState(),
            false);
  }
  public PathPoint getPoint(PathPlannerPath path, int index) {
    return path.getAllPathPoints().get(index);
  }
  private Rotation2d rotationValue(RotationTarget rotationTarget, Pose2d startingPose){
    if (rotationTarget == null){
        return startingPose.getRotation();
    }
    else {
        return rotationTarget.rotation();
    }
  }
  private static double positionDelta(Translation2d a, Translation2d b) {
    Translation2d delta = a.minus(b);

    return Math.abs(delta.getX()) + Math.abs(delta.getY());
  }
  private static double mapPct(double pct, double seg1Pct) {
    double mappedPct;
    if (pct <= seg1Pct) {
      // Map to segment 1
      mappedPct = pct / seg1Pct;
    } else {
      // Map to segment 2
      mappedPct = 1 + ((pct - seg1Pct) / (1.0 - seg1Pct));
    }

    // Round to nearest resolution step
    return Math.round(mappedPct * (1.0 / PathSegment.RESOLUTION)) / (1.0 / PathSegment.RESOLUTION);
  }
}
