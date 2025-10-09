// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.FieldPoses;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import java.util.Set;

public class ReefCentering {

  private final Drive m_drive;
  private static Pose2d nearestSide = new Pose2d();
  private static Side selectedSide;

  public enum Side {
    Left,
    Middle,
    Right,
    Back,
    Barge
  }

  public ReefCentering(Drive drive) {
    m_drive = drive;
  }

  public Pose2d calculateNearestSide() {
    Pose2d nearest;

    switch (selectedSide) {
      case Left, Middle, Right:
        nearest = calculateNearestReefSide();
        break;
      case Back:
        nearest = calculateNearestSourceSide();
        break;
      case Barge:
        nearest = calculateNearestBargeSide();
        break;
      default:
        nearest = new Pose2d();
        break;
    }
    return nearest;
  }

  public Pose2d calculateNearestReefSide() {
    if (m_drive.isRedSide()) return m_drive.getPose().nearest(FieldPoses.redReefPoses);
    else return m_drive.getPose().nearest(FieldPoses.blueReefPoses);
  }

  public Pose2d calculateNearestSourceSide() {
    if (m_drive.isRedSide()) return m_drive.getPose().nearest(FieldPoses.redSourcePoses);
    else return m_drive.getPose().nearest(FieldPoses.blueSourcePoses);
  }

  public Pose2d calculateNearestBargeSide() {
    if (m_drive.isRedSide()) return m_drive.getPose().nearest(FieldPoses.redBargePoses);
    else return m_drive.getPose().nearest(FieldPoses.blueBargePoses);
  }

  private Pose2d calculatePath() {
    double x = nearestSide.getX();
    double y = nearestSide.getY();
    double rot = nearestSide.getRotation().getRadians();

    switch (selectedSide) {
      case Left:
        x -= FieldPoses.leftOffset * Math.sin(rot);
        y += FieldPoses.leftOffset * Math.cos(rot);
        break;
      case Right:
        x += FieldPoses.leftOffset * Math.sin(rot);
        y -= FieldPoses.leftOffset * Math.cos(rot);
        break;
      case Back:
        // nothing to change
        break;
      case Barge:
        y = m_drive.getPose().getY();
        break;
      default:
        break;
    }

    Pose2d scoringPosition = new Pose2d(x, y, new Rotation2d(rot));
    // m_drive.setCenteringPose(scoringPosition);
    // return m_drive.driveToPose(scoringPosition, PathPlannerConstants.slowConstraints, 0.02);
    return scoringPosition;
  }

  public boolean haveConditionsChanged() {

    Pose2d nearSide = calculateNearestSide();

    if (nearSide != nearestSide) return true;

    return false;
  }

  private Command getPathFromWaypoint(Pose2d waypoint) {

    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(
                m_drive.getPose().getTranslation(),
                getPathVelocityHeading(m_drive.getFieldVelocity(), waypoint)),
            waypoint);

    if (waypoints.get(0).anchor().getDistance(waypoints.get(1).anchor()) < 0.01) {
      return Commands.print("Auto alignment too close to desired position to continue");
    }

    PathConstraints pathContraints;

    switch (selectedSide) {
      case Left, Middle, Right:
        pathContraints = Constants.PathPlannerConstants.slowConstraints;
        break;
      case Back:
        pathContraints = Constants.PathPlannerConstants.slowConstraints;
        break;
      case Barge:
        pathContraints = Constants.PathPlannerConstants.slowConstraints;
        break;
      default:
        pathContraints = Constants.PathPlannerConstants.defaultConstraints;
        break;
    }

    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            pathContraints,
            new IdealStartingState(m_drive.getVelocityMagnitude(), m_drive.getHeading()),
            new GoalEndState(0.0, waypoint.getRotation()));

    path.preventFlipping = true;

    return AutoBuilder.followPath(path);
  }

  /**
   * @param cs field relative chassis speeds
   * @return
   */
  private Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target) {
    if (m_drive.getVelocityMagnitude().in(MetersPerSecond) < 0.25) {
      var diff = target.minus(m_drive.getPose()).getTranslation();
      return (diff.getNorm() < 0.01)
          ? target.getRotation()
          : diff.getAngle(); // .rotateBy(Rotation2d.k180deg);
    }
    return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
  }

  public Command createPathCommand(Side side) {
    return Commands.defer(
        () -> {
          selectedSide = side;
          nearestSide = calculateNearestSide();

          Pose2d scoringPosition = calculatePath();
          Command pathCommand;
          if (side == Side.Left || side == Side.Right || side == Side.Middle) {
            // will allow left to right to left
            pathCommand = getPathFromWaypoint(scoringPosition);
          } else {
            // straight path to pose
            pathCommand = getPathFromPose(scoringPosition);
          }

          // return m_drive.align(
          //     new APTarget(scoringPosition).withEntryAngle(scoringPosition.getRotation()));
          return m_drive.driveToPose(
              scoringPosition, Constants.PathPlannerConstants.slowConstraints, 0);
        },
        Set.of(m_drive));
  }

  private Command getPathFromPose(Pose2d scoringPose) {

    PathConstraints pathContraints;

    switch (selectedSide) {
      case Left, Middle, Right:
        pathContraints = Constants.PathPlannerConstants.slowConstraints;
        break;
      case Back:
        pathContraints = Constants.PathPlannerConstants.slowConstraints;
        break;
      case Barge:
        pathContraints = Constants.PathPlannerConstants.defaultConstraints;
        break;
      default:
        pathContraints = Constants.PathPlannerConstants.defaultConstraints;
        break;
    }

    return AutoBuilder.pathfindToPose(
        scoringPose, pathContraints, 0.0 // Goal end velocity in meters/sec
        );
  }
}
