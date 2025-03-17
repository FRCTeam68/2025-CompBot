// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
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
  private Pose2d nearestReefSide = new Pose2d();
  private Pose2d nearestSourceSide = new Pose2d();

  public enum Side {
    Left,
    Middle,
    Right,
    Back
  }

  public ReefCentering(Drive drive) {
    m_drive = drive;
  }

  public Pose2d calculateNearestSide() {
    if (m_drive.isRedSide()) return m_drive.getPose().nearest(FieldPoses.redReefPoses);
    else return m_drive.getPose().nearest(FieldPoses.blueReefPoses);
  }

  public Pose2d calculateNearestSourceSide() {
    if (m_drive.isRedSide()) return m_drive.getPose().nearest(FieldPoses.redSourcePoses);
    else return m_drive.getPose().nearest(FieldPoses.blueSourcePoses);
  }

  private Pose2d calculatePath(Side side) {
    double x = nearestReefSide.getX();
    double y = nearestReefSide.getY();
    double rot = nearestReefSide.getRotation().getRadians();

    switch (side) {
      case Left:
        x -= FieldPoses.leftOffset * Math.sin(rot);
        y += FieldPoses.leftOffset * Math.cos(rot);
        break;
      case Right:
        x += FieldPoses.leftOffset * Math.sin(rot);
        y -= FieldPoses.leftOffset * Math.cos(rot);
        break;
      case Back:
        x = nearestSourceSide.getX();
        y = nearestSourceSide.getY();
        rot = nearestSourceSide.getRotation().getRadians();
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

    if (nearSide != nearestReefSide) return true;

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

    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            Constants.PathPlannerConstants.slowConstraints,
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
          nearestReefSide = calculateNearestSide();
          nearestSourceSide = calculateNearestSourceSide();

          Pose2d scoringPosition = calculatePath(side);
          Command pathCommand = getPathFromWaypoint(scoringPosition);

          return pathCommand;
          // return m_drive.driveToPose(scoringPosition,
          // Constants.PathPlannerConstants.slowConstraints, 0);

        },
        Set.of(m_drive));
  }
}
