package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SuperstructureVisualizer {
  private final String name;
  private final String logPathName = "RobotPose/";
  private static Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();

  public SuperstructureVisualizer(String name) {
    this.name = name;
  }

  public static void setRobotPoseSupplier(Supplier<Pose2d> supplier) {
    robotPoseSupplier = supplier;
  }

  public void update(
      double elevatorPositonMeters,
      double wristPositionRotations,
      boolean hasCoral,
      boolean hasAlgae) {

    Logger.recordOutput(
        logPathName + name + "/Elevator",
        new Pose3d[] {
          new Pose3d(0, 0, elevatorPositonMeters / 2, Rotation3d.kZero),
          new Pose3d(0, 0, elevatorPositonMeters, Rotation3d.kZero),
        });

    Pose3d wristPose =
        new Pose3d(
            0.28575,
            0.0,
            0.411 + elevatorPositonMeters,
            new Rotation3d(0.0, Units.rotationsToRadians(wristPositionRotations), 0.0));
    Logger.recordOutput(logPathName + name + "/Wrist", wristPose);

    if (hasCoral) {
      Logger.recordOutput(
          logPathName + name + "/Coral",
          new Pose3d[] {
            new Pose3d(
                    robotPoseSupplier.get().getX() + 0.18575,
                    robotPoseSupplier.get().getY() + Units.inchesToMeters(-.4),
                    0.69 + elevatorPositonMeters,
                    new Rotation3d(0, Units.degreesToRadians(19), 0))
                .rotateAround(
                    new Translation3d(robotPoseSupplier.get().getTranslation())
                        .plus(wristPose.getTranslation()),
                    wristPose.getRotation())
                .rotateAround(
                    new Translation3d(robotPoseSupplier.get().getTranslation()),
                    new Rotation3d(robotPoseSupplier.get().getRotation()))
          });
    } else {
      Logger.recordOutput(logPathName + name + "/Coral", new Pose3d[] {});
    }

    if (hasAlgae) {
      Logger.recordOutput(
          logPathName + name + "/Algae",
          new Pose3d[] {
            new Pose3d(
                    robotPoseSupplier.get().getX() + 0.01075,
                    robotPoseSupplier.get().getY() + Units.inchesToMeters(-.4),
                    0.445 + elevatorPositonMeters,
                    new Rotation3d(0, Units.degreesToRadians(19), 0))
                .rotateAround(
                    new Translation3d(robotPoseSupplier.get().getTranslation())
                        .plus(wristPose.getTranslation()),
                    wristPose.getRotation())
                .rotateAround(
                    new Translation3d(robotPoseSupplier.get().getTranslation()),
                    new Rotation3d(robotPoseSupplier.get().getRotation()))
          });
    } else {
      Logger.recordOutput(logPathName + name + "/Algae", new Pose3d[] {});
    }
  }
}
