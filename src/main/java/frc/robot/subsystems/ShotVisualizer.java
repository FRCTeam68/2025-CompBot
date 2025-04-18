package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.FieldConstants;
import frc.robot.util.LoggedTunableNumber;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShotVisualizer {
  private static final Transform3d launcherTransform =
      new Transform3d(0.35, 0, 0.8, new Rotation3d(0.0, Units.degreesToRadians(-55.0), 0.0));
  // private static final double shotSpeed = 5.0; // Meters per sec
  private static final double gravity = 9.8; // Meters per sec²
  private static Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();

  public static void setRobotPoseSupplier(Supplier<Pose2d> supplier) {
    robotPoseSupplier = supplier;
  }

  private static final Translation3d blueSpeaker = new Translation3d(0.225, 5.55, 2.1);
  private static final Translation3d redSpeaker = new Translation3d(16.317, 5.55, 2.1);

  private static LoggedTunableNumber tunePitch =
      new LoggedTunableNumber("Shot Visualizer/Pitch", 45);
  private static LoggedTunableNumber tuneShotSpeed =
      new LoggedTunableNumber("Shot Visualizer/Speed", 5);
  private static LoggedTunableNumber tuneYaw = new LoggedTunableNumber("Shot Visualizer/Yaw", 0);
  private static LoggedTunableNumber tuneTargetHeight =
      new LoggedTunableNumber("Shot Visualizer/Target Height", 0);

  public static Command shoot() {
    return new ScheduleCommand( // Branch off and exit immediately
        Commands.defer(
                () -> {
                  final double shotSpeed = tuneShotSpeed.get();
                  final Pose3d startPose =
                      new Pose3d(robotPoseSupplier.get()).transformBy(launcherTransform);
                  final boolean isRed =
                      DriverStation.getAlliance().isPresent()
                          && DriverStation.getAlliance().get().equals(Alliance.Red);
                  final Pose3d endPose =
                      new Pose3d(isRed ? redSpeaker : blueSpeaker, startPose.getRotation());

                  final double duration =
                      startPose.getTranslation().getDistance(endPose.getTranslation()) / shotSpeed;
                  final Timer timer = new Timer();
                  timer.start();
                  return Commands.run(
                          () -> {
                            Logger.recordOutput(
                                "NoteVisualizer",
                                new Pose3d[] {
                                  startPose.interpolate(endPose, timer.get() / duration)
                                });
                          })
                      .until(() -> timer.hasElapsed(duration))
                      .finallyDo(
                          () -> {
                            Logger.recordOutput("NoteVisualizer", new Pose3d[] {});
                          });
                },
                Set.of())
            .ignoringDisable(true));
  }

  public static Command shootParabula() {
    return new ScheduleCommand( // Branch off and exit immediately
        Commands.defer(
                () -> {
                  final double shotSpeed = tuneShotSpeed.get();
                  final Pose3d startPose = new Pose3d(1, 1, 0.5, new Rotation3d());
                  final double initalYaw =
                      Units.degreesToRadians(
                          tuneYaw.get()); // degrees: 0 is away from blue drive station
                  final double initalPitch =
                      Units.degreesToRadians(tunePitch.get()); // degrees from horizontal
                  final double targetHeight =
                      tuneTargetHeight.get(); // height to stop visualization
                  final Timer timer = new Timer();
                  final double Vx = shotSpeed * Math.cos(initalPitch) * Math.cos(initalYaw);
                  final double Vy = shotSpeed * Math.cos(initalPitch) * Math.sin(initalYaw);
                  final double Vz0 = shotSpeed * Math.sin(initalPitch);
                  final double duration =
                      getDurationParabula(startPose, Vx, Vy, Vz0, targetHeight, EndType.falling);
                  timer.start();
                  return Commands.run(
                          () -> {
                            Logger.recordOutput(
                                "ShotVisualizer",
                                new Pose3d[] {
                                  startPose.transformBy(
                                      new Transform3d(
                                          Vx * timer.get(),
                                          Vy * timer.get(),
                                          (Vz0 * timer.get())
                                              - (0.5 * gravity * Math.pow(timer.get(), 2)),
                                          new Rotation3d()))
                                });
                          })
                      .until(() -> timer.hasElapsed(duration))
                      .finallyDo(
                          () -> {
                            Logger.recordOutput("ShotVisualizer", new Pose3d[] {});
                          });
                },
                Set.of())
            .ignoringDisable(true));
  }

  private static enum EndType {
    perimeter,

    rising,

    falling
  }

  @SuppressWarnings("unused")
  private static double getDurationParabula(
      Pose3d startPose, double Vx, double Vy, double Vz0, double targetHeight, EndType endType) {
    // If starting height is less then 0
    // Return 0
    if (startPose.getZ() < 0) return 0;

    Double duration = null;
    Double duration0H = null;

    // Calculate duration for 0 target height
    duration0H =
        (Vz0 + Math.sqrt((2 * gravity * startPose.getZ()) - (2 * gravity * 0) + Math.pow(Vz0, 2)))
            / gravity;

    // Calculate duration for specified end type
    switch (endType) {
      case perimeter:
        Double durationX = null;
        Double durationY = null;
        if (Vx != 0)
          durationX =
              (Vx > 0)
                  ? (FieldConstants.fieldLength - startPose.getX()) / Vx
                  : (0 - startPose.getX()) / Vx;
        if (Vy != 0)
          durationY =
              (Vy > 0)
                  ? (FieldConstants.fieldWidth - startPose.getY()) / Vy
                  : (0 - startPose.getY()) / Vy;

        if (durationX == null && durationY == null) {
          duration = null;
        } else if (durationX != null && durationY == null) {
          duration = durationX;
        } else if (durationX == null && durationY != null) {
          duration = durationY;
        } else {
          duration = Math.min(durationX, durationY);
        }
        break;

      case rising:
        duration =
            (Vz0
                    - Math.sqrt(
                        (2 * gravity * startPose.getZ())
                            - (2 * gravity * targetHeight)
                            + Math.pow(Vz0, 2)))
                / gravity;
        break;

      case falling:
        duration =
            (Vz0
                    + Math.sqrt(
                        (2 * gravity * startPose.getZ())
                            - (2 * gravity * targetHeight)
                            + Math.pow(Vz0, 2)))
                / gravity;
        break;
    }

    // Return lowest non null duration value
    return (duration != null && duration < duration0H) ? duration : duration0H;
  }
}
