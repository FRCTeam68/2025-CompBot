package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import lombok.Getter;

public class PathPlannerUtil {
  @Getter private static List<PathPlannerPath> path = new ArrayList<>();
  private static List<String> loadedPathName = new ArrayList<>();

  public static Boolean loadPathFromList(List<String> pathName) {
    Boolean pathLoadError = false;

    if (pathName != loadedPathName) {
      path.clear();

      for (int i = 0; i < pathName.size(); i++) {
        try {
          if (pathName.get(i).toLowerCase().contains("group")) {
            path.addAll(PathPlannerAuto.getPathGroupFromAutoFile(pathName.get(i)));
          } else {
            path.add(PathPlannerPath.fromPathFile(pathName.get(i)));
          }
        } catch (Exception e) {
          DriverStation.reportError("Error loading path: " + e.getMessage(), e.getStackTrace());
          pathLoadError = true;
        }
      }

      loadedPathName = pathLoadError ? new ArrayList<>() : pathName;
    }

    return !pathLoadError;
  }

  public static Pose2d getInitalStartingPose() {
    if (path.size() >= 1) {
      try {
        return AllianceFlipUtil.apply(path.get(0).getStartingHolonomicPose().get());
      } catch (Exception e) {
        DriverStation.reportError(
            "Error getting starting holonomic pose: " + e.getMessage(), e.getStackTrace());
      }
    } else {
      DriverStation.reportError("Error getting starting pose: Path not initialized.", false);
    }

    return new Pose2d();
  }

  public static Command followPath(int index) {
    if (path.size() > index) {
      return followPath(path.get(index));
    } else {
      DriverStation.reportError("Error following path: Path " + index + " out of range.", false);
    }

    return Commands.none();
  }

  /**
   * Builds a command to follow a path
   *
   * @param pathName The name of the path to load
   * @return A path following command with for the given path
   */
  public static Command followPath(String pathName) {
    try {
      return followPath(PathPlannerPath.fromPathFile(pathName));
    } catch (Exception e) {
      DriverStation.reportError(
          "Error loading individual path from name: " + e.getMessage(), e.getStackTrace());

      return Commands.none();
    }
  }

  /**
   * Builds a command to follow a path
   *
   * @param pathName The path to follow
   * @return A path following command with for the given path
   */
  public static Command followPath(PathPlannerPath path) {
    try {
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      DriverStation.reportError("Error following path: " + e.getMessage(), e.getStackTrace());

      return Commands.none();
    }
  }
}
