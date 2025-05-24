// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LEDColor;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.Segment;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final Lights LED;
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlert;
  private final Alert allDisconnectedAlert =
      new Alert("All vision cameras disconnected.", AlertType.kError);
  private Double[] rotationSamples = new Double[10]; // Amount of rotation samples to use
  private final double maxRotationError = 3; // Acceptable error in degrees
  private final Segment rotationInitalizedIndicator = new Segment(2, 2, 0);
  private final Alert rotationNotInitalizedAlert =
      new Alert(
          "Robot rotation not initalized. Face robot toward april tag. If all cameras are disconnected, press the back button with reef camera facing away from driver station. (This can be done while disabled)",
          AlertType.kError);

  public Vision(Lights LED, VisionConsumer consumer, VisionIO... io) {
    this.LED = LED;
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    // and disconnected alerts
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    this.disconnectedAlert = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
      disconnectedAlert[i] = new Alert(inputs[i].name + " is disconnected.", AlertType.kWarning);
    }

    enableMegaTag1();
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  public Pose2d getTagPose(int cameraIndex) {
    Pose2d tagPose2d;

    if (inputs[cameraIndex].tagIds.length > 0) {
      int tagId = inputs[cameraIndex].tagIds[0];
      var tagPose = aprilTagLayout.getTagPose(tagId);

      if (tagPose.isPresent()) {
        tagPose2d = tagPose.get().toPose2d();
      } else {
        tagPose2d = new Pose2d();
      }
    } else {
      tagPose2d = new Pose2d();
    }
    return tagPose2d;
  }

  public void enableMegaTag1() {
    for (int sampleIndex = 0; sampleIndex < rotationSamples.length; sampleIndex++) {
      rotationSamples[sampleIndex] = null;
    }
    // TODO does this work?
    // rotationSamples = null;
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      inputs[cameraIndex].skipMegaTag1 = false;
    }
    LED.setSolidColor(
        LEDColor.RED.scaleBrightness(Lights.getOnboardLEDBrightness()),
        rotationInitalizedIndicator);
    rotationNotInitalizedAlert.set(true && Constants.currentMode != Mode.SIM);
  }

  public void disableMegaTag1() {
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      inputs[cameraIndex].skipMegaTag1 = true;
    }
    LED.setSolidColor(
        LEDColor.GREEN.scaleBrightness(Lights.getOnboardLEDBrightness()),
        rotationInitalizedIndicator);
    rotationNotInitalizedAlert.set(false);
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/" + inputs[i].name, inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();
    boolean anyConnected = false;

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlert[cameraIndex].set(
          !inputs[cameraIndex].connected && Constants.currentMode != Mode.SIM);
      if (inputs[cameraIndex].connected) anyConnected = true;

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));

        // Disable Megatag 1
        if (observation.type() == PoseObservationType.MEGATAG_1) {
          for (int sampleIndex = rotationSamples.length - 1; sampleIndex > 0; sampleIndex--) {
            rotationSamples[sampleIndex] = rotationSamples[sampleIndex - 1];
          }
          rotationSamples[0] = Math.toDegrees(observation.pose().getRotation().getAngle());
          // testing logged value
          SmartDashboard.putNumber("Testing/MT1 rotation", rotationSamples[0]);
          if (rotationSamples[rotationSamples.length - 1] != null) {
            double max = rotationSamples[0];
            double min = rotationSamples[0];
            for (int sampleIndex = rotationSamples.length - 1; sampleIndex > 0; sampleIndex--) {
              max = (rotationSamples[sampleIndex] > max) ? rotationSamples[sampleIndex] : max;
              min = (rotationSamples[sampleIndex] < min) ? rotationSamples[sampleIndex] : min;
            }
            if (max - min < maxRotationError) {
              disableMegaTag1();
            }
          }
        }
      }

      // Log camera datadata
      Logger.recordOutput(
          inputs[cameraIndex].name + "/TagPoses", tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          inputs[cameraIndex].name + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          inputs[cameraIndex].name + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          inputs[cameraIndex].name + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // update combined disconnected alert
    allDisconnectedAlert.set(!anyConnected && io.length > 0 && Constants.currentMode != Mode.SIM);

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
