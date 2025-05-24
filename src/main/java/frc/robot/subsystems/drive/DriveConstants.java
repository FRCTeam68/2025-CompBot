package frc.robot.subsystems.drive;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import lombok.Builder;

public class DriveConstants {
  // CANbus
  public static final String CANBusName = "DRIVEbus";
  public static final double odometryFrequency =
      new CANBus(CANBusName).isNetworkFD() ? 250.0 : 100.0;

  // Mechanical configuration
  public static final double trackWidthX = Units.inchesToMeters(21.75);
  public static final double trackWidthY = Units.inchesToMeters(21.75);
  public static final double driveBaseRadius = Math.hypot(trackWidthX / 2, trackWidthY / 2);
  public static final double wheelRadius = Units.inchesToMeters(1.94);
  public static final double wheelCOF = 1.0;

  public static final double robotMass = Units.lbsToKilograms(110);
  public static final double robotMOI = 5.07;

  public static final double maxLinearSpeed = 3.77952;
  public static final double maxAngularSpeed = maxLinearSpeed / driveBaseRadius;
  public static final double maxLinearAcceleration = 22.0;
  public static final double maxAngularAcceleration = maxLinearAcceleration / driveBaseRadius;

  public static final double driveGearRatio = (50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0);
  // Turn ratios - MK3: 12.8; MK4i: 150.0/7;
  public static final double turnGearRatio = 150.0 / 7;

  public static final double slipCurrent = 40;

  public static final ClosedLoopOutputType turnClosedLoopOutput = ClosedLoopOutputType.Voltage;
  public static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  /** Includes bumpers! */
  public static final double robotWidth =
      Units.inchesToMeters(28.0) + 2 * Units.inchesToMeters(2.0);

  public static class PigeonConstants {
    public static final int id = 50;
    // TODO add pigeon configs here such as mounting
    // public static final Pigeon2Configuration config = new
    // Pigeon2Configuration().withPigeon2Features(new Pigeon2FeaturesConfigs())
  }

  public static final Translation2d[] moduleTranslations = {
    new Translation2d(trackWidthX / 2, trackWidthY / 2),
    new Translation2d(trackWidthX / 2, -trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, -trackWidthY / 2)
  };

  public static final ModuleConfig[] moduleConfigsComp = {
    // Front Left
    ModuleConfig.builder()
        .driveMotorId(1)
        .turnMotorId(2)
        .encoderId(15)
        .encoderOffset(0.267333984375)
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // Front Right
    ModuleConfig.builder()
        .driveMotorId(3)
        .turnMotorId(4)
        .encoderId(16)
        .encoderOffset(0.18896484375)
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // Back Left
    ModuleConfig.builder()
        .driveMotorId(5)
        .turnMotorId(6)
        .encoderId(17)
        .encoderOffset(-0.388671875)
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // Back Right
    ModuleConfig.builder()
        .driveMotorId(7)
        .turnMotorId(8)
        .encoderId(18)
        .encoderOffset(0.471435546875)
        .turnInverted(true)
        .encoderInverted(false)
        .build()
  };

  @Builder
  public record ModuleConfig(
      int driveMotorId,
      int turnMotorId,
      int encoderId,
      double encoderOffset,
      boolean turnInverted,
      boolean encoderInverted) {}
}
