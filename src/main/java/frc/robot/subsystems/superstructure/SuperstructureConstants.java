package frc.robot.subsystems.superstructure;

public final class SuperstructureConstants {
  public static final class ELEVATOR {
    public static final double max = 5.4;
    public static final double min = 0;
    public static final double bump = 0.25;

    public static final double maxLowSafe = 0.2;
    public static final double minMidSafe = 2.356;
    public static final double maxMidSafe = 3.2;
    public static final double minHighSafe = 4.8;
    public static final double minAtProcessor = 0.68;
    public static final double maxLowWristMoveFromSafe =
        0.6; // height to start wrist move if wrist is in safe position

    // Old block values used in failsafe elevator wrist command
    // TODO Remove these and use values above
    public static final double MAX_POSITION_BLOCK5 = 4.8; // ????
    public static final double MAX_POSITION_BLOCK4 = 4.2; // ????
    public static final double SAFE_IN_BLOCK4 = 2.5;
    public static final double MIN_POSITION_BLOCK4 = 2.1; // ????
    public static final double MAX_POSITION_BLOCK2 = 1.02; // ????
    public static final double MAX_POSITION_BLOCK0 = 0.2;
  }

  public static final class WRIST {
    public static final double max = 0.544;
    public static final double min = 0;
    public static final double bump = 0.03;

    public static final double L2_4 = 0.0672;
    public static final double A1_2 = 0.44;

    public static final double minElevate = 0.059;
    public static final double maxElevate = 0.075;
    public static final double elevate = 0.0672; // packaged position to lift elevator
    public static final double safe = 0.232; // minimum position to move full elevator travel
    public static final double maxAtElevatorMinimum = 0.416;
    public static final double maxAtProcessor = 0.544;
  }

  public static enum Pose {
    L1(1.6, 0.44),
    L2(0.9, WRIST.L2_4),
    L3(2.4, WRIST.L2_4),
    L4(4.6, WRIST.L2_4),
    A1(1.8, WRIST.A1_2),
    A2(3.1, WRIST.A1_2),
    processor(0.68, 0.528),
    preBarge(5.33, 0.352),
    shootBarge(5.33, 0.1),
    cradle(0, 0.192),
    intakeCoral(0, 0.012);

    Pose(double elevator, double wrist) {
      this.elevator = elevator;
      this.wrist = wrist;
    }

    public final double elevator;
    public final double wrist;
  }
}
