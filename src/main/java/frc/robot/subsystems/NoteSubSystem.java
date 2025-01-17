package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.LightsSubsystem.LEDSegment;
import org.littletonrobotics.junction.Logger;

public class NoteSubSystem extends SubsystemBase {

  public enum State {
    IDLE,
    INTAKING_NOTE1,
    SHOOTING,
    SHOOT_SPINNING,
    SPITTING_NOTE
  }

  public enum Target {
    SPEAKER,
    AMP,
    TRAP,
    INTAKE,
    FEEDSTATION,
    SPEAKER_PODIUM,
    SPEAKER_1M,
    SPEAKER_PODIUM_SOURCE,
    CUSTOM
  }

  public enum ActionRequest {
    IDLE,
    STOP,
    STOP_ALL,
    INTAKENOTE,
    BEAM3,
    // BEAM1,
    // BEAM2,
    SPIT_NOTE2,
    SHOOT,
    SHOOT_SPINUP,
    // FEEDSTATION_SPIN,
    DISLODGE_WITH_SHOOTER
  }

  private State m_presentState;
  private ActionRequest m_wantedAction;
  private Target m_target;
  private boolean m_haveNote1;
  private boolean m_spunShooterUp;
  private RollerSubSystem m_Intake;
  private RollerSubSystem m_Feeder1;
  private RollerSubSystem m_Feeder2;
  public ShooterSubSystem m_Shooter;
  private AngleSubSystem m_Angle;
  private Timer m_shootStopTime;
  private double m_shooter_setpoint;
  private double m_shooterRight_setpoint;
  private double m_shooterfeeder2_setpoint;
  private double m_feeder2_setpoint;
  private double m_feeder1_setpoint;
  private double m_intake_setpoint;
  private boolean m_actionChanged;
  private double m_beam_count_total;

  public Counter Beam3;

  public NoteSubSystem() {
    m_presentState = State.IDLE;
    m_target = Target.SPEAKER;
    m_wantedAction = ActionRequest.IDLE;
    setHaveNote1(false);
    setShooterSpunUp(false);
    m_actionChanged = true;
    m_beam_count_total = 0;

    m_Intake = new RollerSubSystem("Intake", Constants.INTAKE.CANID, Constants.INTAKE.CANBUS, true);
    m_Feeder1 =
        new RollerSubSystem("Feeder1", Constants.FEEDER1.CANID, Constants.FEEDER1.CANBUS, true);
    m_Feeder2 =
        new RollerSubSystem("Feeder2", Constants.FEEDER2.CANID, Constants.FEEDER2.CANBUS, false);
    m_Shooter = new ShooterSubSystem();
    m_Angle = new AngleSubSystem();

    // -------------------------------------------------------------------------
    // try1 - what we used at Kettering
    // trigger in robotContainer doing DigitalInput.get() and if true trigger BEAM3 state of Note
    // we were missing the trigger sometimes

    // -------------------------------------------------------------------------
    // try2 - using counter pulse length.
    //         was working then sudden 10s to 100s of negative counts
    //          then at the end of Saturday, March 16th, no upward counts!!!
    //     Not convinced it not a hardware problem.  Need scope to look at signal again.
    //
    // Beam3 = new Counter(Counter.Mode.kPulseLength);
    // Beam3.setUpSource(1);
    // // Set the decoding type to 2X
    // Beam3.setUpSourceEdge(true, true);
    // // Set the counter to count down if the pulses are longer than .02 seconds
    // Beam3.setPulseLengthMode(.02);

    // try 2.1
    // a pulse less that 100ms will be count up.  greater than 100ms count down
    // Beam3.setPulseLengthMode(.1);

    // -------------------------------------------------------------------------
    // try3
    // In semi-period mode, the Counter will count the duration of the pulses on a channel,
    // either from a rising edge to the next falling edge, or from a falling edge to the next rising
    // edge.
    // Because it counts in both rising and falling edges, the period the pulse is high should be
    // returned.
    // To get the pulse width, call the getPeriod() method
    // the count returned should be 2 for every pulse.
    // Beam3 = new Counter(Counter.Mode.kSemiperiod);
    // Beam3.setUpSource(1);
    // Beam3.setSemiPeriodMode(true);

    // -------------------------------------------------------------------------
    // try4
    // two pulse mode, but only use an up source
    // In all modes except semi-period mode, the counter can be configured to increment either once
    // per edge (2X decoding),
    //  or once per pulse (1X decoding). By default, counters are set to two-pulse mode,
    //  though if only one channel is specified the counter will only count up.
    // Beam3 = new Counter(Counter.Mode.kTwoPulse);
    // Beam3.setUpSource(1);
    // Beam3.setUpSourceEdge(true, false);

    // 3/18 - 5:50pm - true, true - not tripping.  no beam count up
    //       5:53pm - true, false - nothing!!
    //       6:00pm  - problem is hardware!!!!
    // 2X mode would be true, true ??  so counter would be 2 for each beam pulse (and getPeriod
    // would return time pulse is high)
    // 1X mode would be true, false ??  counter would be only 1 for each beam pulse (and getPeriod
    // would return time since last pulse)

    // -------------------------------------------------------------------------
    // try5
    // should be basically the same thing as try4
    // DigitalInput m_noteSensor3 = new DigitalInput(1);
    // Beam3 = new Counter(m_noteSensor3);
    // Beam3.setUpSourceEdge(true, true);

    // try5.1, could try just counting rising edge.  but then period is time since last rising ???

    // -------------------------------------------------------------------------
    // try6
    // Initializes an AnalogInput on port 1 and enables 4-bit averaging (32 samples averaged)
    // Using analog might have advantage that it can filter high frequency noise with averaging

    AnalogInput input = new AnalogInput(0);
    input.setAverageBits(4);

    // // Initializes an AnalogTrigger using the above input
    AnalogTrigger noteTriggerAnalog = new AnalogTrigger(input);

    // // Sets the trigger to enable at a voltage of 4 volts, and disable at a value of 1.5 volts
    noteTriggerAnalog.setLimitsVoltage(1.5, 4);

    Beam3 = new Counter(noteTriggerAnalog);
    // // above already sets calls setUpSource counter

    // Apr 4th, states, testing inverted logic level shifter
    Beam3.setUpSourceEdge(false, true);

    // try6.1, could try just counting rising edge.  but then period is time since last rising ???

    // -------------------------------------------------------------------------

    Shuffleboard.getTab("IntakeSubsystem").add(m_Intake);
    Shuffleboard.getTab("Feeder1Subsystem").add(m_Feeder1);
    Shuffleboard.getTab("Feeder2Subsystem").add(m_Feeder2);
    Shuffleboard.getTab("ShooterSubystem").add(m_Shooter);
    Shuffleboard.getTab("AngleSubsystem").add(m_Angle);

    m_shootStopTime = new Timer();

    resetSetpoints();

    System.out.println("Note subsystem created");
    Logger.recordOutput("Note/Comment", "Note subsystem created");
    Logger.recordOutput("Note/State", m_presentState);
    Logger.recordOutput("Note/Target", m_target);
    Logger.recordOutput("Note/Action", m_wantedAction);

    LEDSegment.Do1to4.setBandAnimation(LightsSubsystem.blue, 2);
    LEDSegment.Do5to8.setColor(LightsSubsystem.orange);
    LEDSegment.side1.setBandAnimation(LightsSubsystem.blue, 0.5);
    LEDSegment.side1target.setColor(LightsSubsystem.white);
    LEDSegment.side1heading.setColor(LightsSubsystem.white);
    LEDSegment.side1distance.setColor(LightsSubsystem.white);
    // LEDSegment.side2.setStrobeAnimation(LightsSubsystem.green, 0.5);
  }

  public void resetSetpoints() {

    m_shooterRight_setpoint = Constants.SHOOTER.RIGHT_OFFSET; // default
    switch (m_target) {
      default:
      case SPEAKER, SPEAKER_1M, SPEAKER_PODIUM:
        m_shooter_setpoint = Constants.SHOOTER.SPEAKER_SHOOT_SPEED;
        break;
      case AMP:
        m_shooter_setpoint = Constants.SHOOTER.AMP_SHOOT_SPEED;
        m_shooterRight_setpoint = Constants.SHOOTER.AMP_RIGHT_OFFSET;
        break;
      case TRAP:
        m_shooter_setpoint = Constants.SHOOTER.TRAP_SHOOT_SPEED;
        break;
        // case FEEDSTATION:
        //    not implemented yet
        //     break;
    }

    m_shooterfeeder2_setpoint = Constants.FEEDER2.SHOOT_SPEED;

    m_feeder2_setpoint = Constants.FEEDER2.TAKE_NOTE_SPEED;
    m_feeder1_setpoint = Constants.FEEDER1.TAKE_NOTE_SPEED;
    m_intake_setpoint = Constants.INTAKE.TAKE_NOTE_SPEED;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Note");
    builder.addStringProperty("State", () -> m_presentState.toString(), null);
    builder.addStringProperty("Target", () -> m_target.toString(), null);
    builder.addStringProperty("Action", () -> m_wantedAction.toString(), null);
    builder.addDoubleProperty(
        "setpoint/shooter", this::getShooterSetpointSpeed, this::setShooterSetpointSpeed);
    builder.addDoubleProperty(
        "setpoint/shooterR", this::getShooterRSetpointSpeed, this::setShooterRSetpointSpeed);
    builder.addDoubleProperty(
        "setpoint/shooterFD2", this::getShooterFD2SetpointSpeed, this::setShooterFD2SetpointSpeed);
    builder.addDoubleProperty(
        "setpoint/feeder2", this::getFeeder2SetpointSpeed, this::setFeeder2SetpointSpeed);
    builder.addDoubleProperty(
        "setpoint/feeder1", this::getFeeder1SetpointSpeed, this::setFeeder1SetpointSpeed);
    builder.addDoubleProperty(
        "setpoint/intake", this::getIntakeSetpointSpeed, this::setIntakeSetpointSpeed);
    builder.addBooleanProperty("HaveNote1", this::getHaveNote1, null);
    builder.addBooleanProperty("ShooterSpunUp", this::getShooterSpunUp, null);
  }

  public double getShooterSetpointSpeed() {
    return this.m_shooter_setpoint;
  }

  public double getShooterRSetpointSpeed() {
    return this.m_shooterRight_setpoint;
  }

  public double getShooterFD2SetpointSpeed() {
    return this.m_shooterfeeder2_setpoint;
  }

  public double getFeeder2SetpointSpeed() {
    return this.m_feeder2_setpoint;
  }

  public double getFeeder1SetpointSpeed() {
    return this.m_feeder1_setpoint;
  }

  public double getIntakeSetpointSpeed() {
    return this.m_intake_setpoint;
  }

  public void setShooterSetpointSpeed(double desiredSpeed) {
    m_shooter_setpoint = desiredSpeed;
  }

  public void setShooterRSetpointSpeed(double desiredSpeed) {
    m_shooterRight_setpoint = desiredSpeed;
  }

  public void setShooterFD2SetpointSpeed(double desiredSpeed) {
    m_shooterfeeder2_setpoint = desiredSpeed;
  }

  public void setFeeder2SetpointSpeed(double desiredSpeed) {
    m_feeder2_setpoint = desiredSpeed;
  }

  public void setFeeder1SetpointSpeed(double desiredSpeed) {
    m_feeder1_setpoint = desiredSpeed;
  }

  public void setIntakeSetpointSpeed(double desiredSpeed) {
    m_intake_setpoint = desiredSpeed;
  }

  public void setTarget(Target wantedTarget) {

    m_target = wantedTarget;
    Logger.recordOutput("Note/Comment", "target change");
    Logger.recordOutput("Note/Target", m_target);

    m_shooterRight_setpoint = Constants.SHOOTER.RIGHT_OFFSET; // default; override below if needed

    switch (m_target) {
      default:
      case SPEAKER:
        m_Angle.setState(AngleSubSystem.State.SPEAKER);
        m_shooter_setpoint = Constants.SHOOTER.SPEAKER_SHOOT_SPEED;
        LEDSegment.side1target.setColor(LightsSubsystem.red);
        break;
      case SPEAKER_1M:
        m_Angle.setState(AngleSubSystem.State.SPEAKER_1M);
        m_shooter_setpoint = Constants.SHOOTER.SPEAKER_SHOOT_SPEED;
        LEDSegment.side1target.setColor(LightsSubsystem.orange);
        break;
      case AMP:
        m_Angle.setState(AngleSubSystem.State.AMP);
        m_shooter_setpoint = Constants.SHOOTER.AMP_SHOOT_SPEED;
        m_shooterRight_setpoint = Constants.SHOOTER.AMP_RIGHT_OFFSET;
        LEDSegment.side1target.setColor(LightsSubsystem.yellow);
        break;
      case TRAP:
        m_Angle.setState(AngleSubSystem.State.TRAP);
        m_shooter_setpoint = Constants.SHOOTER.TRAP_SHOOT_SPEED;
        LEDSegment.side1target.setColor(LightsSubsystem.purple);
        break;
      case INTAKE:
        m_Angle.setState(AngleSubSystem.State.INTAKE);
        LEDSegment.side1target.setColor(LightsSubsystem.blue);
        // 3/13/2024 - not going to shoot from intake angle now
        // m_shooter_setpoint = Constants.SHOOTER.SPEAKER_SHOOT_SPEED;
        // m_shooterRight_setpoint = Constants.SHOOTER.RIGHT_OFFSET;
        break;
        // case FEEDSTATION:
        //   not implemented yet
        //     m_Angle.setState(AngleSubSystem.State.FEEDSTATION);
        //     break;
      case SPEAKER_PODIUM:
        m_Angle.setState(AngleSubSystem.State.SPEAKER_PODIUM);
        m_shooter_setpoint = Constants.SHOOTER.SPEAKER_SHOOT_SPEED;
        LEDSegment.side1target.setColor(LightsSubsystem.orange);
        break;
      case SPEAKER_PODIUM_SOURCE:
        m_Angle.setState(AngleSubSystem.State.SPEAKER_PODIUM_SOURCE);
        m_shooter_setpoint = Constants.SHOOTER.SPEAKER_SHOOT_SPEED;
        LEDSegment.side1target.setColor(LightsSubsystem.orange);
        break;
      case CUSTOM:
        break;
    }

    if (m_target != Target.INTAKE) spinUp();
  }

  public void setTargetCustom(double desiredPosition, double desiredSpeed) {

    m_target = Target.CUSTOM;
    m_Angle.setCustomPosition(desiredPosition);
    m_Angle.setState(AngleSubSystem.State.CUSTOM_ANGLE);
    m_shooter_setpoint = desiredSpeed;
    spinUp();
    LEDSegment.side1target.setColor(LightsSubsystem.white);
    Logger.recordOutput("Note/TargtCustom", desiredPosition);
    Logger.recordOutput("Note/Target", m_target);
  }

  public void setAction(ActionRequest wantedAction) {

    m_actionChanged = (wantedAction == m_wantedAction) ? false : true;

    m_wantedAction = wantedAction;
    Logger.recordOutput("Note/Action", m_wantedAction);
  }

  // this is the state machine of the notesubsystem
  @Override
  public void periodic() {

    boolean isAtAngle = false;
    boolean beam3Tripped = false;
    // double beam3Period = 0;

    double beam3count = Beam3.get();
    m_beam_count_total += beam3count;
    if (beam3count > 0) {
      // expect rising edge and falling edge of pulse to be counted
      // Also, can only measure period if count is 2 or more
      // Period is measured from last 2 edges
      // beam3Period = (beam3count>=2) ? Beam3.getPeriod() : -1;
      beam3Tripped = true;
      Beam3.reset();
      // latch to smartboard the last period measurement.
      // need to this to test if two-edge method is generally working
      // AdvantageKit will store for every beam trip and can be used for statistics
      // if period measured is consistant.
      SmartDashboard.putNumber("beam3 last count", beam3count);
      // SmartDashboard.putNumber("beam3 last period", beam3Period);
    }

    if (beam3Tripped) {
      if (m_presentState == State.INTAKING_NOTE1) {
        // front side of note coming through
        Logger.recordOutput("Note/Comment", "stop intake");
        m_Feeder2.setSpeed(0);
        m_Feeder1.setSpeed(0);
        m_Intake.setSpeed(0);
        m_shootStopTime.stop();
        m_shootStopTime.reset();
        setHaveNote1(true);
        LEDSegment.side1.setColor(LightsSubsystem.blue);
        setState(State.IDLE);
        setAction(ActionRequest.IDLE);
      } else if (m_presentState == State.SHOOTING) {
        // backside of note coming through
        Logger.recordOutput("Note/Comment", "note shot");
        // need to wait a bit before shifting to lower speed on shooter
        m_shootStopTime.restart();
        setHaveNote1(false);
        setState(State.IDLE);
        setAction(ActionRequest.IDLE);
      } else if (m_presentState == State.SPITTING_NOTE) {
        Logger.recordOutput("Note/Comment", "note spit out");
        m_shootStopTime.stop();
        m_shootStopTime.reset();
        // just need to reflect no longer have a note
        // not sure why we would spit out a note we have; maybe wrong button hit?
        setHaveNote1(false);
        LEDSegment.side1.setColor(LightsSubsystem.orange);
        // don't stop rollers here as note is not out yet, just show we don't have it anymore
        // driver to stop rollers or start intake again
      }
    }
    Logger.recordOutput("Note/beam3count", beam3count);
    Logger.recordOutput("Note/beam3tripped", beam3Tripped);
    // Logger.recordOutput("Note/beam3period", beam3Period);
    Logger.recordOutput("Note/beam3countTotal", m_beam_count_total);
    // SmartDashboard.putNumber("beam3 count", beam3count);
    SmartDashboard.putBoolean("beam3 tripped", beam3Tripped);
    // SmartDashboard.putNumber("beam3 period", beam3Period);
    SmartDashboard.putNumber("beam3 count total", m_beam_count_total);

    switch (m_wantedAction) {
      default:
      case IDLE:
        if (m_shootStopTime.hasElapsed(.3)) {
          // shooting  has started , beam3 break already seen, timer elasped
          m_shootStopTime.stop();
          m_shootStopTime.reset();
          Logger.recordOutput("Note/Comment", "shoot timer elapsed");
          // 9/17/2024 - backside of note has gone through and now the elasped
          //             time after that has expired, so coast down the shooter.
          // there is a check later that once speed goes below a lowspeed threshold,
          // it will set it to that threshold

          // not stopping FD2 because false triggers.  let it run
          // m_Feeder2.setSpeed(0);

          // step down speed when not autonomous mode
          if (!DriverStation.isAutonomous()) {
            m_Shooter.setSpeed(0); // will coast down to low speed velocity
          }
          LEDSegment.side1.setColor(LightsSubsystem.orange);
          setState(State.IDLE);
        }
        break;
        // case STOP:
        //     Logger.recordOutput("Note/Comment",  "stop 40");
        //     m_Intake.setSpeed(0);
        //     m_Feeder1.setSpeed(0);
        //     m_Feeder2.setSpeed(0);
        //     m_shooter_setpoint = 40;
        //     spinUp();
        //     m_shootStopTime.stop();
        //     m_shootStopTime.reset();
        //     setShooterSpunUp(true);
        //     setState(State.IDLE);
        //     LEDSegment.side1.setColor(LightsSubsystem.orange);
        //     setAction(ActionRequest.IDLE);
        //     break;
      case STOP_ALL:
        Logger.recordOutput("Note/Comment", "stop all");
        m_Intake.setSpeed(0);
        m_Feeder1.setSpeed(0);
        m_Feeder2.setSpeed(0);
        m_Shooter.setSpeed(0);
        m_Shooter.stop();
        m_shootStopTime.stop();
        m_shootStopTime.reset();
        setShooterSpunUp(false);
        setState(State.IDLE);
        LEDSegment.side1.setColor(LightsSubsystem.orange);
        setAction(ActionRequest.IDLE);
        break;
      case INTAKENOTE:
        if (m_actionChanged) {
          // do just once, when action commanded
          // rapid pulse until at angle
          LEDSegment.side1.setBandAnimation(LightsSubsystem.orange, .5);
          m_actionChanged = false;
        }
        if (m_target != Target.INTAKE) {
          setTarget(Target.INTAKE);
        }
        if (!m_haveNote1) {
          double anglenow = m_Angle.getAngleF();
          // looking for intake angle to be close to start rollers.
          // we don't want too far off or note can get stuck.
          // this is hardcode range around 17!
          // there is more tolerance to be off when lower than when higher
          if ((anglenow > 10) && (anglenow < 20)) {
            Logger.recordOutput("Note/Comment", "start intake:" + anglenow);
            m_Intake.setSpeed(m_intake_setpoint);
            m_Feeder1.setSpeed(m_feeder1_setpoint);
            m_Feeder2.setSpeed(m_feeder2_setpoint);
            LEDSegment.side1.setStrobeAnimation(LightsSubsystem.orange, .5);
            setState(State.INTAKING_NOTE1);
            setAction(ActionRequest.IDLE);
          }
        }

        break;
      case SPIT_NOTE2:
        if (m_actionChanged) {
          // do just once, when action commanded
          LEDSegment.side1.setFlowAnimation(LightsSubsystem.yellow, .25);
          m_actionChanged = false;
        }
        if (m_target != Target.INTAKE) {
          setTarget(Target.INTAKE);
        }
        if (m_Angle.atAngle()) {
          Logger.recordOutput("Note/Comment", "spit note");
          m_Intake.setSpeed(-m_intake_setpoint);
          m_Feeder1.setSpeed(-m_feeder1_setpoint);
          m_Feeder2.setSpeed(-m_feeder2_setpoint);
          LEDSegment.side1.setFlowAnimation(LightsSubsystem.yellow, .25);
          setState(State.SPITTING_NOTE);
          setAction(ActionRequest.IDLE);
        }
        break;
      case DISLODGE_WITH_SHOOTER:
        LEDSegment.side1.setFadeAnimation(LightsSubsystem.red, .5);

        Logger.recordOutput("Note/Comment", "reverse spin shooter");
        m_Shooter.setRightOffsetSpeed(0); // top
        m_Feeder2.setSpeed(-m_shooterfeeder2_setpoint);
        m_Shooter.setSpeed(Constants.SHOOTER.DISLODGE_SHOOT_SPEED);
        setShooterSpunUp(false);
        setAction(ActionRequest.IDLE);
        break;
      case SHOOT_SPINUP:
        Logger.recordOutput("Note/Comment", "spinup shooter");
        spinUp();
        setAction(ActionRequest.IDLE);
        break;
      case SHOOT:
        if (m_actionChanged) {
          // do just once, when action commanded
          LEDSegment.side1.setStrobeAnimation(LightsSubsystem.red, .5);
          m_actionChanged = false;
        }
        //  if (m_Shooter.atSpeed()) -- not implemented yet

        if (m_Angle.atAngle()) {
          Logger.recordOutput("Note/Comment", "feed shooter");
          m_Feeder2.setSpeed(m_shooterfeeder2_setpoint);
          setState(State.SHOOTING);
          LEDSegment.side1.setColor(LightsSubsystem.red);
          setAction(ActionRequest.IDLE);
        }
        break;
    }

    // further LED states ...
    isAtAngle = m_Angle.atAngle();

    String readyToShootMsg = "no";
    if (m_haveNote1) {
      if (m_target != Target.INTAKE) {
        if ((isAtAngle) && (m_spunShooterUp) && (m_Shooter.atSpeed())) {
          // all ready to shoot
          //  if at intake angle, you can shoot, but will not turn it green
          //  the green is to indicate you have changed from intake (blue) to another angle
          LEDSegment.side1.setColor(LightsSubsystem.green);
          readyToShootMsg = "green";
        } else {
          // waiting for conditions to be ready to shoot
          // note in manual mode, driver still has to drive to field position for selected target
          // and point correcting direction
          LEDSegment.side1.setBandAnimation(LightsSubsystem.green, .5);
          readyToShootMsg = "waiting A or S";
        }
      }
      // else condition is at intake target, with note;  stay with all blue set by beam break
      //                                              or pass note sets Fade blue
    }
    Logger.recordOutput("Note/ReadyToShoot", readyToShootMsg);

    SmartDashboard.putBoolean("AtAngle AMP", (m_target == Target.AMP) && (isAtAngle));
    SmartDashboard.putBoolean("AtAngle TRAP", (m_target == Target.TRAP) && (isAtAngle));
    SmartDashboard.putBoolean("AtAngle Podium", (m_target == Target.SPEAKER_PODIUM) && (isAtAngle));
    SmartDashboard.putBoolean("AtAngle Intake", (m_target == Target.INTAKE) && (isAtAngle));
    SmartDashboard.putBoolean("AtAngle Speaker", (m_target == Target.SPEAKER) && (isAtAngle));
  }

  public void spinUp() {
    m_Shooter.setRightOffsetSpeed(m_shooterRight_setpoint);
    m_Shooter.setSpeed(m_shooter_setpoint);
    setShooterSpunUp(true);
  }

  public void setHaveNote1(boolean haveNote1) {
    m_haveNote1 = haveNote1;
    Logger.recordOutput("Note/HaveNote1", m_haveNote1);
  }

  public boolean getHaveNote1() {
    return this.m_haveNote1;
  }

  public void setShooterSpunUp(boolean spunUp) {
    m_spunShooterUp = spunUp;
    Logger.recordOutput("Note/shooterSpunUp", m_spunShooterUp);
  }

  public boolean getShooterSpunUp() {
    return this.m_spunShooterUp;
  }

  public boolean atTargetAngle() {
    return m_Angle.atAngle();
  }

  public boolean atTargetSpeed() {
    return m_Shooter.atSpeed();
  }

  private void setState(State desiredStation) {
    m_presentState = desiredStation;
    Logger.recordOutput("Note/State", m_presentState);
  }

  public State getState() {
    return this.m_presentState;
  }

  public ActionRequest getAction() {
    return this.m_wantedAction;
  }

  public Target getTarget() {
    return this.m_target;
  }

  public void bumpIntake1Speed(double bumpAmount) {
    m_Intake.bumpSpeed(bumpAmount);
    m_intake_setpoint = m_Intake.getSpeed();
    m_Feeder1.bumpSpeed(bumpAmount);
    m_feeder1_setpoint = m_Feeder1.getSpeed();
    m_Feeder2.bumpSpeed(bumpAmount);
    m_feeder2_setpoint = m_Feeder2.getSpeed();
  }

  public void bumpIntake2Speed(double bumpAmount) {
    m_Intake.bumpSpeed(bumpAmount);
    m_intake_setpoint = m_Intake.getSpeed();
    m_Feeder1.bumpSpeed(bumpAmount);
    m_feeder1_setpoint = m_Feeder1.getSpeed();
  }

  public void bumpShooterSpeed(double bumpAmount) {
    m_Shooter.setSpeed(m_shooter_setpoint);
    m_Shooter.bumpSpeed(bumpAmount);
    m_shooter_setpoint = m_Shooter.getSpeed();

    // m_Feeder2.setSpeed(m_shooterfeeder2_setpoint);
    // m_Feeder2.bumpSpeed(bumpAmount);
    // m_shooterfeeder2_setpoint=m_Feeder2.getSpeed();
  }

  public void setPassSpeed(double speed) {
    setTarget(Target.INTAKE);
    m_shooter_setpoint = speed;
    m_shooterRight_setpoint = Constants.SHOOTER.RIGHT_OFFSET;
    m_Shooter.setSpeed(m_shooter_setpoint);
    LEDSegment.side1.setFadeAnimation(LightsSubsystem.blue, .5);
    // setAction(ActionRequest.SHOOT);   cannot do this unless AtSpeed works
    // so use user delay for spinup to happen.
  }

  public void bumpAnglePosition(double bumpAmount) {
    m_Angle.bumpPosition(bumpAmount);
  }

  public void zeroAngleSubsystem() {
    m_Angle.zeroAngleSensor();
  }
}
