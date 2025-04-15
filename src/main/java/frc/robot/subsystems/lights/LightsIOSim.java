package frc.robot.subsystems.lights;

public class LightsIOSim implements LightsIO {
  public LightsIOSim() {}

  @Override
  public void updateInputs(LightsIOInputs inputs) {
    inputs.connected = true;
    inputs.current = 0.0;
  }
}
