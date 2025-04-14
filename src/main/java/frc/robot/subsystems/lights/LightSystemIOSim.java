package frc.robot.subsystems.lights;

public class LightSystemIOSim implements LightSystemIO {
  public LightSystemIOSim() {}

  @Override
  public void updateInputs(LightSystemIOInputs inputs) {
    inputs.connected = true;
    inputs.current = 0.0;
  }
}
