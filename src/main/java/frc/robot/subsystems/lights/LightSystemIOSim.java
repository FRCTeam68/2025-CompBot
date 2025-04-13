package frc.robot.subsystems.lights;

public class LightSystemIOSim implements LightSystemIO {
  public LightSystemIOSim() {}

  @Override
  public void updateInputs(LightSystemIOInputs inputs) {
    inputs.connected = true;
    inputs.current = 0.0;
  }

  //   @Override
  //   public void setBrightness(double percent) {}

  //   @Override
  //   public void clearAnimation(Segment segment) {}

  //   @Override
  //   public void setAnimation(Animation animation, Segment segment) {}

  //   @Override
  //   public void setColor(Color color, Segment segment) {}
}
