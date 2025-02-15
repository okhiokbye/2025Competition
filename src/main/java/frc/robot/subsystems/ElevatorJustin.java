package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorJustin extends PIDSubsystem {

private final TalonFX m_elevator;
private  

public ElevatorJustin() {
//bastard
//they call me the dog for how much i be lickin dem toes
    super(new PIDController(0, 0, 0));

}

@Override
public void useOutput(double output, double setpoint) {
  m_elevator.setVoltage(output);
}

@Override
public double getMeasurement() {
  return  m_elevator.getRotorPosition().getValueAsDouble();
}

public Command

}
