package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorJustin extends PIDSubsystem {

private final TalonFX m_elevator;

private int elevation;

@SuppressWarnings("removal")
public ElevatorJustin() {
//bastard
//they call me the dog for how much i be lickin dem toes
    super(new PIDController(0, 0, 0));
    m_elevator = new TalonFX(16);
}

@Override
public void useOutput(double output, double setpoint) {
  m_elevator.setVoltage(output);
}

@Override
public double getMeasurement() {
  return  m_elevator.getRotorPosition().getValueAsDouble();
}

@SuppressWarnings("removal")
public Command elevate(int input) {
  double elevation = m_elevator.getRotorPosition().getValueAsDouble();
  double motorPower;
  switch(input) {
    case 1:
      motorPower = super.getController().calculate(elevation, 0); 
      return new RunCommand(() -> m_elevator.set(motorPower));
    
    case 2:
      motorPower = super.getController().calculate(elevation, 0); 
      return new RunCommand(() -> m_elevator.set(motorPower));
    
    case 3:
      motorPower = super.getController().calculate(elevation, 0);  
      return new RunCommand(() -> m_elevator.set(motorPower));
    
    case 4:
      motorPower = super.getController().calculate(elevation, 0); 
      return new RunCommand(() -> m_elevator.set(motorPower));
    
    default:
      return new RunCommand(() -> m_elevator.set(0)); 
  }
}

}
