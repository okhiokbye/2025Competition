package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorJustin extends PIDSubsystem {

private final TalonFX m_elevator;

private static final double LEVEL_1 = 8.0; // idk what how these values are supposed to be calculated D:
private static final double LEVEL_2 = 25.0;
private static final double LEVEL_3 = 40.0;
private static final double LEVEL_4 = 55.0;


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
  double currPosition = getMeasurement();
  double targetPosition;
  double motorPower;

  switch(input) {
    case 1:
      targetPosition = LEVEL_1;
      break;
    
    case 2:
    targetPosition = LEVEL_2;
    break;
    
    case 3:
    targetPosition = LEVEL_3;
    break;
    
    case 4:
    targetPosition = LEVEL_4;
    break;
    
    default:
      return new RunCommand(() -> m_elevator.set(0)); 
  }

  motorPower = super.getController().calculate(currPosition, targetPosition);
  return new RunCommand(() -> m_elevator.set(motorPower));

}

public void periodic() {
  // maybe use in future

}
}


// package frc.robot.subsystems;

// import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.PIDSubsystem;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class ElevatorJustin extends PIDSubsystem {

// private final TalonFX m_elevator;

// private int elevation;

// @SuppressWarnings("removal")
// public ElevatorJustin() {
// //bastard
// //they call me the dog for how much i be lickin dem toes
//     super(new PIDController(0, 0, 0));
//     m_elevator = new TalonFX(16);
// }

// @Override
// public void useOutput(double output, double setpoint) {
//   m_elevator.setVoltage(output);
// }

// @Override
// public double getMeasurement() {
//   return  m_elevator.getRotorPosition().getValueAsDouble();
// }

// @SuppressWarnings("removal")
// public Command elevate(int input) {
//   double elevation = m_elevator.getRotorPosition().getValueAsDouble();
//   double motorPower;
//   switch(input) {
//     case 1:
//       motorPower = super.getController().calculate(elevation, 0); 
//       return new RunCommand(() -> m_elevator.set(motorPower));
    
//     case 2:
//       motorPower = super.getController().calculate(elevation, 0); 
//       return new RunCommand(() -> m_elevator.set(motorPower));
    
//     case 3:
//       motorPower = super.getController().calculate(elevation, 0);  
//       return new RunCommand(() -> m_elevator.set(motorPower));
    
//     case 4:
//       motorPower = super.getController().calculate(elevation, 0); 
//       return new RunCommand(() -> m_elevator.set(motorPower));
    
//     default:
//       return new RunCommand(() -> m_elevator.set(0)); 
//   }
// }

// }