package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.concurrent.atomic.DoubleAdder;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorJustin extends PIDSubsystem {

private final TalonFX m_elevator1;

private static final double LEVEL_0 = 567689;
private static final double LEVEL_1 = 8.0;
private static final double LEVEL_2 = 25.0;
private static final double LEVEL_3 = 40.0;
private static final double LEVEL_4 = 55.0;  
private DigitalInput limitSwitch;

private final DoubleArrayEntry elevatorInfo;
@SuppressWarnings("removal")
public ElevatorJustin() {
    super(new PIDController(5, 0, 0), 0);
    this.getController().disableContinuousInput();
    
    m_elevator1 = new TalonFX(16);
  
    double[] defaultArray = {0.0,0.0,0.0};
    elevatorInfo = NetworkTableInstance.getDefault().getDoubleArrayTopic("elevatorInfo").getEntry(defaultArray, PubSubOption.keepDuplicates(true), PubSubOption.pollStorage(10));
    limitSwitch = new DigitalInput(1);  
    m_elevator1.setPosition(0);

    
  }

@Override
public void useOutput(double output, double setpoint) { 
  m_elevator1.setVoltage(output);

}

@Override
public double getMeasurement() {
  return  m_elevator1.getPosition().getValueAsDouble()*360*(1.0/189); 
  // convert to degrees of rotation on output shaft 
}

public BooleanSupplier getLimit(){
  return ()-> limitSwitch.get();
}
public Command zero(){
  return runOnce( ()-> this.zero());
}
public void zeroHeight(){
  m_elevator1.setPosition(0);

}

@SuppressWarnings("removal")
public Command elevate(int input) {
 //WHEN MOVING ELEVATOR UP AND DOWN STOW IN L4 AIM POSITION OR ELSE INTAKE DIE

  switch(input) {
    case 1:
    return new RunCommand(()-> setSetpoint(LEVEL_1), this);
    
    case 2:
    return new RunCommand(()-> setSetpoint(LEVEL_2),this);
    
    case 3:
    return new RunCommand(()-> setSetpoint(LEVEL_3),this);
    
    case 4:
    return new RunCommand(()-> setSetpoint(LEVEL_4), this);

    case 5: 
    return new RunCommand(()-> setSetpoint(LEVEL_0), this); // reload height
    default: 
    return new RunCommand(()-> System.out.println("how tf u call me"));
  }

 

}

public void periodic() {
  double[] info = {getMeasurement(),getSetpoint(),this.getController().getError()};
  elevatorInfo.set(info);
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