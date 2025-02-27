package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class AlgaeSwarit extends PIDSubsystem {
    private final SparkMax m_algaeJaw;
    private final SparkMax m_algaeTop;
    private final SparkMax m_algaeBot;

    public AlgaeSwarit(){
        super(new PIDController(3,0,0));
        m_algaeTop = new SparkMax(17, MotorType.kBrushless);
        m_algaeBot = new SparkMax(18, MotorType.kBrushless);
        m_algaeJaw = new SparkMax(19, MotorType.kBrushless);
      
    }

    @Override
    protected void useOutput(double output, double setpoint) {
      m_algaeJaw.setVoltage(output);
    }

    @Override
    protected double getMeasurement() {
       return m_algaeJaw.getEncoder().getPosition()*360*(1.0/36.0);
       // convert to degrees of rotation on output shaft 
    }
    public RunCommand setJawPos(int n){
      switch(n){
        case 1:
        return new RunCommand(()->this.setSetpoint(56), this);
        case 2:
        return new RunCommand(()->this.setSetpoint(-56), this);
        case 3:
        return new RunCommand(()->this.setSetpoint(56), this);
        default:
        return new RunCommand(()->System.out.println("algae does not exist"), this);
      }
    }
    public void runTeeth(double output){
      m_algaeBot.set(output);
      m_algaeBot.set(output);
    }
    public StartEndCommand rollIn(){
      return new StartEndCommand(()->runTeeth(0.5),()->runTeeth(0));
    }
    public StartEndCommand rollOut(){
      return new StartEndCommand(()->runTeeth(-0.5),()->runTeeth(0));
    }
    }
 
