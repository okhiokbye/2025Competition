package frc.robot.subsystems;
import java.io.File;
import java.io.IOException;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
@SuppressWarnings("removal")
public class CoralCryus extends PIDSubsystem {
    private final SparkMax m_shoot;
    private final SparkMax m_pitch;
    private final SparkMax m_intake;
    private final RelativeEncoder m_encoder;
    private final DigitalInput beamBreak;
    private final DoubleArrayEntry shooterInfo;
   
    @SuppressWarnings("removal")
    public CoralCryus() {
        super(new PIDController(5,0,0),0);
        m_shoot = new SparkMax(13, MotorType.kBrushless);
        m_pitch = new SparkMax(14, MotorType.kBrushless);
        m_encoder = m_pitch.getEncoder();
        m_encoder.setPosition(0);
        this.getController().disableContinuousInput();
        m_intake = new SparkMax(67 , MotorType.kBrushless);
        double[] defaultArray = {0.0,0.0,0.0};
        shooterInfo = NetworkTableInstance.getDefault().getDoubleArrayTopic("shooterInfo").getEntry(defaultArray, PubSubOption.keepDuplicates(true), PubSubOption.pollStorage(10));
        beamBreak = new DigitalInput(0);
    }
    @Override
    public void useOutput(double output, double setpoint){
        m_pitch.set(output);
    }
    @Override
    public double getMeasurement(){
        return m_encoder.getPosition();
    }
    //I have no clue if this is how we're supposed to code it but I based it off of shoot coral
    public Command intake() {
        return new StartEndCommand(
            () -> m_intake.set(.2),
            () -> m_intake.set(0));
    }

    public Command detake() {
        return new StartEndCommand(
            () -> m_intake.set(-.2),
            () -> m_intake.set(0));
    }

    public Command shootCoral() {
        return new StartEndCommand(
            //set motor power manually below
            () -> m_shoot.set(-0.6),
            () -> m_shoot.set(0)
        );
    }
    public BooleanSupplier beamBreak(){
        return () -> beamBreak.get();
    }

    public Command aim(int position) {
        switch (position) {
            case 1:
            return new RunCommand(()-> setSetpoint(5465687), this); // aim L1
            
            case 2:
            return new RunCommand(()-> setSetpoint(34567), this); // aim L4
                
            case 3: 
            return new RunCommand(()-> setSetpoint(345678), this); // reload position
        
            default:
            return new RunCommand(()-> System.out.println("how tf u call me"));
                
        }
    }
  


    public void periodic() {
        // This method will be called once per scheduler run
        double[] info = {getMeasurement(), getSetpoint(), this.getController().getError(),beamBreak.get() ? 1d :0d}; //1= true, 0= false      
        shooterInfo.set(info);
    }    

}   