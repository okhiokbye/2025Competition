package frc.robot.subsystems;
import java.io.File;
import java.io.IOException;

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
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
@SuppressWarnings("removal")
public class CoralCryus extends PIDSubsystem {
    private final SparkMax m_shoot;
    private final SparkMax m_pitch;
    private final SparkMax m_intake;
    private final RelativeEncoder m_encoder;
    private final PIDController m_pitchController = new PIDController(0 , 0 , 0);
    @SuppressWarnings("removal")
    public CoralCryus() {
        super(new PIDController(5,0,0),0);
        m_shoot = new SparkMax(13, MotorType.kBrushless);
        m_pitch = new SparkMax(14, MotorType.kBrushless);
        m_encoder = m_pitch.getEncoder();
        this.getController().disableContinuousInput();
        m_intake = new SparkMax(16 , MotorType.kBrushless);
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

    public void aim(int position) {
        switch (position) {
            case 1:
                setSetpoint(6969); // aim L1
                break;
            case 2:
                setSetpoint(657689); // aim L4
                break;
            case 3: 
                setSetpoint(218731283); // reload position
                break;
            default:
                break;
        }
    }
  


    public void periodic() {
        // This method will be called once per scheduler run
                
    }    

}   