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

public class CoralCryus extends SubsystemBase {
    private final SparkMax m_outtakecoral;
    private final SparkMax m_magicBox;
    
    private final RelativeEncoder m_encoder;
    private final PIDController m_magicBoxController = new PIDController(0 , 0 , 0);
    
    public CoralCryus() {
        m_outtakecoral = new SparkMax(13, MotorType.kBrushless);
        m_magicBox = new SparkMax(14, MotorType.kBrushless);
        m_encoder = m_magicBox.getEncoder();
    }


    public Command shootCoral() {
        return new StartEndCommand(
            //set motor power manually below
            () -> m_outtakecoral.set(-1),
            () -> m_outtakecoral.set(0)
        );
    }

    public Command shootMagicBoxLow() {
    double posLow = m_encoder.getPosition();    
    double magicBoxAngle = m_magicBoxController.calculate(posLow, 0.1042);
        return new RunCommand(() -> m_magicBox.set(magicBoxAngle));
    }
    
    public Command shootMagicBoxHigh() {
    double posLow = m_encoder.getPosition();    
    double magicBoxAngle = m_magicBoxController.calculate(posLow, 0.25);
        return new RunCommand(() -> m_magicBox.set(magicBoxAngle));
    }

    public void periodic() {
        // This method will be called once per scheduler run
                
    }    

}   