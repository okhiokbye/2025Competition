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
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Commands.*;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class CoralSubsystem extends SubsystemBase {
    private final SparkMax m_outtakecoral;
    private final SparkMax m_magicBox;
    
    public CoralSubsystem() {
        m_outtakecoral = new SparkMax(13, MotorType.kBrushless);
        m_magicBox = new SparkMax(14, MotorType.kBrushless);
    }

    public Command shootCoral() {
        return new StartEndCommand(
            //set motor power manually below
            () -> m_outtakecoral.set(-1),
            () -> m_outtakecoral.set(0)
        );
    }

    public Command shootMagicBoxLow() {
    //set rotation limit in # of rotations below
      //  m_magicBox.setSoftLimit(SoftLimitDirection.kForward, 0.25);
      //  m_magicBox.enableSoftLimit(SoftLimitDirection.kForward, true);
        return new StartEndCommand(
            () -> m_magicBox.set(1),
            () -> m_magicBox.set(0)
        );
    }

    public Command shootMagicBoxHigh() {
    //set rotation limit in # of rotations below
       // m_magicBox.setSoftLimit(SoftLimitDirection.kForward, 0.25);
       // m_magicBox.enableSoftLimit(SoftLimitDirection.kForward, true);
        return new StartEndCommand(
            () -> m_magicBox.set(1),
            () -> m_magicBox.set(0)
        );
    }

    public void periodic() {
        // This method will be called once per scheduler run
     //   m_magicBox.setSoftLimit(SoftLimitDirection.kReverse, 0);
      //  m_magicBox.enableSoftLimit(SoftLimitDirection.kReverse, true);
        m_magicBox.set(-1);
    }    

}   