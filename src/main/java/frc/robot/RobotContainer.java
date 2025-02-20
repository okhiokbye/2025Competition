// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveLewis;
import frc.robot.subsystems.ElevatorJustin;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.CoralCryus;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

import java.io.File;
import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandJoystick m_driverController = new CommandJoystick(0);
  private final CommandJoystick m_aimJoystick = new CommandJoystick(1);
  private final DriveLewis m_swerve = new DriveLewis();
  private final CoralCryus m_outtakecoral = new CoralCryus();
  private final ElevatorJustin m_elevator = new ElevatorJustin();

  
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_swerve.configureAutoBuilder();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    
    m_swerve.setDefaultCommand(m_swerve.driveCommand(
      () -> -m_driverController.getRawAxis(1), 
      ()-> -m_driverController.getRawAxis(0), 
      ()->-m_driverController.getRawAxis(2)
    ));

    m_aimJoystick.button(1).onTrue(m_outtakecoral.shootCoral().withTimeout(0.2));
    m_aimJoystick.button(2).onTrue(m_outtakecoral.shootMagicBoxLow());
    m_aimJoystick.button(3).onTrue(m_outtakecoral.shootMagicBoxHigh());
    
    m_aimJoystick.button(4).onTrue(m_elevator.elevate(1));
    m_aimJoystick.button(5).onTrue(m_elevator.elevate(2));
    m_aimJoystick.button(6).onTrue(m_elevator.elevate(3));
    m_aimJoystick.button(7).onTrue(m_elevator.elevate(4));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PathPlannerAuto("2xl4");
  }
}
