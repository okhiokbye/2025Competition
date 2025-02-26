// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;

import frc.robot.subsystems.DriveLewis;
import frc.robot.subsystems.ElevatorJustin;

import frc.robot.subsystems.Vision;
import frc.robot.subsystems.CoralCryus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

import java.io.File;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandJoystick m_driverController = new CommandJoystick(0);
  private final CommandJoystick m_aimJoystick = new CommandJoystick(1);
  private final DriveLewis m_swerve = new DriveLewis();
  private final Vision m_vision = new Vision();
  private final CoralCryus m_shooter = new CoralCryus();
  private final ElevatorJustin m_elevator = new ElevatorJustin();

  
  public RobotContainer() {
    NamedCommands.registerCommand("elevateL1", m_elevator.elevate(1));
    NamedCommands.registerCommand("elevateL2", m_elevator.elevate(2));
    NamedCommands.registerCommand("elevateL3", m_elevator.elevate(3));
    NamedCommands.registerCommand("elevateL4", m_elevator.elevate(4));
    NamedCommands.registerCommand("elevateL0", m_elevator.elevate(1)); // reload

    NamedCommands.registerCommand("aimL1", m_shooter.aim(1)); 
    NamedCommands.registerCommand("aimL4", m_shooter.aim(2));
    NamedCommands.registerCommand("aimL0", m_shooter.aim(3)); // reload 
    NamedCommands.registerCommand("shoot", m_shooter.shootCoral());
    NamedCommands.registerCommand("intake", m_shooter.intake());
    // NamedCommands.registerCommand("getBeamBreak",  m_shooter.beamBreak()); figure out what pathplanner needs for intake stopping later - or maybe it doesnt matter
   
    // Configure the trigger bindings
    m_swerve.configureAutoBuilder();
    configureBindings();
 
    PathfindingCommand.warmupCommand().schedule();
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

    m_aimJoystick.button(1).onTrue(m_shooter.shootCoral().withTimeout(0.5));
    m_aimJoystick.button(2).onTrue(m_shooter.intake().until(m_shooter.beamBreak()));
  
     m_aimJoystick.button(3).onTrue(m_elevator.runOnce(()->m_elevator.setSetpoint(m_elevator.getSetpoint()-(0.05))) ); //fine tune for zero routine
     m_aimJoystick.button(4).onTrue(m_elevator.runOnce(()->m_elevator.setSetpoint(m_elevator.getSetpoint()+(0.05))) ); //fine tune for zero routine
     m_aimJoystick.button(6).and(m_elevator.getLimit()).onTrue(m_elevator.zero()); // zero if the limit switch is also triggered

     m_aimJoystick.button(9).onTrue(Commands.parallel(m_elevator.elevate(1), m_shooter.aim(4)).andThen(m_shooter.aim(1))); // aim L1
     m_aimJoystick.button(10).onTrue(Commands.parallel(m_elevator.elevate(2), m_shooter.aim(4)).andThen(m_shooter.aim(2))); // aim L2
     m_aimJoystick.button(11).onTrue(Commands.parallel(m_elevator.elevate(3), m_shooter.aim(4)).andThen(m_shooter.aim(3))); // aim L3
     m_aimJoystick.button(12).onTrue(Commands.parallel(m_elevator.elevate(4), m_shooter.aim(4)).andThen(m_shooter.aim(4))); // aim L4
     m_aimJoystick.button(2).onTrue(Commands.parallel(m_elevator.elevate(4), m_shooter.aim(4)).andThen(m_shooter.aim(5)));
   
     //AUTOALIGn BUTTONS ARE 7 AND 8, LEFT AND RIGHT REEF BRANCH RESPECTIVELY
     
     m_driverController.button(7).onTrue(m_swerve.driveToPose(m_vision.findRightBranch()));
     m_driverController.button(8).onTrue(m_swerve.driveToPose(m_vision.findLeftBranch())); 
    
    
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
