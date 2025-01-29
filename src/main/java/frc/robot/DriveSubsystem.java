package frc.robot;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class DriveSubsystem extends SubsystemBase {
    private SwerveDrive swerveDrive;

    public DriveSubsystem() {
        double maximumSpeed = Units.feetToMeters(4.5);
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        swerveDrive.setModuleEncoderAutoSynchronize(true, 0.50);
        swerveDrive.setAutoCenteringModules(false);
    }

    /**
     * Command to drive the robot using translative values and heading as angular
     * velocity.
     *
     * @param translationX     Translation in the X direction.
     * @param translationY     Translation in the Y direction.
     * @param angularRotationX Rotation of the robot to set
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier angularRotationX) {
            SmartDashboard.putString("X: ", "heehee" + MathUtil.applyDeadband(translationX.getAsDouble(),0.2));
            SmartDashboard.putString("Y: " , "heehee" + MathUtil.applyDeadband(translationY.getAsDouble(),0.2));
            SmartDashboard.putString("Z: ", "heehee"+    Math.pow(MathUtil.applyDeadband(angularRotationX.getAsDouble(),0.5),3));
            return run(() -> {
            // Make the robot move
            swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                    MathUtil.applyDeadband(translationX.getAsDouble(),0.2) * 0,//swerveDrive.getMaximumChassisVelocity(),
                    MathUtil.applyDeadband(translationY.getAsDouble(),0.2) * swerveDrive.getMaximumChassisVelocity()*0), 0.8),
                    Math.pow(MathUtil.applyDeadband(angularRotationX.getAsDouble(),0.5),3)*0 * swerveDrive.getMaximumChassisAngularVelocity(),
                    true,
                    false);
        });
    }
}
