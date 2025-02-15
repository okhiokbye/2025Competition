package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.Map;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
public class DriveLewis extends SubsystemBase {
    private SwerveDrive swerveDrive;
    private NetworkTable table;
    private SwerveDrivePoseEstimator poseEstimator;
    private boolean doRejectUpdate = false;


    public DriveLewis() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        double maximumSpeed = Units.feetToMeters(15.5);
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
        swerveDrive.zeroGyro();
        Matrix stdDevs = new Matrix<N3,N1>(Nat.N3(), Nat.N1(), new double[] {0.7, 0.7, 9999999});
        swerveDrive.setVisionMeasurementStdDevs(stdDevs);
        poseEstimator = swerveDrive.swerveDrivePoseEstimator;

   
    }

    private double Yawo = 0;
    private double yawVelo = 0;
    
    public void periodic() {

        // NetworkTableEntry tx = table.getEntry("tx");
        // NetworkTableEntry ty = table.getEntry("ty");
        // NetworkTableEntry ta = table.getEntry("ta");

        // //read values periodically
        // double x = tx.getDouble(0.0);
        // double y = ty.getDouble(0.0);
        // double area = ta.getDouble(0.0); // WHAT % (0-100) OF APRIL TAG IT READS 

        // //post to smart dashboard periodically
        // SmartDashboard.putNumber("LimelightX", x);
        // SmartDashboard.putNumber("LimelightY", y);
        // SmartDashboard.putNumber("LimelightArea", area);

        
        poseEstimator.update(swerveDrive.getYaw(), getModulePositions());
        LimelightHelpers.SetRobotOrientation("limelight", poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        double Yawcurr = swerveDrive.getYaw().getDegrees();
        double changet = .02;

        yawVelo = (Yawcurr - Yawo) / changet;
        Yawo = Yawcurr;


        if (Math.abs(yawVelo) > 720) { // 720 degrees per second
            doRejectUpdate = true;
        }
        if (mt2.tagCount == 0) {
            doRejectUpdate = true;
        }

        // Add vision measurement to pose estimator
        if (!doRejectUpdate) {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
            poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
        }

        swerveDrive.addVisionMeasurement(mt2.pose,mt2.timestampSeconds);
          
        
        // Pose2d estimatedPose = poseEstimator.getEstimatedPosition();
        // SmartDashboard.putNumber("est x", estimatedPose.getX());
        // SmartDashboard.putNumber("est y", estimatedPose.getY());
        // SmartDashboard.putNumber("est angle", estimatedPose.getRotation().getDegrees());


        // double[] poseArray = new double[] {
        //   estimatedPose.getX(),
        //   estimatedPose.getY(),
        //   estimatedPose.getRotation().getDegrees()
        // };

        // NetworkTableInstance.getDefault().getTable("AdvantageScope").getEntry("rohotpose").setDoubleArray(poseArray);



  }

  
  
    public Pose2d getPose() {
     return swerveDrive.getPose();
    }

    public void resetPose(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public SwerveModulePosition[] getModulePositions() {
    Map<String, swervelib.SwerveModule> modules = swerveDrive.getModuleMap();
    return new SwerveModulePosition[] {
        modules.get("frontleft").getPosition(),
        modules.get("frontright").getPosition(),
        modules.get("backleft").getPosition(),
        modules.get("backright").getPosition()
    };
  }

    public ChassisSpeeds getChassisSpeeds(){
      Map<String, swervelib.SwerveModule> mapvelo = swerveDrive.getModuleMap();
      return swerveDrive.kinematics.toChassisSpeeds(
        mapvelo.get("backright").getState(), 
        mapvelo.get("frontleft").getState(), 
        mapvelo.get("frontright").getState(), 
        mapvelo.get("backleft").getState());
  }


    public void configureAutoBuilder(){
    RobotConfig config = null;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }


    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                                new PIDConstants(0.121, 0.0001, 0.01), // Translation PID constants
                                new PIDConstants(0.008, 0.0, 0.0064) // Rotation PID constants
                        ),
                        config, // The robot configuration
                        () -> {
                          // Boolean supplier that controls when the path will be mirrored for the red alliance
                          // This will flip the path being followed to the red side of the field.
                          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            
                          var alliance = DriverStation.getAlliance();
                          if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                          }
                          return false;
                        },
                        this // Reference to this subsystem to set requirements
                );
              }

  private void driveRobotRelative(ChassisSpeeds speeds) {
     // TODO Auto-generated method stub)
     swerveDrive.drive(speeds);
              }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
      SmartDashboard.putString("X: ", "heehee" + MathUtil.applyDeadband(translationX.getAsDouble(), 0.2));
      SmartDashboard.putString("Y: ", "heehee" + MathUtil.applyDeadband(translationY.getAsDouble(), 0.2));
      SmartDashboard.putString("Z: ","heehee" + Math.pow(MathUtil.applyDeadband(angularRotationX.getAsDouble(), 0.5), 3));
      return run(() -> {
        // Make the robot move
        swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
            MathUtil.applyDeadband(translationX.getAsDouble(), 0.2)* swerveDrive.getMaximumChassisVelocity(),
            MathUtil.applyDeadband(translationY.getAsDouble(), 0.2) * swerveDrive.getMaximumChassisVelocity()
                ),
            0.8),
            Math.pow(MathUtil.applyDeadband(angularRotationX.getAsDouble(), 0.5), 3) 
                * swerveDrive.getMaximumChassisAngularVelocity(),
            true,
            false);
      });
    }
}
