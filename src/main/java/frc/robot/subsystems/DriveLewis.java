package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
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
    StructPublisher<Pose3d> publisher1 = NetworkTableInstance.getDefault().getStructTopic("lewissucksihatehimandwanthimtodie", Pose3d.struct).publish();
    StructPublisher<Pose3d> publisher2 = NetworkTableInstance.getDefault().getStructTopic("idonthatelewis", Pose3d.struct).publish();
    StructPublisher<Pose2d> publisher3 = NetworkTableInstance.getDefault().getStructTopic("lewisistheworstpersoneverNEWPOSE1", Pose2d.struct).publish();
    StructPublisher<Pose2d> publisher4 = NetworkTableInstance.getDefault().getStructTopic("lewisisnotagoodpersonNEWPOSE2", Pose2d.struct).publish();
    
    StructPublisher<Pose3d> yagslPosePub = NetworkTableInstance.getDefault().getStructTopic("yagslPose", Pose3d.struct).publish();
    private double Yawo = 0;
    private double yawVelo = 0;
    private final AprilTagFieldLayout layout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();


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
        //swerveDrive.getGyro().setOffset(new Rotation3d(0,0,Math.PI));
       
        Optional<Alliance> alliance = DriverStation.getAlliance();
        boolean rorb = alliance.orElse(Alliance.Red) == DriverStation.Alliance.Red;

        if (rorb){
          LimelightHelpers.setCameraPose_RobotSpace("limelight", .355, 0.0127, 0.0762, 0, 0, 0);
        }
        else {
          LimelightHelpers.setCameraPose_RobotSpace("limelight", -.355, 0.0127, 0.0762, 0, 0, 180);
        }
      

          
    }

    
    
    public void periodic() {

        //poseEstimator.update(swerveDrive.getYaw(), getModulePositions());
        LimelightHelpers.SetRobotOrientation("limelight", getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);

    //     LimelightHelpers.PoseEstimate botposeMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    //     // double[] deft = {1.0,1.0,1.0,1.0,1.0,1.0};
    //     // double[] botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(deft);
    //  // LimelightHelpers.LimelightResults.getBotPose3d_wpiBlue();
        
    //     // double tx = botposeMT2[0];
    //     // double ty = botposeMT2[1];
    //     // double tz = botposeMT2[2];
    //     //  double roll = botpose[3];
    //     //  double pitch = botpose[4];
    //     //  double yaw = botpose[5];

    //     // Pose3d robotPose = new Pose3d(
    //     //   tx, ty, tz,
    //     //   new Rotation3d(roll, pitch, yaw)
    //     // );
        
    //    Pose3d posenew = LimelightHelpers.getBotPose3d_wpiBlue("limelight");

    //     double yawRad1 = posenew.getRotation().getZ();
    //     double yawRad2 = botposeMT2.pose.getRotation().getRadians();
    //    // double yawDeg = Math.toDegrees(yawRad);

    //     // double tx = posenew.getTranslation().getX();
    //     // double ty = posenew.getTranslation().getY();

    //     double mtx = botposeMT2.pose.getX();
    //     double mty = botposeMT2.pose.getY();
        


    //    // Pose3d correctedPose = new Pose3d(tx, ty, 0, new Rotation3d(0, 0, yawRad));
    //     Pose3d lewispose1 = new Pose3d(mtx, mty, 0, new Rotation3d(0, 0, yawRad1));
    //     Pose3d lewispose2 = new Pose3d(mtx, mty, 0, new Rotation3d(0, 0, yawRad2));

       // Pose3d robotPose = new Pose3d(botposeMT2.pose.getX(), botposeMT2.pose.getY(),0, new Rotation3d(0, 0, botposeMT2.pose.getRotation().getDegrees()));


        // publisher.set(robotPose);

        // publisher1.set(lewispose1); 
        // publisher2.set(lewispose2); // NEW ONE



        // publisher3.set(findRightBranch());
        // publisher4.set(findLeftBranch());


        // publisher.set(robotPose);
        // NetworkTableInstance.getDefault().getTable("AdvantageScope").getEntry("rohotpose").setDoubleArray(robotPose);

        // SmartDashboard.putNumber("lime x", tx);
        // SmartDashboard.putNumber("schlime y", ty);
        // SmartDashboard.putNumber("lime yaw", Math.toDegrees(yaw));
        // SmartDashboard.putString("Botpose Array", Arrays.toString(botpose));
        

        // //read values periodically
        // double x = tx.getDouble(0.0);
        // double y = ty.getDouble(0.0);

        // // //post to smart dashboard periodically
        // // SmartDashboard.putNumber("LimelightX", x);
        // // SmartDashboard.putNumber("LimelightY", y);
        // // SmartDashboard.putNumber("LimelightArea", area);

        
        
       // LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        // double Yawcurr = swerveDrive.getYaw().getDegrees();
        // double changet = .02;

        // yawVelo = (Yawcurr - Yawo) / changet;
        // Yawo = Yawcurr;


        // if (Math.abs(yawVelo) > 720) { // 720 degrees per second
        //     doRejectUpdate = true;
        // }
        // if (botposeMT2.tagCount == 0) {  
        //     doRejectUpdate = true;
        // }

        // // Add vision measurement to pose estimator
        // if (!doRejectUpdate) {
        //   Matrix stdDevs = new Matrix<N3,N1>(Nat.N3(), Nat.N1(), new double[] {0.7, 0.7, 9999999});
        //   swerveDrive.setVisionMeasurementStdDevs(stdDevs);
        //   swerveDrive.addVisionMeasurement(botposeMT2.pose,botposeMT2.timestampSeconds);
        // }

        //   yagslPosePub.set(new Pose3d(swerveDrive.swerveDrivePoseEstimator.getEstimatedPosition()));

        
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

    private boolean hasTarget() { // js checks if limelight there
      return LimelightHelpers.getTV("");
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
  public Command zeroGyroCommand(){
    return Commands.runOnce( ()-> swerveDrive.zeroGyro());
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
                                new PIDConstants(1, 0, 0), // Translation PID constants
                                new PIDConstants(1, 0, 0) // Rotation PID constants
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



    // public Pose2d findLeftBranch(){
    // long id =  NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(0); // get primary id
    
    // double x_off = Units.inchesToMeters(5);
    // double y_off = Units.inchesToMeters(9+15); //change to left offset later
   

    // Pose2d goalPose = new Pose2d();
    // try{
    //     goalPose = layout.getTagPose((int)id).get().toPose2d();
    //     System.out.println(id);
    //  } 
    //  catch(Exception NoSuchElementException){
    //      System.out.println("what the fuck");
    //  }

    // Pose2d goalPose2 = new Pose2d(goalPose.getX() + x_off*Math.cos(Math.toRadians(90+goalPose.getRotation().getDegrees()))+y_off*Math.cos(goalPose.getRotation().getRadians()),
    //                               goalPose.getY() + x_off*Math.sin(Math.toRadians(90+goalPose.getRotation().getDegrees()))+y_off*Math.sin(goalPose.getRotation().getRadians()),
    //                               new Rotation2d(Math.PI + goalPose.getRotation().getRadians())); 

    // // get offset targeted pose?
    //   return goalPose2; // TODO: find biggest apriltag in view, set pose to track as offset (see reef) 

  //  }
    public BooleanSupplier isValidTarget(){
      return ()-> (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(0) != 0);
    }
  //   public Pose2d findRightBranch(){
  //     long id =  NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(0); // get primary id
    
  //     double x_off = -Units.inchesToMeters(5);
  //     double y_off = Units.inchesToMeters(9+15); //change to left offset later
  //     Pose2d goalPose = new Pose2d();
  //     try{
  //         goalPose = layout.getTagPose((int)id).get().toPose2d();
  //         System.out.println(id);
  //      } 
  //      catch(Exception NoSuchElementException){
  //          System.out.println("what the fuck");
  //      }
  
  //     Pose2d goalPose2 = new Pose2d(goalPose.getX() + x_off*Math.cos(Math.toRadians(90+goalPose.getRotation().getDegrees()))+y_off*Math.cos(goalPose.getRotation().getRadians()),
  //                                   goalPose.getY() + x_off*Math.sin(Math.toRadians(90+goalPose.getRotation().getDegrees()))+y_off*Math.sin(goalPose.getRotation().getRadians()),
  //                                   new Rotation2d(Math.PI + goalPose.getRotation().getRadians())); 
  
  //  // TODO: IF NO ID AVAILABLE, THEN REFUSE TO RUN
     
  //     // get offset targeted pose?
  //     return goalPose2; // TODO: find biggest apriltag in view, set pose to track as offset (see reef) 
  //   }
    

   






    public Command driveToPose(Pose2d pose) { 
      // Create the constraints to use while pathfinding
      PathConstraints constraints = new PathConstraints(
          swerveDrive.getMaximumChassisVelocity(), 4.0,
          swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
      // Since AutoBuilder is configured, we can use it to build pathfinding commands
      return AutoBuilder.pathfindToPose(
          pose,
          constraints,
          edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
      );
    }
    private void driveRobotRelative(ChassisSpeeds speeds) {
     // TODO Auto-generated method stub)k 
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
                ),0.8),
            MathUtil.applyDeadband(angularRotationX.getAsDouble(), 0.2) 
                * swerveDrive.getMaximumChassisAngularVelocity(),
            true,
            false);
      });
    }
}