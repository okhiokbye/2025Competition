package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.NoSuchElementException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
public class Vision extends SubsystemBase{
    private final AprilTagFieldLayout layout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
    StructPublisher<Pose2d> publisher3 = NetworkTableInstance.getDefault().getStructTopic("lewisistheworstpersoneverNEWPOSE1", Pose2d.struct).publish();
    StructPublisher<Pose2d> publisher4 = NetworkTableInstance.getDefault().getStructTopic("lewisisnotagoodpersonNEWPOSE2", Pose2d.struct).publish();
    public Vision(){
        
    }
     public Pose2d findLeftBranch(){
    long id =  NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(17); // get primary id
    
    double x_off = Units.inchesToMeters(5);
    double y_off = Units.inchesToMeters(9+15); //change to left offset later
    Pose2d goalPose = new Pose2d();
    try{
        goalPose = layout.getTagPose((int)id).get().toPose2d();
     } 
     catch(Exception NoSuchElementException){
         System.out.println("what the fuck");
     }

   
    Pose2d goalPose2 = new Pose2d(goalPose.getX() + x_off*Math.cos(Math.toRadians(90+goalPose.getRotation().getDegrees()))+y_off*Math.cos(goalPose.getRotation().getRadians()),
                                  goalPose.getY() + x_off*Math.sin(Math.toRadians(90+goalPose.getRotation().getDegrees()))+y_off*Math.sin(goalPose.getRotation().getRadians()),
                                  new Rotation2d(Math.PI + goalPose.getRotation().getRadians())); 

    // get offset targeted pose?
      return goalPose2; // TODO: find biggest apriltag in view, set pose to track as offset (see reef) 

    }

    public Pose2d findRightBranch(){
      long id =  NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(17); // get primary id
    
      double x_off = -Units.inchesToMeters(5);
      double y_off = Units.inchesToMeters(9+15); //change to left offset later
    
      Pose2d goalPose = new Pose2d();
    try{
       goalPose = layout.getTagPose((int)id).get().toPose2d();
    } 
    catch(Exception NoSuchElementException){
        System.out.println("what the fuck");
    }
      Pose2d goalPose2 = new Pose2d(goalPose.getX() + x_off*Math.cos(Math.toRadians(90+goalPose.getRotation().getDegrees()))+y_off*Math.cos(goalPose.getRotation().getRadians()),
                                    goalPose.getY() + x_off*Math.sin(Math.toRadians(90+goalPose.getRotation().getDegrees()))+y_off*Math.sin(goalPose.getRotation().getRadians()),
                                    new Rotation2d(Math.PI + goalPose.getRotation().getRadians())); 
  

     
      // get offset targeted pose?
      return goalPose2; // TODO: find biggest apriltag in view, set pose to track as offset (see reef) 
    }
    @Override
    public void periodic() {
        publisher3.set(findRightBranch());
        publisher4.set(findLeftBranch());
    }
}
