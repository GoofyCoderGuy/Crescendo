package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.FieldConstants;
import frc.robot.Constants.Constants.LimelightConstants;
import frc.robot.Constants.Constants.ShooterConstants;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotContainer.SWERVE;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.io.IOException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class LimelightInterface extends SubsystemBase {

   // private static final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");


    private PhotonCamera tagCam;
    private PhotonCamera noteCam;
    private static boolean tag = false;
    public boolean Error = false;
    public boolean LED = false;
    public boolean upadate = false;
    AprilTagFieldLayout fieldLayout;
    double notecameraheight = Units.inchesToMeters(15.5);


	Transform3d camPose = new Transform3d(
			
        //new Translation3d(Units.inchesToMeters(12.25), -Units.inchesToMeters(10.875), Units.inchesToMeters(11)),
	    new Translation3d(Units.inchesToMeters(1.911942), -Units.inchesToMeters(10.8125), Units.inchesToMeters(10.634)),//24.974 //11.6661	//10.634 //	
        new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(180)));//37.4// try negative pitch
	PhotonPoseEstimator photonPoseEstimator;

    private Pose2d NoteCamPose = new Pose2d(Units.inchesToMeters(-16),Units.inchesToMeters(-5.5), new Rotation2d());

    InterpolatingTreeMap<Double, Double> shooterInterp;

 

    public LimelightInterface() {
        shooterInterp = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

        shooterInterp.put(0.9826, 0.0);//subwoofer
        shooterInterp.put(1.5525, 0.09);//starting line
        shooterInterp.put(2.2, .19);
        shooterInterp.put(2.7, .232);
        shooterInterp.put(2.95, 0.24);
        shooterInterp.put(3.21, 0.263);//podium
        shooterInterp.put(3.4, 0.276);//middle stage
        shooterInterp.put(3.6, 0.28);
        shooterInterp.put(3.8, 0.285);
        shooterInterp.put(4.0, 0.288);
        shooterInterp.put(4.5, 0.302);
        shooterInterp.put(4.7, 0.314);
        shooterInterp.put(5.23, 0.329);
        shooterInterp.put(5.54, 0.332);
        shooterInterp.put(6.03, 0.341);
        shooterInterp.put(6.2739, 0.348);
        shooterInterp.put(6.512,0.3521);
        shooterInterp.put(6.75, 0.3571);
        shooterInterp.put(7.0, 0.362);

        try {
			fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
		} catch (IOException e) {
			System.out.println("Couldn't Find April Tag Layout File");
			e.printStackTrace();
		}

		tagCam = new PhotonCamera("Tag_Camera");
        //noteCam = new PhotonCamera("Note_cam");

        

		//noteCam = new PhotonCamera("Global_Shutter_Camera");
		photonPoseEstimator =
				new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, tagCam, camPose);
		photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

   
    @Override
    public void periodic() {

        

        SmartDashboard.putNumber("SpeakerSetPoint", getSetPoint());
        SmartDashboard.putBoolean("April Tag", tagCam.getLatestResult().getTargets().size() == 2);
        SmartDashboard.putNumber("distance", getSpeakerDistance());
        SmartDashboard.putNumber("photonDistance",PhotonUtils.calculateDistanceToTargetMeters(camPose.getZ(), LimelightConstants.aprilTag_Height, Units.degreesToRadians(37.4), 0));
        if(tagCam.getLatestResult().getTargets().isEmpty()){
            
        }else{
            SmartDashboard.putNumber("ID", tagCam.getLatestResult().getTargets().get(0).getFiducialId());
        }
            //SmartDashboard.putString("Angle", notePose().toString());
        
        Logger.recordOutput("Distance", getSpeakerDistance());
        Logger.recordOutput("Tags/TwoTag", tagCam.getLatestResult().getTargets().size() == 2);
        Logger.recordOutput("Tags/Number", tagCam.getLatestResult().getTargets().size());
        Logger.recordOutput("Update", upadate);
    }

    public Optional<EstimatedRobotPose> getEstimatedPose() {
		if (tagCam.getLatestResult().getTargets().size() != 2){
            upadate = false;
            return Optional.empty();
        }
        if(tagCam.getLatestResult().getTargets().get(0).getFiducialId() == 5 || tagCam.getLatestResult().getTargets().get(0).getFiducialId() == 6 ||
        tagCam.getLatestResult().getTargets().get(0).getFiducialId() == 11 || tagCam.getLatestResult().getTargets().get(0).getFiducialId() == 12 ||
        tagCam.getLatestResult().getTargets().get(0).getFiducialId() == 13 || tagCam.getLatestResult().getTargets().get(0).getFiducialId() == 14 ||
        tagCam.getLatestResult().getTargets().get(0).getFiducialId() == 15 || tagCam.getLatestResult().getTargets().get(0).getFiducialId() == 16){
            upadate = false;
            return Optional.empty();
        }
        upadate = true;
        return photonPoseEstimator.update();
        
        
	}
 
    public double getSpeakerDistance() {
        var speakerPose = (RobotContainer.IsRed() ? FieldConstants.Speaker_Red_Pose : FieldConstants.Speaker_Blue_Pose);

        return SWERVE.getPose().getTranslation().getDistance(speakerPose);
    }
    public double getAmpDistance() {
        var ampPose = (RobotContainer.IsRed() ? FieldConstants.Amp_Red_Pose : FieldConstants.Amp_Blue_Pose); //first should be red

        return SWERVE.getPose().getTranslation().getDistance(ampPose) - Units.inchesToMeters(14);
    }
    public Rotation2d getSpeakerAngle() {
        var speakerPose = (RobotContainer.IsRed() ? FieldConstants.Speaker_Red_Pose : FieldConstants.Speaker_Blue_Pose);

        return SWERVE.getPose().getTranslation().minus(speakerPose).getAngle();
    }

    public Rotation2d getLeadingSpeakerAngle() {
        Translation2d speakerPose = (RobotContainer.IsRed() ? FieldConstants.Speaker_Red_Pose : FieldConstants.Speaker_Blue_Pose);
        double flightTime = getSpeakerDistance() / ShooterConstants.noteVelocity;
        Translation2d leadingSpeakerPose = speakerPose.minus(new Translation2d(0, SWERVE.getYVelocity() * flightTime));
        Logger.recordOutput("LeadTarget", leadingSpeakerPose);
        return SWERVE.getPose().getTranslation().minus(leadingSpeakerPose).getAngle();
    }
    public Rotation2d getLeadingAmpAngle() {
        Translation2d ampPose = (RobotContainer.IsRed() ? FieldConstants.Amp_Red_Pose : FieldConstants.Amp_Blue_Pose);
        double flightTime = getAmpDistance() / ShooterConstants.noteVelocity;
        Translation2d leadingAmpPose = ampPose.minus(new Translation2d(0, SWERVE.getYVelocity() * flightTime));
        Logger.recordOutput("LeadAmpTarget", leadingAmpPose);
        return SWERVE.getPose().getTranslation().minus(leadingAmpPose).getAngle();
    }

    public double getAngleToSpeaker() {
        double distance = getSpeakerDistance();

        return 56.3231 - (6.9771 * distance);
    }

    public double getSetPoint() {
        return shooterInterp.get(getSpeakerDistance());
        // DO NOT DELETE THE FOLLOWING EQUATION:  return (0.484411 - (0.0058831 * getAngleToSpeaker()));
    }

    public double getTargetAngle() {

        double flightTime = getSpeakerDistance() / ShooterConstants.noteVelocity;
        double dropDistance = (9.8 / 2) * (flightTime * flightTime);
        Logger.recordOutput("DropDistance", dropDistance);
        double newHeight = dropDistance + LimelightConstants.speakerGoalHeight;
        var speakerPose = (RobotContainer.IsRed() ? FieldConstants.Speaker_Red_Pose : FieldConstants.Speaker_Blue_Pose);

        Logger.recordOutput("Target", new Pose3d(speakerPose.getX(), speakerPose.getY(), newHeight, new Rotation3d()));
        return Units.radiansToDegrees(Math.atan(Math.abs(newHeight / getSpeakerDistance())));
    }

    // public Optional<Pose2d> notePose(){
    //     if(noteCam == null) return  Optional.empty();
    //     if(!noteCam.getLatestResult().hasTargets()) return  Optional.empty();
    //     PhotonTrackedTarget target = noteCam.getLatestResult().getBestTarget();
    //     if(target == null) return  Optional.empty();
    //     double pitch = Units.degreesToRadians(target.getPitch());
    //     double yaw = Units.degreesToRadians(target.getYaw());
    //     double dx = notecameraheight / Math.tan(pitch);
    //     double dy = dx * Math.tan(yaw);
    //     Transform2d notepose = new Transform2d(dx,dy,new Rotation2d());
    //     if(pitch > 10) return  Optional.empty();
    //     else return Optional.of(NoteCamPose.plus(notepose));
    // }

    //  public Translation2d getNoteAngle() {
      
    //         Translation2d NotePose = notePose().get().getTranslation();
    //         return SWERVE.getPose().getTranslation().plus(NotePose);
    // }
}