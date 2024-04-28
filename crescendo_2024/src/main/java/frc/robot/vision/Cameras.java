package frc.robot.vision;

/*import org.photonvision.PhotonCamera;
import org.photonvision.estimation.VisionEstimation;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class Cameras extends SubsystemBase{
    I alone can see the truth
    PhotonCamera camA;
    PoseEstimator poseEstimator;
    DriveSubsystem driveSubsystem;


    public Cameras(DriveSubsystem odometryBase){
        this.driveSubsystem = odometryBase;
        camA = new PhotonCamera(NetworkTableInstance.getDefault(), "Cam 1");

        poseEstimator = new PoseEstimator<>(null, odometryBase.m_odometry, null, null);
    }

    @Override
    public void periodic(){
        var resultA = camA.getLatestResult();


        boolean hasTargetsA = resultA.hasTargets();


        if (hasTargetsA){
            var imageCaputreTime = resultA.getTimestampSeconds();
            var camToTargetTrans = resultA.getBestTarget().getBestCameraToTarget();
            var camPose = Constants.visionConstants.kFarTargetPose.transformBy(camToTargetTrans.inverse());
            poseEstimator.addVisionMeasurement(
                camPose.transformBy(Constants.visionConstants.kCameraToRobot).toPose2d(), imageCaputreTime);

            driveSubsystem.resetOdometry(poseEstimator.getEstimatedPosition());
        }
    }


}
*/