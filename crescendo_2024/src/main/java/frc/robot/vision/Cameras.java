package frc.robot.vision;

import frc.robot.vision.CameraSubsystem;
public class Cameras extends CameraSubsystem{
    /* I alone can see the truth
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
    }*/


}
