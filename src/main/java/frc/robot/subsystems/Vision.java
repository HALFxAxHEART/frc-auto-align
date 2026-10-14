package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.*;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.RobotLog;
import frc.robot.Constants.VisionConstants.Camera;
import frc.robot.Constants.VisionConstants.CameraPose;

/**
 * WHAT THIS FILE IS: the vision subsystem. The Limelight sees AprilTags (the square barcodes on the field), works out
 * where the robot is from them, and this class passes that to the drivetrain to fix its position.
 * Wheel odometry alone slowly drifts (wheels slip, robots get pushed); vision pulls it back to the truth.
 * <p>
 * Feeds every Limelight's AprilTag pose into the drivetrain's pose estimator.
 * <p>
 * Enabled: MegaTag2. It takes the heading from the Pigeon and only corrects X/Y, which keeps the
 * pose steady (MegaTag1 single-tag headings jump around).
 * <p>
 * Disabled: MegaTag1 with 2+ tags is allowed to fix the heading too, so MegaTag2 starts the match
 * with a correct heading even if the robot was placed crooked.
 * <p>
 * Cameras are listed in {@code Constants.VisionConstants.kCameras}. Any number of Limelights works: each one is
 * handled the same way and adds its own position measurement. Each camera's mount position must be correct (set in
 * code with configuredInCode, or on its web page) or its positions will be wrong.
 * <p>
 * Dashboard per camera: Vision/&lt;name&gt;/Connected, Tags seen, Correction (m), Heading disagreement (deg).
 */
public class Vision extends SubsystemBase {
    /** How often to re-send the in-code camera settings while disabled, seconds (in case a Limelight rebooted) */
    private static final double kResendConfigSeconds = 1.0;

    private final CommandSwerveDrivetrain m_drivetrain;
    private final double m_createdTime = Timer.getFPGATimestamp();
    private double m_lastTagTime = 0.0;
    private double m_lastConfigSendTime = 0.0;
    /** Last heartbeat number from each camera, and when it last changed. Used to tell if a camera is connected. */
    private final Map<String, Double> m_lastHeartbeat = new HashMap<>();
    private final Map<String, Double> m_lastHeartbeatChange = new HashMap<>();

    public Vision(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        // Dashboard switch: false = ignore the cameras completely (pure wheel odometry)
        SmartDashboard.putBoolean("Vision/Use camera", true);
        sendCameraConfigs();
    }

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();
        // Settings sent from code are kept by NetworkTables, but re-send while disabled in case a camera rebooted
        if (DriverStation.isDisabled() && now - m_lastConfigSendTime > kResendConfigSeconds) {
            sendCameraConfigs();
        }

        for (Camera camera : kCameras) {
            checkConnected(camera.name(), now);
            if (SmartDashboard.getBoolean("Vision/Use camera", true)) {
                updateCamera(camera.name());
            }
        }
        SmartDashboard.putNumber("Vision/Seconds since last tag", now - m_lastTagTime);
    }

    /** Sends the mount position and pipeline of every camera set up with configuredInCode */
    private void sendCameraConfigs() {
        m_lastConfigSendTime = Timer.getFPGATimestamp();
        for (Camera camera : kCameras) {
            if (camera.pipeline() != null) {
                LimelightHelpers.setPipelineIndex(camera.name(), camera.pipeline());
            }
            CameraPose pose = camera.pose();
            if (pose != null) {
                LimelightHelpers.setCameraPose_RobotSpace(camera.name(),
                    pose.forwardM(), pose.sideM(), pose.upM(), pose.rollDeg(), pose.pitchDeg(), pose.yawDeg());
            }
        }
    }

    /** A Limelight's "hb" (heartbeat) number goes up every frame. If it stops changing, the camera is gone. */
    private void checkConnected(String camera, double now) {
        double heartbeat = LimelightHelpers.getLimelightNTDouble(camera, "hb");
        if (!m_lastHeartbeat.containsKey(camera) || heartbeat != m_lastHeartbeat.get(camera)) {
            m_lastHeartbeat.put(camera, heartbeat);
            m_lastHeartbeatChange.put(camera, now);
        }
        boolean connected = heartbeat != 0 && now - m_lastHeartbeatChange.get(camera) < kHeartbeatTimeoutSeconds;
        SmartDashboard.putBoolean("Vision/" + camera + "/Connected", connected);
        if (now - m_createdTime > kBootGraceSeconds) {
            RobotLog.eventOnRise("limelight " + camera, !connected, "LIMELIGHT DISCONNECTED: " + camera);
        }
    }

    /** @return Whether any camera has given a pose in the last {@code seconds} seconds. */
    public boolean seenTagRecently(double seconds) {
        return Timer.getFPGATimestamp() - m_lastTagTime < seconds;
    }

    private void updateCamera(String camera) {
        var driveState = m_drivetrain.getState();

        // MegaTag2 needs the robot's heading every loop, so send it even if we skip this frame
        LimelightHelpers.SetRobotOrientation(camera, driveState.Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        // Vision is unreliable while spinning fast
        double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
        if (Math.abs(omegaRps) >= kMaxSpinRps) {
            return;
        }

        if (DriverStation.isDisabled()) {
            PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(camera);
            if (mt1 != null && mt1.tagCount >= kMegaTag1MinTags) {
                recordCorrection(camera, driveState.Pose, mt1);
                m_drivetrain.addVisionMeasurement(mt1.pose, mt1.timestampSeconds,
                    VecBuilder.fill(kMegaTag1StdDev, kMegaTag1StdDev, kMegaTag1StdDev));
            }
        } else {
            PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camera);
            if (mt2 != null && mt2.tagCount > 0) {
                recordCorrection(camera, driveState.Pose, mt2);
                // Huge heading std dev = ignore the vision heading, trust the Pigeon
                m_drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds,
                    VecBuilder.fill(kMegaTag2XYStdDev, kMegaTag2XYStdDev, 9999999));
            }
        }
    }

    /** How far the camera disagrees with odometry each time it sees tags. Small (under ~0.3m) = odometry is
     *  tracking well on its own. Big = odometry drifts when it can't see tags (wheel size, gearing, or gyro). */
    private void recordCorrection(String camera, Pose2d odometryPose, PoseEstimate estimate) {
        m_lastTagTime = Timer.getFPGATimestamp();
        // Drawn on the field in AdvantageScope; compare with DriveState/Pose to see odometry drift
        RobotLog.pose("Vision/" + camera, estimate.pose);
        SmartDashboard.putNumber("Vision/" + camera + "/Tags seen", estimate.tagCount);
        SmartDashboard.putNumber("Vision/" + camera + "/Correction (m)",
            odometryPose.getTranslation().getDistance(estimate.pose.getTranslation()));
        SmartDashboard.putNumber("Vision/" + camera + "/Heading disagreement (deg)",
            odometryPose.getRotation().minus(estimate.pose.getRotation()).getDegrees());
    }
}
