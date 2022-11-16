package frc3512.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhotonVision implements VisionIO {
  private static String visionCamName = "mmal_service_16.1";
  private static String driverCamName = "HD_USB_Camera";
  private PhotonCamera m_visionCamera;
  private PhotonCamera m_driverCamera;

  private PhotonPipelineResult m_result;
  private PhotonTrackedTarget m_bestTarget;
  private boolean haveTargets = false;

  public VisionIOPhotonVision() {
    m_visionCamera = new PhotonCamera(visionCamName);
    m_driverCamera = new PhotonCamera(driverCamName);
    m_driverCamera.setDriverMode(true);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.haveTargets = haveTargets;
    inputs.area = haveTargets ? m_bestTarget.getArea() : 0;
    inputs.pitch = haveTargets ? m_bestTarget.getPitch() : 0;
    inputs.yaw = haveTargets ? m_bestTarget.getYaw() : 0;
  }

  @Override
  public void updateVision() {
    m_result = m_visionCamera.getLatestResult();
    m_bestTarget = m_result.getBestTarget();
    haveTargets = m_result.hasTargets();
  }
}
