package com.team5817.frc2024.subsystems.vision;

import com.team5817.frc2024.Constants;
import com.team5817.frc2024.RobotState;
import com.team5817.frc2024.Constants.PoseEstimatorConstants;
import com.team5817.frc2024.RobotState.VisionUpdate;
import com.team5817.frc2024.loops.ILooper;
import com.team5817.frc2024.loops.Loop;
import com.team5817.frc2024.subsystems.Subsystem;
import com.team5817.lib.TunableNumber;
import com.team254.lib.util.MovingAverage;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;
import java.util.Optional;

import javax.lang.model.util.ElementScanner14;

import org.littletonrobotics.junction.Logger;
import org.opencv.core.Core;
import org.opencv.imgproc.Subdiv2D;

public class VisionDeviceManager extends Subsystem {

	private static VisionDeviceManager mInstance;

	public static VisionDeviceManager getInstance() {
		if (mInstance == null) {
			mInstance = new VisionDeviceManager();
		}
		return mInstance;
	}

	private VisionDevice mDomCamera;
	private VisionDevice mSubCamera;

	private List<VisionDevice> mAllCameras;

	private static TunableNumber timestampOffset = new TunableNumber("VisionTimestampOffset", (0.1), false);

	private MovingAverage mHeadingAvg = new MovingAverage(100);
	private double mMovingAvgRead = 0.0;

	private static boolean disable_vision = false;

	private VisionDeviceManager() {
		mDomCamera = new VisionDevice("limelight-dom");
		mSubCamera = new VisionDevice("limelight-sub");
		mAllCameras = List.of(mDomCamera, mSubCamera);
	}

	@Override
	public void registerEnabledLoops(ILooper enabledLooper){
		enabledLooper.register(new Loop() {
			@Override
			public void onLoop(double timestamp) {
			if(mDomCamera.getVisionUpdate().isPresent()&&mSubCamera.getVisionUpdate().isPresent()){
				for(VisionDevice device:checkEpipolar(mSubCamera, mDomCamera)){
					RobotState.getInstance().addVisionUpdate(device.getVisionUpdate().get());
				}//TODO add traditional filtering 	
			}else{
				for(VisionDevice device: mAllCameras){
					if(device.getVisionUpdate().isPresent())
						RobotState.getInstance().addVisionUpdate(device.getVisionUpdate().get());
				}
			}
			
			}

			@Override
			public void onStart(double timestamp) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public void onStop(double timestamp) {
				// TODO Auto-generated method stub
				
			}
		});
	}

	@Override
	public void readPeriodicInputs() {
		mAllCameras.forEach(VisionDevice::readPeriodicInputs);
		mMovingAvgRead = mHeadingAvg.getAverage();
	}

	@Override
	public void writePeriodicOutputs() {
		mAllCameras.forEach(VisionDevice::writePeriodicOutputs);
	}

	@Override
	public void outputTelemetry() {
		mAllCameras.forEach(VisionDevice::outputTelemetry);
		SmartDashboard.putNumber("Vision heading moving avg", getMovingAverageRead());
		SmartDashboard.putBoolean("vision disabled", visionDisabled());
	}

	public VisionDevice getBestDevice(){
		if(mDomCamera.getVisionUpdate().get().getTa()>mSubCamera.getVisionUpdate().get().getTa())
			return mDomCamera;
		return mSubCamera;
	}

	public List<VisionDevice> checkEpipolar(VisionDevice domDevice,VisionDevice subDevice){
		Transform3d expectedDelta = PoseEstimatorConstants.kDomVisionDevice.kRobotToCamera.plus(PoseEstimatorConstants.kSubVisionDevice.kRobotToCamera.inverse());
		Transform3d delta;
		delta = new Transform3d(domDevice.getVisionUpdate().get().getTargetToCamera(),
				subDevice.getVisionUpdate().get().getTargetToCamera());
		Transform3d error = delta.plus(expectedDelta.inverse());
		Logger.recordOutput("PoseEstimator/Expected Transform", expectedDelta);
		Logger.recordOutput("PoseEstimator/Real Transform", delta);
		Logger.recordOutput("PoseEstimator/error",error);
		if(error.getTranslation().getNorm()>0.1||error.getRotation().getAngle()>0.5)//TODO Find threshold (meters and radians)
			return List.of(getBestDevice());
		return List.of(domDevice,subDevice);
		}
	public Double getMovingAverageRead() {
		return mMovingAvgRead;
	}

	public synchronized MovingAverage getMovingAverage() {
		return mHeadingAvg;
	}


	public synchronized VisionDevice getLeftVision() {
		return mDomCamera;
	}

	public synchronized VisionDevice getRightVision() {
		return mSubCamera;
	}

	public static double getTimestampOffset() {
		return timestampOffset.get();
	}

	public static boolean visionDisabled() {
		return disable_vision;
	}

	public static void setDisableVision(boolean disable) {
		disable_vision = disable;
	}

    public boolean fullyConnected() {
        return false;
    }
}
