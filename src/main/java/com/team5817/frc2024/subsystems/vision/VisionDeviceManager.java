package com.team5817.frc2024.subsystems.vision;

import com.team5817.frc2024.Constants;
import com.team5817.frc2024.subsystems.Subsystem;
import com.team5817.lib.TunableNumber;
import com.team254.lib.util.MovingAverage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;

import org.opencv.core.Core;

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

	public Double getMovingAverageRead() {
		return mMovingAvgRead;
	}

	public synchronized MovingAverage getMovingAverage() {
		return mHeadingAvg;
	}

	public synchronized boolean fullyConnected() {
		return mDomCamera.isConnected() && mSubCamera.isConnected();
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
}
