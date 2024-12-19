// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5817.lib.swerve;

import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.SynchronousPIDF;

/** Add your docs here. */
public class SwerveHeadingController {
    private Rotation2d targetHeading = Rotation2d.fromDegrees(0);
    private double lastTimestamp;

    private double disabledStartTimestamp = 0;
    private boolean atTarget = false;
    private boolean isDisabled = false;
    public void disableHeadingController(boolean disable) {
        disabledStartTimestamp = lastTimestamp;
        isDisabled = disable;
    }

    private SynchronousPIDF openLoopStabilizeController;
    private SynchronousPIDF velocityStabilizeController;
	private SynchronousPIDF openLoopSnapController;
    private SynchronousPIDF velocitySnapController;
	
	enum State{
		Snap,
		Stabilize
	}
	State currentState = State.Stabilize;
	
    public SwerveHeadingController() {
        openLoopStabilizeController = new SynchronousPIDF(0.00025, 0.0, 0, 1.0);
        velocityStabilizeController = new SynchronousPIDF(0.001, 0.0, 0, 100);

		openLoopSnapController = new SynchronousPIDF(0.00025, 0.0, 0, 1.0);//TODO FIND
        velocitySnapController = new SynchronousPIDF(0.001, 0.0, 0, 100);
		
    }
    public void setTargetStabilizeHeading(Rotation2d heading) {
        targetHeading = Rotation2d.fromDegrees(heading.getDegrees());
        atTarget = false;
		currentState = State.Stabilize;
    }
	public void setTargetSnapHeading(Rotation2d heading) {
        targetHeading = Rotation2d.fromDegrees(heading.getDegrees());
        atTarget = false;
		currentState = State.Snap;
    }
    public double update(Rotation2d heading, double timestamp) {
        if(isDisabled) {
            if((timestamp - disabledStartTimestamp) > 0.25) {
                isDisabled = false;
				if(currentState == State.Stabilize)
                setTargetStabilizeHeading(heading);
				else
				setTargetSnapHeading(heading);
            }
            return 0;
        }
        return getRotationCorrection(heading, timestamp);
    }


    public boolean atTarget(){
        return atTarget;
    }
    public double getRotationCorrection(Rotation2d heading, double timestamp) {
        double error = new Rotation2d(targetHeading).rotateBy(heading.inverse()).getDegrees();
        double dt = timestamp - lastTimestamp;
        lastTimestamp = timestamp;
        if(Math.abs(error)< 2.5){
            atTarget = true;
        }
		double correctionForce;
		if(currentState == State.Stabilize)
        correctionForce = openLoopStabilizeController.calculate(error, dt);
		else
		correctionForce = openLoopSnapController.calculate(error, dt);

        if(Math.abs(correctionForce) > 0.017){
            correctionForce = .0075* Math.signum(correctionForce);
        }
        return correctionForce;
    }

    public double getVelocityCorrection(double error, double timestamp) {
        double dt = timestamp - lastTimestamp;
        lastTimestamp = timestamp;
        if(Math.abs(error)< 3){
            atTarget = true;
        }
		double correctionForce;
		if(currentState == State.Stabilize)
        correctionForce = velocityStabilizeController.calculate(error, dt);
		else
		correctionForce = velocitySnapController.calculate(error, dt);
       return correctionForce;
    }

    public double getTargetHeading(){
        return targetHeading.getDegrees();
    }

}
