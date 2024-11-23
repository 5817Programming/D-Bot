package com.team5817.lib.swerve;


import com.uni.lib.motion.PPPathPointState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DriveMotionPlanner {
	// Pure Pursuit Constants
	public static final double kPathLookaheadTime = 0.1; // From 1323 (2019)
	public static final double kPathMinLookaheadDistance = 0.3; // From 1323 (2019)
	public static final double kAdaptivePathMinLookaheadDistance = 0.15;
	public static final double kAdaptivePathMaxLookaheadDistance = 0.61;
	public static final double kAdaptiveErrorLookaheadCoefficient = 0.01;

	public enum FollowerType {
		PID,
		PURE_PURSUIT,
	}

	FollowerType mFollowerType = FollowerType.PID;
	public void setFollowerType(FollowerType type) {
		mFollowerType = type;
	}

	private double defaultCook = 0.5;
	private boolean useDefaultCook = true;

	public void setDefaultCook(double new_value) {
		defaultCook = new_value;
	}

	TrajectoryIterator mCurrentTrajectory;
	boolean mIsReversed = false;
	double mLastTime = Double.POSITIVE_INFINITY;
	public PPPathPointState mLastSetpoint = null;
	public PPPathPointState mSetpoint = new PPPathPointState();
	Pose2d mError = Pose2d.identity();

	ErrorTracker mErrorTracker = new ErrorTracker(14 * 100);
	HeadingController mHeadingController = new HeadingController();
	Translation2d mTranslationalError = Translation2d.identity();
	Rotation2d mPrevHeadingError = Rotation2d.identity();
	Pose2d mCurrentState = Pose2d.identity();

	double mCurrentTrajectoryLength = 0.0;
	double mTotalTime = Double.POSITIVE_INFINITY;
	double mStartTime = Double.POSITIVE_INFINITY;
	ChassisSpeeds mOutput = new ChassisSpeeds();

	Lookahead mSpeedLookahead = null;

	// PID controllers for path following
	double mDt = 0.0;

	public DriveMotionPlanner() {}

	public void setTrajectory(final TrajectoryIterator trajectory) {
		mCurrentTrajectory = trajectory;
		mSetpoint = trajectory.getCurrentState();
		mLastSetpoint = null;
		useDefaultCook = true;
		mSpeedLookahead = new Lookahead(
				kAdaptivePathMinLookaheadDistance,
				kAdaptivePathMaxLookaheadDistance,
				0.0,
				Constants.SwerveMaxspeedMPS);
		mCurrentTrajectoryLength =
				mCurrentTrajectory.getTimeView().last_interpolant();
	}

	public void reset() {
		mErrorTracker.reset();
		mTranslationalError = Translation2d.identity();
		mPrevHeadingError = Rotation2d.identity();
		mLastSetpoint = null;
		mOutput = new ChassisSpeeds();
		mLastTime = Double.POSITIVE_INFINITY;
	}

	protected ChassisSpeeds updatePIDChassis(ChassisSpeeds chassisSpeeds) {
		// Feedback on longitudinal error (distance).
		final double kPathk =
			DriverStation.getAlliance().get() == Alliance.Blue? 3:-3; 
			// 2.4;/* * Math.ypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)*/;//0.15;
		Twist2d pid_error = Pose2d.log(mError);
		chassisSpeeds.vxMetersPerSecond = (chassisSpeeds.vxMetersPerSecond * 1) - kPathk * pid_error.dx;
		chassisSpeeds.vyMetersPerSecond = (chassisSpeeds.vyMetersPerSecond * .8) + kPathk * pid_error.dy;
		// chassisSpeeds.vxMetersPerSecond = (chassisSpeeds.vxMetersPerSecond * 1) ;
		// chassisSpeeds.vyMetersPerSecond = (chassisSpeeds.vyMetersPerSecond * 1) ;
	
		chassisSpeeds.omegaRadiansPerSecond = mHeadingController.getVelocityCorrection(-mError.getRotation().getDegrees(), Timer.getFPGATimestamp());
		// chassisSpeeds.omegaRadiansPerSecond = 0;
		return chassisSpeeds;
	}

	protected ChassisSpeeds updatePurePursuit(Pose2d current_state, double feedforwardOmegaRadiansPerSecond) {
		double lookahead_time = kPathLookaheadTime;
		final double kLookaheadSearchDt = 0.01;
		PPPathPointState lookahead_state =
				mCurrentTrajectory.preview(lookahead_time);
		double actual_lookahead_distance = mSetpoint.getPose().distance(lookahead_state.getPose());
		double adaptive_lookahead_distance = mSpeedLookahead.getLookaheadForSpeed(mSetpoint.getVelocity())
				+ kAdaptiveErrorLookaheadCoefficient * mError.getTranslation().norm();

	while (actual_lookahead_distance < adaptive_lookahead_distance
				&& mCurrentTrajectory.getRemainingProgress() > lookahead_time) {
			lookahead_time += kLookaheadSearchDt;
			lookahead_state = mCurrentTrajectory.preview(lookahead_time);
			actual_lookahead_distance = mSetpoint.getPose().distance(lookahead_state.getPose());
		}

		// If the Lookahead Point's Distance is less than the Lookahead Distance transform it so it is the lookahead
		// distance away
		if (actual_lookahead_distance < adaptive_lookahead_distance) {
			lookahead_state = new PPPathPointState(
 							lookahead_state
									.getPose()
									.transformBy(Pose2d.fromTranslation(new Translation2d(
													 (kPathMinLookaheadDistance - actual_lookahead_distance),
											0.0))),
                            lookahead_state.getCourse(),
                            lookahead_state.getmCurvature(),
                            lookahead_state.getVelocity(),
                            lookahead_state.getAcceleration(),
                            lookahead_state.t(),
							lookahead_state.getHeadingRate()
							);

           		}
		if (lookahead_state.getVelocity() == 0.0) {
			mCurrentTrajectory.advance(Double.POSITIVE_INFINITY);
			return new ChassisSpeeds();
		}

		// Find the vector between robot's current position and the lookahead state
		Translation2d lookaheadTranslation = new Translation2d(
				current_state.getTranslation(), lookahead_state.getPose().getTranslation());

		// Set the steering direction as the direction of the vector
		Rotation2d steeringDirection = lookaheadTranslation.direction();

		// Convert from field-relative steering direction to robot-relative
		steeringDirection = steeringDirection.rotateBy(current_state.inverse().getRotation());

		// Use the Velocity Feedforward of the Closest Point on the Trajectory
		double normalizedSpeed = Math.abs(mSetpoint.getVelocity()) / Constants.SwerveMaxspeedMPS;

		// The Default Cook is the minimum speed to use. So if a feedforward speed is less than defaultCook, the robot
		// will drive at the defaultCook speed
		if (normalizedSpeed > defaultCook || mSetpoint.t() > (mCurrentTrajectoryLength / 2.0)) {
			useDefaultCook = false;
		}
		if (useDefaultCook) {
			normalizedSpeed = defaultCook;
		}

		SmartDashboard.putNumber("PurePursuit/NormalizedSpeed", normalizedSpeed);

		// Convert the Polar Coordinate (speed, direction) into a Rectangular Coordinate (Vx, Vy) in Robot Frame
		final Translation2d steeringVector =
				new Translation2d(steeringDirection.cos() * normalizedSpeed, steeringDirection.sin() * normalizedSpeed);
		ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
				steeringVector.x() * Constants.SwerveMaxspeedMPS,
				steeringVector.y() * Constants.SwerveMaxspeedMPS,
				feedforwardOmegaRadiansPerSecond);

		// Use the PD-Controller for To Follow the Time-Parametrized Heading
		final double kThetakP = 3.5;
		final double kThetakD = 0.0;
		final double kPositionkP = 2.0;

		chassisSpeeds.vxMetersPerSecond = chassisSpeeds.vxMetersPerSecond
				+ kPositionkP * mError.getTranslation().x();
		chassisSpeeds.vyMetersPerSecond = chassisSpeeds.vyMetersPerSecond
				+ kPositionkP * mError.getTranslation().y();
		chassisSpeeds.omegaRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond
				+ (kThetakP * mError.getRotation().getRadians())
				+ kThetakD * ((mError.getRotation().getRadians() - mPrevHeadingError.getRadians()) / mDt);
		return chassisSpeeds;
	}

	public ChassisSpeeds update(double timestamp, Pose2d current_state) {
		if (mCurrentTrajectory == null) return null;

		if (!Double.isFinite(mLastTime)) mLastTime = timestamp;
		mDt = timestamp - mLastTime;
		mLastTime = timestamp;
		PPPathPointState sample_point;
		mCurrentState = current_state;

			// Compute error in robot frame
			mPrevHeadingError = mError.getRotation();
			mError = current_state.inverse().transformBy(mSetpoint.getPose());
			mErrorTracker.addObservation(mError);


            switch (mFollowerType) {
				case PID:
 				sample_point = mCurrentTrajectory.advance(mDt);
				// RobotState.getInstance().setDisplaySetpointPose(Pose2d.fromTranslation(RobotState.getInstance().getFieldToOdom(timestamp)).transformBy(sample_point.state().state().getPose()));
				mSetpoint = sample_point;

				final double velocity_m = mSetpoint.getVelocity();
				// Field relative
				var course = mSetpoint.getCourse();
				Rotation2d motion_direction = course;
				// Adjust course by ACTUAL heading rather than planned to decouple heading and translation errors.

				var chassis_speeds = new ChassisSpeeds(
						velocity_m * motion_direction.cos(),
						velocity_m * motion_direction.sin(),
						0
						);
				mOutput = updatePIDChassis(chassis_speeds);
	                   
                    break;
                case PURE_PURSUIT:
 				double searchStepSize = 1.0;
				double previewQuantity = 0.0;
				double searchDirection = 1.0;
				double forwardDistance = distance(current_state, previewQuantity + searchStepSize);
				double reverseDistance = distance(current_state, previewQuantity - searchStepSize);
				searchDirection = Math.signum(reverseDistance - forwardDistance);
				while (searchStepSize > 0.001) {
					SmartDashboard.putNumber("PurePursuit/PreviewDist", distance(current_state, previewQuantity));
					if (Util.epsilonEquals(distance(current_state, previewQuantity), 0.0, 0.0003937)) break;
					while (
					/* next point is closer than current point */ distance(
									current_state, previewQuantity + searchStepSize * searchDirection)
							< distance(current_state, previewQuantity)) {
						/* move to next point */
						previewQuantity += searchStepSize * searchDirection;
					}
					searchStepSize /= 10.0;
					searchDirection *= -1;
				}
				SmartDashboard.putNumber("PurePursuit/PreviewQtd", previewQuantity);
				sample_point = mCurrentTrajectory.advance(previewQuantity);
				// RobotState.getInstance().setDisplaySetpointPose(Pose2d.fromTranslation(RobotState.getInstance().getFieldToOdom(timestamp)).transformBy(sample_point.state().state().getPose()));
				mSetpoint = sample_point;
				mOutput = updatePurePursuit(current_state, 0.0);
	                   
                    break;
            }
		return mOutput;
	}

	public Pose2d getEndPosition() {
		return mCurrentTrajectory.getTimeView().sample(mCurrentTrajectoryLength).getPose();
	}

	public synchronized Translation2d getTranslationalError() {
		return new Translation2d(
				mError.getTranslation().x(), mError.getTranslation().y());
	}

	public synchronized Rotation2d getHeadingError() {
		return mError.getRotation();
	}

	private double distance(Pose2d current_state, double additional_progress) {
		return mCurrentTrajectory
				.preview(additional_progress)
				.getPose()
				.distance(current_state);
	}

	public synchronized PPPathPointState getSetpoint() {
		return mSetpoint;
	}

	public synchronized ErrorTracker getErrorTracker() {
		return mErrorTracker;
	}
}