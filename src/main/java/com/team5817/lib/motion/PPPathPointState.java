package com.team5817.lib.motion;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Util;

public class PPPathPointState {
    protected final Pose2d mPose;
    protected final Rotation2d mMotionDirection;
    protected final double mCurvature;
    protected final double mHeading_rate;
    protected final double mVelocity;
    protected final double mAcceleration;
    protected final double mT;

    public PPPathPointState(){
        mPose = Pose2d.identity();
        mMotionDirection = Rotation2d.identity();
        mCurvature = 0;
        mVelocity = 0;
        mAcceleration = 0;
        mHeading_rate = 0;
        mT = 0;
    }

    public PPPathPointState(Pose2d pose, Rotation2d motion_direction, double curvature, double velocity, double acceleration, double t, double heading_rate){
        this.mHeading_rate = heading_rate;
        this.mPose = pose;
        this.mMotionDirection = motion_direction;
        this.mCurvature = curvature;
        this.mVelocity = velocity;
        this.mAcceleration = acceleration;
        this.mT = t;
    }

    public Pose2d getPose(){
        return mPose;
    }   

    public PPPathPointState transformBy(Pose2d transform){
        return new PPPathPointState(mPose.transformBy(transform), mMotionDirection, mCurvature,mVelocity, mAcceleration, mT, mHeading_rate);
    }

    public PPPathPointState mirror(){
        return new PPPathPointState(mPose.mirror().getPose(), mMotionDirection.mirror(), -mCurvature, mVelocity, mAcceleration, mT, mHeading_rate);
    }

    public PPPathPointState mirrorAboutX(double x){
        return new PPPathPointState(mPose.mirrorAboutX(x), mMotionDirection.mirrorAboutX(), -mCurvature, mVelocity, mAcceleration, mT, mHeading_rate);
    }

    public PPPathPointState mirrorAboutY(double y){
        return new PPPathPointState(mPose.mirrorAboutY(y), mMotionDirection.mirrorAboutY(), -mCurvature, mVelocity, mAcceleration, mT, mHeading_rate);
    }

    public double getmCurvature(){
        return mCurvature;
    }

    public double getVelocity(){        
        return mVelocity;
    }

    public double getAcceleration(){
        return mAcceleration;
    }
   
    public Translation2d getTranslation(){
        return mPose.getTranslation();    
    }   

    public PPPathPointState interpolate(final PPPathPointState other, double x) {
        return new PPPathPointState(getPose().interpolate(other.getPose(), x),
                mMotionDirection.interpolate(other.mMotionDirection, x),
                Util.interpolate(getmCurvature(), other.getmCurvature(), x),
                Util.interpolate(getVelocity(), other.getVelocity(), x),
                Util.interpolate(getVelocity(), other.getVelocity(), x),
                Util.interpolate(mT, other.t(), x),
                Util.interpolate(getHeadingRate(), other.getHeadingRate(), x)
                );
    }

    public double getHeadingRate(){
        return mHeading_rate;
    }
    public double t(){
        return mT;
    }
    public PPPathPointState rotateBy(Rotation2d rotation){
        return new PPPathPointState(mPose.rotateBy(rotation), Rotation2d.identity(), mCurvature, mVelocity, mAcceleration, mT, mHeading_rate); 
    }
    
    public PPPathPointState add(PPPathPointState other){
        return this.transformBy(other.getPose());
    }
    
    public Rotation2d getCourse(){        
        return mMotionDirection;
    }



    
}
