package com.team5817.frc2024.controlboard;

import com.team5817.frc2024.subsystems.Drive;
import com.team5817.frc2024.subsystems.LEDs;
import com.team5817.frc2024.subsystems.Superstructure;

public class DriverControls {

	ControlBoard mControlBoard = ControlBoard.getInstance();

	Superstructure mSuperstructure = Superstructure.getInstance();
	Drive mDrive = Drive.getInstance();
	LEDs mLEDs = LEDs.getInstance();

	/* ONE CONTROLLER */

	public void oneControllerMode() {
			mDrive.overrideHeading(true);
	}

	/* TWO CONTROLLERS */

	public void twoControllerMode() {
	}

}
