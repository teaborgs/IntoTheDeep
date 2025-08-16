package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class PinpointEncoder {
	private final DcMotorEx motor;
	private final boolean reversed;

	public PinpointEncoder(Object pinpoint, boolean reversed, DcMotorEx motor) {
		this.motor = motor;
		this.reversed = reversed;
	}

	public int getCurrentPosition() {
		return reversed ? -motor.getCurrentPosition() : motor.getCurrentPosition();
	}
}
