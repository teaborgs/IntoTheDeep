package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.Servo;

public class OpenCloseSystem extends AbstractSystem {
	private float OPEN;
	private float CLOSED;
	private final Servo servo;

	private boolean isOpen = false; // Track state

	public OpenCloseSystem(Servo servo, float open, float closed) {
		this.servo = servo;
		OPEN = open;
		CLOSED = closed;
	}

	@Override
	public void init() {
		open(); // Or setPosition(OPEN); and isOpen = true;
	}

	public void open() {
		servo.setPosition(OPEN);
		isOpen = true;
	}

	public void close() {
		servo.setPosition(CLOSED);
		isOpen = false;
	}

	public boolean isOpen() {
		return isOpen;
	}
}
