package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.Servo;

public class OpenCloseSystem extends AbstractSystem
{
	private float OPEN;
	private float CLOSED;

	private final Servo servo;

	public OpenCloseSystem(Servo servo, float open, float closed)
	{
		this.servo = servo;
		OPEN = open;
		CLOSED = closed;
	}

	@Override
	public void init()
	{
		servo.setPosition(OPEN);
	}

	public void open()
	{
		servo.setPosition(OPEN);
	}

	public void close()
	{
		servo.setPosition(CLOSED);
	}

	public Action openAction()
	{
		return new InstantAction(this::open);
	}

	public Action closeAction()
	{
		return new InstantAction(this::close);
	}
}