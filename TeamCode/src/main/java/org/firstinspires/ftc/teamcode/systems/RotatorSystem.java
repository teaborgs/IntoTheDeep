package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.Servo;

public class RotatorSystem extends AbstractSystem
{
	public static final float ROTATOR_BASE = 0.476f;

	private final Servo rotator;

	public RotatorSystem(Servo rotator)
	{
		this.rotator = rotator;
	}

	@Override
	public void init()
	{
		rotator.setPosition(ROTATOR_BASE);
	}

	public void setPosition(double position)
	{
		rotator.setPosition(position);
	}

	public Action setPositionAction(double position)
	{
		return new InstantAction(() -> setPosition(position));
	}
}