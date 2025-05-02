package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.Servo;

public class TumblerSystem extends AbstractSystem
{
	private float TRANSFER;
	private float IDLE;
	private float HOLD;
	private float HOVER;
	private float BUSY;

	private TumblerDestination _destination = TumblerDestination.IDLE;

	private final Servo tumbler;

	public TumblerSystem(Servo tumbler)
	{
		this(tumbler, 0, 0, 0, 0, 0);
	}

	public TumblerSystem(Servo tumbler, float transfer, float idle, float hold, float hover, float pickup)
	{
		this(tumbler, transfer, idle, hold, hover, pickup, false);
	}

	public TumblerSystem(Servo tumbler, float transfer, float idle, float hold, float hover, float pickup, boolean reverse)
	{
		this.tumbler = tumbler;
		TRANSFER = transfer;
		IDLE = idle;
		HOLD = hold;
		HOVER = hover;
		BUSY = pickup;
		if (reverse) tumbler.setDirection(Servo.Direction.REVERSE);

	}

	@Override
	public void init()
	{
		_destination = TumblerDestination.IDLE;
		tumbler.setPosition(IDLE);
	}

	public void setDestination(TumblerDestination destination)
	{
		_destination = destination;
		switch (destination)
		{
			case TRANSFER:
				tumbler.setPosition(TRANSFER);
				break;
			case IDLE:
				tumbler.setPosition(IDLE);
				break;
			case HOLD:
				tumbler.setPosition(HOLD);
				break;
			case HOVER:
				tumbler.setPosition(HOVER);
				break;
			case BUSY:
				tumbler.setPosition(BUSY);
				break;
		}
	}

	public Action setDestinationAction(TumblerDestination destination)
	{
		return new InstantAction(() -> setDestination(destination));
	}

	public TumblerDestination getDestination()
	{
		return _destination;
	}

	public enum TumblerDestination {
		TRANSFER,
		IDLE,
		HOLD,
		HOVER,
		BUSY
	}
}