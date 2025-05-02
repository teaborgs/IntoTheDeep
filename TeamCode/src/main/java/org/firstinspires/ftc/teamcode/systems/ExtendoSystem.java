package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.Servo;

public class ExtendoSystem extends AbstractSystem
{
	private final float RETRACTED = 0.637f; //0.81 0.51 0.16
	private final float HALF = 0.37f; //0.65
	private final float SAMPLE = 0.485f; //0.65
	private final float SECOND_SAMPLE = 0.477f; //0.65
	private final float THIRD_SAMPLE = 0.5f; //0.65
	private final float EXTENDED = 0.5f; //0.5 0.83 0.41

	private ExtendoLevel _level = ExtendoLevel.RETRACTED;

	private final Servo extendo;

	public ExtendoSystem(Servo extendo)
	{
		this.extendo = extendo;


	}

	@Override
	public void init()
	{
		_level = ExtendoLevel.RETRACTED;
		extendo.setPosition(RETRACTED);

	}

	public void extend(ExtendoLevel level)
	{
		_level = level;
		switch (level)
		{
			case RETRACTED:
				extendo.setPosition(RETRACTED);
				break;
			case HALF:
				extendo.setPosition(HALF);
				break;
			case SAMPLE:
				extendo.setPosition(SAMPLE);
				break;
			case SECOND_SAMPLE:
				extendo.setPosition(SECOND_SAMPLE);
				break;
			case THIRD_SAMPLE:
				extendo.setPosition(THIRD_SAMPLE);
				break;
			case EXTENDED:
				extendo.setPosition(EXTENDED);
				break;
		}
	}

	public Action extendAction(ExtendoLevel level)
	{
		return new InstantAction(() -> extend(level));
	}

	public ExtendoLevel getExtendoLevel()
	{
		return _level;
	}

	public enum ExtendoLevel
	{
		RETRACTED,
		HALF,
		SAMPLE,
		SECOND_SAMPLE,
		THIRD_SAMPLE,
		EXTENDED
	}
}