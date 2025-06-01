package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.Servo;

public class ExtendoServoSystem extends AbstractSystem
{
	private final float RETRACTED = 0.64f;
	private final float HALF = 0.37f;
	private final float SAMPLE = 0.485f;
	private final float SECOND_SAMPLE = 0.477f;
	private final float THIRD_SAMPLE = 0.5f;
	private final float EXTENDED = 0.33f;

	private ExtendoLevel _level = ExtendoLevel.RETRACTED;

	private final Servo extendoServo;

	public ExtendoServoSystem(Servo extendoServo) {
		this.extendoServo = extendoServo;
	}

	@Override
	public void init() {
		_level = ExtendoLevel.RETRACTED;
		extendoServo.setPosition(RETRACTED);
	}

	public void extend(ExtendoLevel level) {
		_level = level;
		switch (level) {
			case RETRACTED:
				extendoServo.setPosition(RETRACTED);
				break;
			case HALF:
				extendoServo.setPosition(HALF);
				break;
			case SAMPLE:
				extendoServo.setPosition(SAMPLE);
				break;
			case SECOND_SAMPLE:
				extendoServo.setPosition(SECOND_SAMPLE);
				break;
			case THIRD_SAMPLE:
				extendoServo.setPosition(THIRD_SAMPLE);
				break;
			case EXTENDED:
				extendoServo.setPosition(EXTENDED);
				break;
		}
	}

	public Action extendAction(ExtendoLevel level) {
		return new InstantAction(() -> extend(level));
	}

	public ExtendoLevel getExtendoLevel() {
		return _level;
	}

	public enum ExtendoLevel {
		RETRACTED,
		HALF,
		SAMPLE,
		SECOND_SAMPLE,
		THIRD_SAMPLE,
		EXTENDED
	}
}
