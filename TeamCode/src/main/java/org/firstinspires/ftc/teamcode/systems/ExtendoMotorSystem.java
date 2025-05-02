package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ExtendoMotorSystem extends AbstractSystem {
	private final DcMotorEx extendo;

	private final int RETRACTED = 0;
	private final int HALF = 300;
	private final int EXTENDED = 800;

	private ExtendoLevel _level = ExtendoLevel.RETRACTED;

	public ExtendoMotorSystem(DcMotorEx extendo) {
		this.extendo = extendo;
		this.extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		this.extendo.setTargetPosition(0);
		this.extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		this.extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		this.extendo.setDirection(DcMotor.Direction.REVERSE);
	}

	@Override
	public void init() {
		extend(ExtendoLevel.RETRACTED);
	}

	public void extend(ExtendoLevel level) {
		_level = level;
		int targetPosition = getTargetPositionForLevel(level);
		extendo.setTargetPosition(targetPosition);
		extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		extendo.setPower(1.0);
	}

	public Action extendoAction(ExtendoLevel level) {
		return new InstantAction(() -> {
			extend(level);
		});
	}

	public ExtendoLevel getExtendoLevel() {
		return _level;
	}

	private int getTargetPositionForLevel(ExtendoLevel level) {
		switch (level) {
			case RETRACTED:
				return RETRACTED;
			case HALF:
				return HALF;
			case EXTENDED:
				return EXTENDED;
			default:
				return RETRACTED;
		}
	}

	public enum ExtendoLevel {
		RETRACTED,
		HALF,
		EXTENDED
	}
}
