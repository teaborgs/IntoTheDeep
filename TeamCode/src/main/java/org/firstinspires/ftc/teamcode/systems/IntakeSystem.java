package org.firstinspires.ftc.teamcode.systems;

import static org.team11260.fastload.ReloadIntentListener.stop;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeSystem extends AbstractSystem
{
	private final DcMotorEx intakeMotor;

	public enum IntakeDirection
	{
		FORWARD,
		REVERSE,
		STOP
	}

	public IntakeSystem(DcMotorEx intakeMotor)
	{
		this.intakeMotor = intakeMotor;
		this.intakeMotor.setDirection(DcMotor.Direction.FORWARD);
		this.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		this.intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		stop(); // Ensure it's stopped on init
	}

	@Override
	public void init()
	{
		stop();
	}

	private void stop()
	{
		intakeMotor.setPower(0f);
	}

	public void setIntakeDirection(IntakeDirection direction)
	{
		switch (direction)
		{
			case FORWARD:
				intakeMotor.setPower(1f);
				break;
			case REVERSE:
				intakeMotor.setPower(-1f);
				break;
			case STOP:
			default:
				intakeMotor.setPower(0f);
				break;
		}
	}
}



