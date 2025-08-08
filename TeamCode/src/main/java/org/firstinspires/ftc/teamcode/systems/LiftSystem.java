package org.firstinspires.ftc.teamcode.systems;

import static org.firstinspires.ftc.teamcode.Utilities.RunSequentially;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class LiftSystem extends AbstractSystem
{
	// IDLE, BASKET_LOW, BASKET_HIGH, CLEAR_SPECIMEN, CHAMBER_HIGH_FIXED_CLAW, CHAMBER_HIGH_PLACE_FIXED_CLAW, SUSPEND, CHAMBER_HIGH, CHAMBER_HIGH_PLACE
	private final int[] LIFT_LEVELS = { 100, 580, 1320, 200, 780, 500, 600, 360, 770 };
	public int TOLERANCE = 50;

	private final DcMotorEx lift1, lift2;

	public LiftSystem(DcMotorEx lift1, DcMotorEx lift2)
	{
		this.lift1 = lift1;
		this.lift2 = lift2;
		lift1.setDirection(DcMotor.Direction.FORWARD);
		lift2.setDirection(DcMotor.Direction.REVERSE);
		lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		lift1.setTargetPositionTolerance(TOLERANCE);
		lift2.setTargetPositionTolerance(TOLERANCE);
	}

	@Override
	public void init()
	{
		setLiftLevel(LiftLevel.IDLE);
		lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	}

	public void autoPower()
	{
		if (isLiftBusy()) {
			lift1.setPower(0.4);
			lift2.setPower(0.4);
		} else {
			lift1.setPower(0);
			lift2.setPower(0);
		}
	}

	public void setLiftLevel(LiftLevel level)
	{
		lift1.setTargetPosition(LIFT_LEVELS[level.ordinal()]);
		lift2.setTargetPosition(LIFT_LEVELS[level.ordinal()]);
	}

	public void setLiftPower(double power)
	{
		lift1.setPower(power);
		lift2.setPower(power);
	}

	public Action moveLiftTo(LiftLevel level)
	{
		return RunSequentially(
				new InstantAction(() -> setLiftLevel(level)),
				telemetryPacket -> isLiftBusy()
		);
	}

	public boolean isLiftBusy()
	{
		return lift1.isBusy() || lift2.isBusy();
	}

	public enum LiftLevel
	{
		IDLE,
		BASKET_LOW,
		BASKET_HIGH,
		CLEAR_SPECIMEN,
		CHAMBER_HIGH_FIXED_CLAW,
		CHAMBER_HIGH_PLACE_FIXED_CLAW,
		SUSPEND,
		CHAMBER_HIGH,
		CHAMBER_HIGH_PLACE
	}
}