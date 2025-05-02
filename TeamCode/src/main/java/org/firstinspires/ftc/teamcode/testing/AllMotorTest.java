package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.InputSystem;

import java.util.List;

@TeleOp(name = "All Motor Test", group = "Testing")
public final class AllMotorTest extends BaseOpMode
{
	private List<DcMotorEx> motors;
	private final InputSystem.Axis motorAxis = new InputSystem.Axis("left_stick_y");

	@Override
	protected void OnInitialize()
	{
		motors = hardwareMap.getAll(DcMotorEx.class);
		for (DcMotorEx motor : motors) {
			motor.setDirection(DcMotorEx.Direction.FORWARD);
			motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
			motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		}
	}

	@Override
	protected void jOnInitialize()
	{

	}

	@Override
	protected void OnRun()
	{
		for (DcMotorEx motor : motors) {
			motor.setPower(input1.getValue(motorAxis));
		}
	}
}
