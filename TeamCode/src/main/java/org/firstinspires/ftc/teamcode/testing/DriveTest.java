package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.InputSystem;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@TeleOp(name = "Drive Test", group = "Testing")
public final class DriveTest extends BaseOpMode
{
	private MecanumDrive mecanumDrive;

	public static final InputSystem.Key TURBO_KEY = new InputSystem.Key("right_bumper");
	public static final InputSystem.Key SUPPRESS_KEY = new InputSystem.Key("left_bumper");
	public static final InputSystem.Axis DRIVE_X = new InputSystem.Axis("left_stick_x");
	public static final InputSystem.Axis DRIVE_Y = new InputSystem.Axis("left_stick_y");
	public static final InputSystem.Axis DRIVE_ROT_L = new InputSystem.Axis("left_trigger");
	public static final InputSystem.Axis DRIVE_ROT_R = new InputSystem.Axis("right_trigger");

	@Override
	protected void OnInitialize()
	{
		mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
	}

	@Override
	protected void jOnInitialize()
	{

	}

	@Override
	protected void OnRun()
	{
		float speed = 0.7f;
		if (input1.isPressed(TURBO_KEY)) speed = 1.0f;
		else if (input1.isPressed(SUPPRESS_KEY)) speed = 0.4f;
		mecanumDrive.setDrivePowers(
				new PoseVelocity2d(new Vector2d(input1.getValue(DRIVE_Y),
						input1.getValue(DRIVE_X)).times(speed),
						input1.getValue(DRIVE_ROT_L) + input1.getValue(DRIVE_ROT_R)
				)
		);
	}
}