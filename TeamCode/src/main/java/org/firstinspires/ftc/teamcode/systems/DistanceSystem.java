package org.firstinspires.ftc.teamcode.systems;

import static org.firstinspires.ftc.teamcode.Utilities.RunSequentially;
import static org.firstinspires.ftc.teamcode.Utilities.setTimeout;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class DistanceSystem extends AbstractSystem
{
	private Rev2mDistanceSensor left, right;

	private float headingToleranceCM = 3;

	public DistanceSystem(Rev2mDistanceSensor left, Rev2mDistanceSensor right)
	{
		this.left = left;
		this.right = right;
	}

	@Override
	public void init()
	{
	}

	public boolean StatusOk()
	{
		return !(left.didTimeoutOccur() && left.didTimeoutOccur());
	}

	public double GetLeftDistanceCM()
	{
		return left.getDistance(DistanceUnit.CM);
	}

	public double GetRightDistanceCM()
	{
		return right.getDistance(DistanceUnit.CM);
	}

	public double GetAverageDistanceCM()
	{
		return (GetLeftDistanceCM() + GetRightDistanceCM()) / 2.0;
	}

	public Action ApproachAndPlace(RobotHardware robot, float targetDistCm, float releaseDistCm)
	{
		return RunSequentially(
				telemetryPacket -> {
					Vector2d linearVel = new Vector2d(-0.3, 0);
					double angularVel = 0;

					if (GetAverageDistanceCM() <= releaseDistCm)
					{
						robot.lift.setLiftLevel(LiftSystem.LiftLevel.CHAMBER_HIGH_PLACE_FIXED_CLAW);
					}

					if (Math.abs(GetLeftDistanceCM() - GetRightDistanceCM()) > headingToleranceCM && GetAverageDistanceCM() < 40)
					{
						angularVel = (GetLeftDistanceCM() - GetRightDistanceCM()) > 0 ? -0.3 : 0.3;
					}

					robot.drivetrain.setDrivePowers(new PoseVelocity2d(linearVel, angularVel));
					return GetAverageDistanceCM() > targetDistCm;
				},
				new InstantAction(() -> robot.drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0)))
		);
	}
}