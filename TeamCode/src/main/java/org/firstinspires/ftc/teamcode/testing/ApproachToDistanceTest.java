package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.Utilities.RunSequentially;
import static org.firstinspires.ftc.teamcode.Utilities.WaitFor;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.systems.LiftSystem;

@Autonomous(name = "Approach to distance test", group = "Testing")
public class ApproachToDistanceTest extends BaseOpMode
{
	private RobotHardware robot;

	@Override
	protected void OnInitialize()
	{
		robot = new RobotHardware(hardwareMap);
		robot.init();
	}

	@Override
	protected void jOnInitialize()
	{

	}

	@Override
	protected void OnRun()
	{
		Actions.runBlocking(RunSequentially(
				WaitFor(1),
				robot.lift.moveLiftTo(LiftSystem.LiftLevel.CHAMBER_HIGH_FIXED_CLAW),
				robot.distanceSystem.ApproachAndPlace(robot, 11, 13),
				WaitFor(30)
		));
	}
}