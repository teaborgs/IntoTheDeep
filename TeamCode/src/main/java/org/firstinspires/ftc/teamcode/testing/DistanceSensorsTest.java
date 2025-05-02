package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "Dist Sensors Test", group = "Testing")
public class DistanceSensorsTest extends BaseOpMode
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
	}

	@Override
	protected void OnTelemetry(Telemetry telemetry)
	{
		telemetry.setMsTransmissionInterval(500);
		telemetry.addData("Left", robot.distanceSystem.GetLeftDistanceCM());
		telemetry.addData("Right", robot.distanceSystem.GetRightDistanceCM());
		telemetry.addData("Average", robot.distanceSystem.GetAverageDistanceCM());
	}
}