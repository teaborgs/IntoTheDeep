
package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.InputSystem;

@TeleOp(name = "One Motor Test", group = "Testing")
public final class OneMotorTest extends BaseOpMode
{
	private DcMotorEx motor;
	private final InputSystem.Axis powerAxis = new InputSystem.Axis("left_stick_y");
	@Override
	protected void OnInitialize()
	{
		motor = hardwareMap.get(DcMotorEx.class, "motor");
		motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}

	@Override
	protected void jOnInitialize()
	{

	}

	@Override
	protected void OnRun()
	{
		motor.setPower(input1.getValue(powerAxis));
	}

	@Override
	protected void OnTelemetry(Telemetry telemetry)
	{
		telemetry.addData("[INFO] Power", input1.getValue(powerAxis));
		telemetry.addData("[INFO] Motor Current", motor.getCurrent(CurrentUnit.AMPS));
	}
}