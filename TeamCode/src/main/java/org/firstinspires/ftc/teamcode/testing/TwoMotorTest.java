package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.InputSystem;

@TeleOp(name = "Two Motor Test", group = "Testing")
public final class TwoMotorTest extends BaseOpMode
{
	private DcMotorEx motor1, motor2;
	private final InputSystem.Axis powerAxis = new InputSystem.Axis("left_stick_y");
	@Override
	protected void OnInitialize()
	{
		motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
		motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
		motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
	}

	@Override
	protected void jOnInitialize()
	{

	}

	@Override
	protected void OnRun()
	{
		motor1.setPower(input1.getValue(powerAxis));
		motor2.setPower(input1.getValue(powerAxis));
	}

	@Override
	protected void OnTelemetry(Telemetry telemetry)
	{
		telemetry.addData("[INFO] Power", input1.getValue(powerAxis));
		telemetry.addData("[INFO] Motor 1 Current", motor1.getCurrent(CurrentUnit.AMPS));
		telemetry.addData("[INFO] Motor 2 Current", motor2.getCurrent(CurrentUnit.AMPS));
		telemetry.addData("[INFO] Motor 1 Pos", motor1.getCurrentPosition());
		telemetry.addData("[INFO] Motor 2 Pos", motor2.getCurrentPosition());
	}
}