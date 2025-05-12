package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.Utilities.RunInParallel;
import static org.firstinspires.ftc.teamcode.Utilities.RunSequentially;
import static org.firstinspires.ftc.teamcode.Utilities.WaitFor;
import static org.firstinspires.ftc.teamcode.Utilities.setTimeout;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.InputSystem;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardwareNEW;
import org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.teamcode.systems.ExtendoServoSystem;
import org.firstinspires.ftc.teamcode.systems.ExtendoMotorSystem;
import org.firstinspires.ftc.teamcode.systems.ExtendoSystem;
import org.firstinspires.ftc.teamcode.systems.IntakeSystem.IntakeDirection;
import org.firstinspires.ftc.teamcode.systems.LiftSystem;
import org.firstinspires.ftc.teamcode.systems.RotatorSystem;
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;
import org.firstinspires.ftc.teamcode.systems.OpenCloseSystem;
import org.firstinspires.ftc.teamcode.systems.IntakeSystem;
import org.w3c.dom.Attr;

import java.util.HashMap;

@TeleOp(name = "\uD83D\uDC79Fane Boss TBD\uD83D\uDC79 Op", group = "TeleOp")
public final class tbdOp extends LinearOpMode {
	private RobotHardwareNEW robot;
	private InputSystem driveInput;
	private InputSystem armInput;
	private Utilities.State suspendState = Utilities.State.IDLE;
	private TumblerSystem intakeTumbler;

	private static class Keybindings {
		public static class Arm {
			public static final InputSystem.Key BASKET_LOW_KEY = new InputSystem.Key("dpad_down");
			public static final InputSystem.Key BASKET_HIGH_KEY = new InputSystem.Key("dpad_up");
			public static final InputSystem.Key CHAMBER_HIGH_KEY = new InputSystem.Key("dpad_left");
			public static final InputSystem.Key PRIMARY_KEY = new InputSystem.Key("a");
			public static final InputSystem.Key SPECIMEN_KEY = new InputSystem.Key("b");
			public static final InputSystem.Key SUSPEND_KEY = new InputSystem.Key("y");
			public static final InputSystem.Key CANCEL_SUSPEND_KEY = new InputSystem.Key("x");
		}

		public static class Drive {
			public static final InputSystem.Key SUPPRESS_KEY = new InputSystem.Key("left_bumper");
			public static final InputSystem.Axis DRIVE_X = new InputSystem.Axis("left_stick_x");
			public static final InputSystem.Axis DRIVE_Y = new InputSystem.Axis("left_stick_y");
			public static final InputSystem.Axis DRIVE_ROT_L = new InputSystem.Axis("left_trigger");
			public static final InputSystem.Axis DRIVE_ROT_R = new InputSystem.Axis("right_trigger");
			public static final	InputSystem.Key INTAKE_FORWARD = new InputSystem.Key("right_bumper");
			public static final InputSystem.Key INTAKE_REVERSE = new InputSystem.Key("b");
			public static final InputSystem.Key EXTENDO_FULL_KEY = new InputSystem.Key("dpad_up");
			public static final InputSystem.Key EXTENDO_HALF_KEY = new InputSystem.Key("dpad_left");
			public static final InputSystem.Key EXTENDO_ZERO_KEY = new InputSystem.Key("dpad_down");
			public static final InputSystem.Key GRAB_TRANSFER_KEY = new InputSystem.Key("a");
			public static final InputSystem.Key GRAB_WALL_KEY = new InputSystem.Key("x");
			public static final InputSystem.Key GRAB_RESET_KEY = new InputSystem.Key("y");
		}
	}

	@Override
	public void runOpMode() throws InterruptedException {
		OnInitialize();

		waitForStart();
		if (isStopRequested()) return;
		OnStart();

		while (opModeIsActive()) {
			OnRun();


		}
	}

	protected void OnInitialize() {
		driveInput = new InputSystem(gamepad1);
		armInput = new InputSystem(gamepad2);
	}

	protected void OnStart() {
		robot = new RobotHardwareNEW(hardwareMap);
		robot.init();
		intakeTumbler = robot.intakeTumbler;
	}

	protected void OnRun() {
		//robot.lift.autoPower();
		Suspender();
		Drive();
		IntakeControl();
		Extendo();
		Scorer();
		if (suspendState == Utilities.State.BUSY) return;
	}


	private void Drive()
	{
		float speed = 1f;
		 if (driveInput.isPressed(tbdOp.Keybindings.Drive.SUPPRESS_KEY)) speed = 0.4f;
		robot.drivetrain.setDrivePowers(
				new PoseVelocity2d(new Vector2d(driveInput.getValue(tbdOp.Keybindings.Drive.DRIVE_Y),
						driveInput.getValue(tbdOp.Keybindings.Drive.DRIVE_X)).times(-speed * (liftedSpecimen ? -1 : 1)),
						(driveInput.getValue(tbdOp.Keybindings.Drive.DRIVE_ROT_L) - driveInput.getValue(tbdOp.Keybindings.Drive.DRIVE_ROT_R)) * speed
				)
		);
	}
	private Utilities.State extendoState = Utilities.State.IDLE;
	private Utilities.State scorerState = Utilities.State.IDLE;
	private boolean waitingToDrop = false;
	private boolean grabbedWall = false;
	private boolean liftedSpecimen = false;

	private ExtendoMotorSystem.ExtendoLevel extendoLevel = ExtendoMotorSystem.ExtendoLevel.RETRACTED;
	private LiftSystem.LiftLevel liftLevel = LiftSystem.LiftLevel.BASKET_HIGH;
	public IntakeDirection intakeDirection = IntakeDirection.STOP;
	private boolean liftOverride = false;
	/*
	private ExtendoSystem.ExtendoLevel extendoLevel = ExtendoSystem.ExtendoLevel.RETRACTED;
	private LiftSystem.LiftLevel liftLevel = LiftSystem.LiftLevel.BASKET_HIGH;
	private boolean liftOverride = false;
	private void Extendo()
	{
	/*	if (driveInput.wasPressedThisFrame(tbdOp.Keybindings.Drive.EXTENDO_FULL_KEY))
			extendoLevel = ExtendoSystem.ExtendoLevel.EXTENDED;
		else if (driveInput.wasPressedThisFrame(tbdOp.Keybindings.Drive.EXTENDO_HALF_KEY))
			extendoLevel = ExtendoSystem.ExtendoLevel.HALF;
		else if (driveInput.wasPressedThisFrame(tbdOp.Keybindings.Drive.EXTENDO_ZERO_KEY))
			extendoLevel = ExtendoSystem.ExtendoLevel.RETRACTED;

		if (robot.extendo.getExtendoLevel() != extendoLevel)
			robot.extendo.extend(extendoLevel); */

/*
	}


 */

	private void Suspender()
	{

	}
	private void Scorer() {

			if (armInput.wasPressedThisFrame(Keybindings.Arm.BASKET_LOW_KEY)) {
				liftLevel = LiftSystem.LiftLevel.BASKET_LOW;
			} else if (armInput.wasPressedThisFrame(Keybindings.Arm.BASKET_HIGH_KEY)) {
				liftLevel = LiftSystem.LiftLevel.BASKET_HIGH;
			} else if (armInput.wasPressedThisFrame(Keybindings.Arm.CHAMBER_HIGH_KEY)) {
				liftLevel = LiftSystem.LiftLevel.CHAMBER_HIGH;
			}

			robot.lift.setLiftLevel(liftLevel);

	}




	private void Extendo()
	{
		if (driveInput.wasPressedThisFrame(tbdOp.Keybindings.Drive.EXTENDO_FULL_KEY))
			extendoLevel = ExtendoMotorSystem.ExtendoLevel.EXTENDED;
		else if (driveInput.wasPressedThisFrame(tbdOp.Keybindings.Drive.EXTENDO_HALF_KEY))
			extendoLevel = ExtendoMotorSystem.ExtendoLevel.HALF;
		else if (driveInput.wasPressedThisFrame(tbdOp.Keybindings.Drive.EXTENDO_ZERO_KEY))
			extendoLevel = ExtendoMotorSystem.ExtendoLevel.RETRACTED;

		if (robot.extendo.getExtendoLevel() != extendoLevel)
			robot.extendo.extend(extendoLevel);
	}

	private void IntakeControl() {
		if(driveInput.isPressed(tbdOp.Keybindings.Drive.INTAKE_FORWARD))
		{
			intakeTumbler.setDestination(TumblerSystem.TumblerDestination.BUSY);
			intakeDirection = IntakeDirection.FORWARD;
		}
		else if(driveInput.isPressed(Keybindings.Drive.INTAKE_REVERSE))
		{
			intakeDirection = IntakeDirection.REVERSE;
			intakeTumbler.setDestination(TumblerSystem.TumblerDestination.HOVER);
		}
		else
			intakeDirection = IntakeDirection.STOP;
		robot.intake.setIntakeDirection(intakeDirection);
	}




}


