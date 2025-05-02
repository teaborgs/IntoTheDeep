package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utilities.CutPower;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.systems.AbstractSystem;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

/**
 * Base class for all OpModes. This class provides a simple way to create OpModes with a consistent
 * structure. It also provides a way to use the FTC Dashboard for telemetry.
 *
 * @author Christi
 */
public abstract class BaseOpMode extends LinearOpMode
{
	private volatile boolean internal_DebugMode = false;
	private volatile boolean internal_DummyMode = false;

	protected InputSystem input1, input2;
	private final InputSystem.Key debugKey = new InputSystem.Key("left_stick_button");
	private final InputSystem.Key dummyKey = new InputSystem.Key("back");

	private ElapsedTime internal_ElapsedTimeSinceInit;
	private ElapsedTime internal_ElapsedTimeSinceStart;
	private final ElapsedTime internal_LoopTime = new ElapsedTime();

	@Override
	public void runOpMode()
	{
		internal_ElapsedTimeSinceInit = new ElapsedTime();

		boolean internal_IsAutonomous = this.getClass().isAnnotationPresent(Autonomous.class);
		this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		telemetry.setMsTransmissionInterval(50);
		telemetry.addLine("[STATUS] Initializing...");
		telemetry.update();

		input1 = new InputSystem(gamepad1, telemetry);
		input2 = new InputSystem(gamepad2, telemetry);

		// Initialize the robot
		OnInitialize();

		// Wait for the start button to be pressed
		while (!isStarted() && !isStopRequested()) {
			if (IsDebugRequested()) {
				internal_DebugMode = !internal_DebugMode;
				telemetry.addLine("[DEBUG] Debug mode is enabled");
				telemetry.update();
			}
			ListenForDummyMode();
			WhileWaitingForStart();
		}

		// Run the robot
		internal_ElapsedTimeSinceStart = new ElapsedTime();
		OnStart();
		Thread telemetryThread = new Thread(() -> {
			while (!isStopRequested()) {
				internal_LoopTime.reset();
				if (IsDebugRequested()) internal_DebugMode = !internal_DebugMode;
				OnTelemetry(telemetry);
				if (internal_DebugMode) OnDebug();
				telemetry.update();
			}
		});
		telemetryThread.start();
		while (!isStopRequested()) {
			OnRun();
			if (internal_IsAutonomous) break;
		}
		OnStop();

		// Stop the robot
		telemetry.clear();
		telemetry.addLine("[STATUS] Stopping...");
		telemetry.update();
	}


	protected abstract void jOnInitialize();

	protected abstract void OnRun();

	protected void WhileWaitingForStart()
	{
		// Override this method to add custom behavior while waiting for start
	}

	protected abstract void OnInitialize();

	/**
	 * This method is called when the OpMode is started.
	 * It is called before the loop starts.
	 * Override this method to add custom start behavior
	 */
	protected void OnStart()
	{
	}

	/**
	 * This method is called when the OpMode is stopped.
	 * It is called after the loop has stopped.
	 * Override this method to add custom stop behavior
	 */
	protected void OnStop()
	{
	}

	/**
	 * Override this method to add telemetry data
	 */
	protected void OnTelemetry(Telemetry telemetry)
	{
		telemetry.addLine("[SYSTEM] Op Mode running...");
		telemetry.addData("[SYSTEM] Loop Time", internal_LoopTime.milliseconds());
	}

	/**
	 * Override this method to add custom debug behavior
	 */
	protected void OnDebug()
	{
		telemetry.addLine("[DEBUG] Debug mode is enabled");
		telemetry.addData("[DEBUG] Dummy Mode", internal_DummyMode);
		Class<?> clazz = this.getClass();
		ArrayList<Field> fields = new ArrayList<>(Arrays.asList(clazz.getDeclaredFields()));
		HashMap<Field, Object> fieldParents = new HashMap<>();
		fields.forEach(field -> fieldParents.put(field, this));

		// check if any of the fields is a RobotHardware object and extract all known custom classes
		for (Field field : fields) {
			field.setAccessible(true);
			try {
				Object value = field.get(this);
				if (value instanceof RobotHardware) {
					for (Field declaredField : value.getClass().getDeclaredFields()) {
						declaredField.setAccessible(true);
						try {
							Object declaredValue = declaredField.get(value);
							if (declaredValue instanceof AbstractSystem) {
								for (Field systemField : declaredValue.getClass().getDeclaredFields()) {
									if (systemField.getType().isPrimitive()) continue;
									fields.add(systemField);
									fieldParents.put(systemField, declaredValue);
								}
							} else {
								fields.add(declaredField);
								fieldParents.put(declaredField, value);
							}
						} catch (Exception e) {
							telemetry.addData("[!] " + declaredField.getName(), "Error: " + e.getMessage());
							e.printStackTrace();
						}
					}
					break;
				}
			} catch (Exception e) {
				telemetry.addData("[!] " + field.getName(), "Error: " + e.getMessage());
				e.printStackTrace();
			}
		}

		for (Field field : fields) {
			field.setAccessible(true);
			try {
				Object value = field.get(fieldParents.get(field));
				if (value instanceof DcMotorEx) {
					DcMotorEx motor = (DcMotorEx) value;
					telemetry.addData("[DEBUG] " + Utilities.toDisplayString(field.getName()) + " Power", motor.getPower());
					telemetry.addData("[DEBUG] " + Utilities.toDisplayString(field.getName()) + " Current", motor.getCurrent(CurrentUnit.MILLIAMPS));
					telemetry.addData("[DEBUG] " + Utilities.toDisplayString(field.getName()) + " Velocity", motor.getVelocity());
					telemetry.addData("[DEBUG] " + Utilities.toDisplayString(field.getName()) + " Position", motor.getCurrentPosition());
					telemetry.addData("[DEBUG] " + Utilities.toDisplayString(field.getName()) + " Target Position", motor.getTargetPosition());
					telemetry.addData("[DEBUG] " + Utilities.toDisplayString(field.getName()) + " Busy", motor.isBusy());
				} else if (value instanceof Servo) {
					Servo servo = (Servo) value;
					telemetry.addData("[DEBUG] " + Utilities.toDisplayString(field.getName()) + " Position", servo.getPosition());
				} else if (value instanceof Enum) {
					telemetry.addData("[DEBUG] " + Utilities.toDisplayString(field.getName()), ((Enum<?>) value).name());
				} else if (value instanceof Rev2mDistanceSensor) {
					telemetry.addData("[DEBUG] " + Utilities.toDisplayString(field.getName()) + " Distance", ((Rev2mDistanceSensor) value).getDistance(DistanceUnit.CM));
				} else {
					telemetry.addData("[DEBUG] " + Utilities.toDisplayString(field.getName()), value == null ? "NULL" : value.toString());
				}
			} catch (Exception e) {
				telemetry.addData("[!] " + Utilities.toDisplayString(field.getName()), "Error: " + e.getMessage());
				e.printStackTrace();
			}
		}
		telemetry.addData("[DEBUG] Elapsed Time Since Init", getElapsedTimeSinceInit());
		telemetry.addData("[DEBUG] Elapsed Time Since Start", getElapsedTimeSinceStart());
		telemetry.addData("[DEBUG] Loop Time", internal_LoopTime.milliseconds());
		telemetry.update();
	}

	private boolean IsDebugRequested()
	{
		return input1.wasPressedThisFrame(debugKey) || input2.wasPressedThisFrame(debugKey);
	}

	private void ListenForDummyMode()
	{
		if (internal_DummyMode) return;
		if (!internal_DebugMode) return;
		if (input1.wasPressedThisFrame(dummyKey) || input2.wasPressedThisFrame(dummyKey)) {
			internal_DummyMode = true;

			Class<?> clazz = this.getClass();
			ArrayList<Field> fields = new ArrayList<>(Arrays.asList(clazz.getDeclaredFields()));
			HashMap<Field, Object> fieldParents = new HashMap<>();
			fields.forEach(field -> fieldParents.put(field, this));

			// check if any of the fields is a RobotHardware object and extract all known custom classes
			for (Field field : fields) {
				field.setAccessible(true);
				try {
					Object value = field.get(this);
					if (value instanceof RobotHardware) {
						for (Field declaredField : value.getClass().getDeclaredFields()) {
							declaredField.setAccessible(true);
							try {
								Object declaredValue = declaredField.get(value);
								if (declaredValue instanceof AbstractSystem) {
									for (Field systemField : declaredValue.getClass().getDeclaredFields()) {
										if (systemField.getType().isPrimitive()) continue;
										fields.add(systemField);
										fieldParents.put(systemField, declaredValue);
									}
								} else {
									fields.add(declaredField);
									fieldParents.put(declaredField, value);
								}
							} catch (Exception e) {
								telemetry.addData("[!] " + declaredField.getName(), "Error: " + e.getMessage());
								e.printStackTrace();
							}
						}
						break;
					}
				} catch (Exception e) {
					telemetry.addData("[!] " + field.getName(), "Error: " + e.getMessage());
					e.printStackTrace();
				}
			}

			for (Field field : fields) {
				field.setAccessible(true);
				try {
					Object value = field.get(fieldParents.get(field));
					if (value instanceof DcMotorEx) {
						DcMotorEx motor = (DcMotorEx) value;
						motor.setPower(0);
						motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
					} else if (value instanceof Servo) {
						CutPower((Servo) value);
					}
				} catch (Exception e) {
					telemetry.addData("[!] " + field.getName(), "Error: " + e.getMessage());
				}
			}

			telemetry.addLine("[DEBUG] Dummy mode is enabled");
			telemetry.update();
		}
	}

	protected double getElapsedTimeSinceInit()
	{
		if (internal_ElapsedTimeSinceInit == null) return -1;
		return internal_ElapsedTimeSinceInit.seconds();
	}

	protected double getElapsedTimeSinceStart()
	{
		if (internal_ElapsedTimeSinceStart == null) return -1;
		return internal_ElapsedTimeSinceStart.seconds();
	}

	protected ElapsedTime getLoopTime()
	{
		return internal_LoopTime;
	}
}