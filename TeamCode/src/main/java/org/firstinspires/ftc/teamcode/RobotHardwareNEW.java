package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.systems.DistanceSystem;
import org.firstinspires.ftc.teamcode.systems.ExtendoServoSystem;
import org.firstinspires.ftc.teamcode.systems.LiftSystem;
import org.firstinspires.ftc.teamcode.systems.OpenCloseSystem;
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;
import org.firstinspires.ftc.teamcode.systems.ExtendoMotorSystem;
import org.firstinspires.ftc.teamcode.systems.IntakeSystem;

public final class RobotHardwareNEW {
	public final MecanumDrive drivetrain;
	public TumblerSystem intakeTumbler, scoreTumbler;
	public ExtendoMotorSystem extendo;
	public ExtendoServoSystem scoreExtendo;
	public LiftSystem lift;
	public OpenCloseSystem scoreClaw, axle;
	public DistanceSystem distanceSystem;
	public IntakeSystem intake;

	public RobotHardwareNEW(HardwareMap hardwareMap) {
		drivetrain = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

		intake = new IntakeSystem(hardwareMap.get(DcMotorEx.class, "intake"));
		extendo = new ExtendoMotorSystem(hardwareMap.get(DcMotorEx.class, "extendo"));

		// Uncomment and edit these as needed
		/*
		intakeTumbler = new TumblerSystem(hardwareMap.get(Servo.class, "intakeTumbler"), 0.14f, 0.14f, 0.14f, 0.14f, 0.23f);
		scoreTumbler = new TumblerSystem(hardwareMap.get(Servo.class, "scoreTumbler"), 0.99f, 0.3f, 0.5f, 0.32f, 0.41f);
		scoreClaw = new OpenCloseSystem(hardwareMap.get(Servo.class, "scoreClaw"), 0.61f, 0.33f);
		lift = new LiftSystem(hardwareMap.get(DcMotorEx.class, "lift1"), hardwareMap.get(DcMotorEx.class, "lift2"));
		scoreTumbler = new TumblerSystem(hardwareMap.get(Servo.class, "scoreStumbler"), 0.43f, 0.43f, 0.5f, 0.94f, 0.77f);
		axle = new OpenCloseSystem(hardwareMap.get(Servo.class, "axleServo1"), 0.5f, 0.39f);
		distanceSystem = new DistanceSystem(hardwareMap.get(Rev2mDistanceSensor.class, "leftDist"),
											hardwareMap.get(Rev2mDistanceSensor.class, "rightDist"));
		*/
	}

	public void init() {
		// ✅ Initialize intake system
		intake.init();
		extendo.init();

		// Uncomment and edit these as needed
		/*
		intakeTumbler.init();
		scoreTumbler.setDestination(TumblerSystem.TumblerDestination.TRANSFER);
		scoreClaw.init();
		lift.init();
		scoreTumbler.init();
		scoreClaw.init();
		scoreTumbler.setDestination(TumblerSystem.TumblerDestination.TRANSFER);
		scoreClaw.close();
		axle.init();
		*/
	}
}
