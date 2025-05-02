package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public final class Utilities
{
	public static void setTimeout(Runnable runnable, long delay)
	{
		new Thread(() -> {
			try {
				Thread.sleep(delay);
				runnable.run();
			}
			catch (Exception e) {
				e.printStackTrace();
			}
		}).start();
	}

	public static double centimetersToInches(double centimeters)
	{
		return centimeters / 2.54;
	}

	public static Vector2d centimetersToInches(Vector2d pose)
	{
		return pose.times(1 / 2.54);
	}

	public static Pose2d centimetersToInches(Pose2d pose)
	{
		return new Pose2d(centimetersToInches(pose.position), pose.heading);
	}

	public static double inchesToCentimeters(double inches)
	{
		return inches * 2.54;
	}

	public static Vector2d inchesToCentimeters(Vector2d pose)
	{
		return pose.times(2.54);
	}

	public static Pose2d inchesToCentimeters(Pose2d pose)
	{
		return new Pose2d(inchesToCentimeters(pose.position), pose.heading);
	}

	public static double DegToRad(double deg)
	{
		return Math.toRadians(deg);
	}

	/**
	 * Convert an angle to a servo position
	 *
	 * @param angle    The angle to convert
	 * @param maxAngle The angle that corresponds to the maximum servo position relative to the starting position (in degrees, usually 180)
	 * @return The servo position
	 */
	public static double angleToServoPosition(double angle, double maxAngle)
	{
		return angle / maxAngle;
	}

	/**
	 * Convert an angle to a servo position (assuming the maximum angle is 180 degrees)
	 *
	 * @param angle The angle to convert
	 * @return The servo position
	 */
	public static double angleToServoPosition(double angle)
	{
		return angleToServoPosition(angle, 180);
	}

	public static Action WaitFor(double delaySec)
	{
		return new SleepAction(delaySec);
	}

	public static Action RunInParallel(Action... actions)
	{
		return new ParallelAction(actions);
	}

	public static Action RunSequentially(Action... actions)
	{
		return new SequentialAction(actions);
	}

	public static String toDisplayString(String input)
	{
		StringBuilder displayString = new StringBuilder();
		for (char c : input.toCharArray()) {
			if (Character.isUpperCase(c) && displayString.length() > 0)
				displayString.append(' ');
			displayString.append(c);
		}
		if (displayString.length() > 0)
			displayString.setCharAt(0, Character.toUpperCase(displayString.charAt(0)));
		return displayString.toString();
	}

	public static void CutPower(DcMotorEx... devices)
	{
		for (DcMotorEx device : devices) {
			device.setPower(0);
			device.setMotorDisable();
		}
	}

	public static void RestorePower(DcMotorEx... devices)
	{
		for (DcMotorEx device : devices) device.setMotorEnable();
	}

	public static void CutPower(Servo... devices)
	{
		for (Servo device : devices) device.getController().pwmDisable();
	}

	public static void RestorePower(Servo... devices)
	{
		for (Servo device : devices) device.getController().pwmEnable();
	}

	public enum State
	{
		IDLE,
		BUSY
	}

	public enum Alliance
	{
		BLUE,
		RED
	}
}