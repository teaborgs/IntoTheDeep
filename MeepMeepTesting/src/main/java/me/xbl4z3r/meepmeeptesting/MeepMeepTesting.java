package me.xbl4z3r.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting
{
	public static void main(String[] args)
	{
		MeepMeep meepMeep = new MeepMeep(800);

		RoadRunnerBotEntity robot = new DefaultBotBuilder(meepMeep)
				// Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
				.setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
				.followTrajectorySequence(drive -> {
					// Create a trajectory for the bot to follow
					return drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
							.setTangent(DegToRad(180))
							.splineToConstantHeading(centimetersToInches(new Vector2d(-68, 5)), DegToRad(180))
							.lineTo(centimetersToInches(new Vector2d(-71, 5)))
							.setTangent(DegToRad(45))
							.splineToConstantHeading(centimetersToInches(new Vector2d(-50, 70)), DegToRad(180))
							.setTangent(DegToRad(180))
							.splineToConstantHeading(centimetersToInches(new Vector2d(-115, 95)), DegToRad(45))
							.setTangent(0)
							.lineTo(centimetersToInches(new Vector2d(-25, 95)))
							.setTangent(DegToRad(180))
							.splineToConstantHeading(centimetersToInches(new Vector2d(-115, 125)), DegToRad(45))
							.setTangent(0)
							.lineTo(centimetersToInches(new Vector2d(-25, 125)))
							.setTangent(DegToRad(180))
							.splineToConstantHeading(centimetersToInches(new Vector2d(-115, 145)), DegToRad(45))
							.setTangent(0)
							.lineTo(centimetersToInches(new Vector2d(-25, 145)))
							.setTangent(DegToRad(90))
							.lineTo(centimetersToInches(new Vector2d(-25, 130)))
							.setTangent(DegToRad(90))
							.splineToLinearHeading(centimetersToInches(new Pose2d(-5, 80, DegToRad(180))), DegToRad(0))

							.build();
				});

		meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
				.setDarkMode(true)
				.addEntity(robot)
				.start();
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
		return new Pose2d(centimetersToInches(pose.vec()), pose.getHeading());
	}

	public static double DegToRad(double deg)
	{
		return Math.toRadians(deg);
	}
}
