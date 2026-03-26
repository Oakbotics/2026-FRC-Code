package frc.robot.util;

import java.util.Optional;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.util.HubTracker.Shift;
import frc.robot.vision.LimeLightSubsystem;
import frc.robot.vision.VisionConstants;

public class ElasticDashboard {

    private final CommandSwerveDrivetrain drivetrain;
    private final LimeLightSubsystem limelight;
    private final Field2d m_field = new Field2d();

    public ElasticDashboard(CommandSwerveDrivetrain drivetrain, LimeLightSubsystem limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;

        SmartDashboard.putData("Elastic/RobotField", m_field);
    }

    public void update() {
        //health
        SmartDashboard.putBoolean("Elastic/DSConnected", DriverStation.isDSAttached());
        SmartDashboard.putNumber("Elastic/BatteryVoltage", RobotController.getBatteryVoltage());
        SmartDashboard.putBoolean("Elastic/BrownedOut", RobotController.isBrownedOut());
        SmartDashboard.putNumber("Elastic/CANUtilization", RobotController.getCANStatus().percentBusUtilization * 100.0);
        SmartDashboard.putBoolean("Elastic/Enabled", DriverStation.isEnabled());

        //match
        Pose2d pose = drivetrain.getState().Pose;
        m_field.setRobotPose(pose); // Updates the 2D Map

        SmartDashboard.putNumber("Elastic/PoseX", pose.getX());
        SmartDashboard.putNumber("Elastic/PoseY", pose.getY());
        SmartDashboard.putNumber("Elastic/HeadingDeg", pose.getRotation().getDegrees());
        SmartDashboard.putString("Elastic/RobotMode", getCurrentMode());
        
        Optional<Alliance> alliance = DriverStation.getAlliance();
        String allianceStr = alliance.isPresent() ? alliance.get().name() : "UNKNOWN";
        SmartDashboard.putString("Elastic/Alliance", allianceStr);
        
        //direction
        double heading = pose.getRotation().getDegrees();
        String direction = (Math.abs(heading) < 90) ? "FACING RED" : "FACING BLUE";
        SmartDashboard.putString("Elastic/RobotDirection", direction);

        //vision
        SmartDashboard.putNumber("Elastic/DistToHub", pose.getTranslation().getDistance(VisionConstants.hubPosition()));
        int rightTagId = limelight.getRightID();
        SmartDashboard.putNumber("Elastic/LimelightTagID", rightTagId);
        SmartDashboard.putBoolean("Elastic/SeesHubTag", rightTagId != -1 && VisionConstants.HUB_TAG_IDS.contains(rightTagId));

        updateHubShiftInfo(alliance);
    }


    private void updateHubShiftInfo(Optional<Alliance> alliance) {
        Optional<Alliance> autoWinner = HubTracker.getAutoWinner();
        SmartDashboard.putString("Elastic/AutoWinner", autoWinner.isPresent() ? autoWinner.get().name() : "NONE");

        Optional<Shift> currentShift = HubTracker.getCurrentShift();
        SmartDashboard.putString("Elastic/CurrentShift", currentShift.isPresent() ? currentShift.get().name() : "---");

        boolean hubActiveForUs = alliance.isPresent() && HubTracker.isActive(alliance.get());
        SmartDashboard.putBoolean("Elastic/HubActiveForUs", hubActiveForUs);

        var timeRemaining = HubTracker.timeRemainingInCurrentShift();
        SmartDashboard.putNumber("Elastic/ShiftSecsLeft", timeRemaining.isPresent() ? timeRemaining.get().in(Units.Seconds) : -1);

        Optional<Shift> nextShift = HubTracker.getNextShift();
        SmartDashboard.putString("Elastic/NextShift", nextShift.isPresent() ? nextShift.get().name() : "---");

        boolean hubActiveNext = alliance.isPresent() && HubTracker.isActiveNext(alliance.get());
        SmartDashboard.putBoolean("Elastic/HubActiveNextShift", hubActiveNext);

        double matchTime = HubTracker.getMatchTime();
        if (nextShift.isPresent() && matchTime >= 0) {
            SmartDashboard.putNumber("Elastic/SecsUntilNextShift", nextShift.get().startTime - matchTime);
        } else {
            SmartDashboard.putNumber("Elastic/SecsUntilNextShift", -1);
        }

        SmartDashboard.putString("Elastic/HubStatus", buildHubStatusString(alliance, hubActiveForUs, timeRemaining, hubActiveNext));
    }

    private String buildHubStatusString(Optional<Alliance> alliance, boolean hubActiveForUs, Optional<edu.wpi.first.units.measure.Time> timeRemaining, boolean hubActiveNext) {
        if (alliance.isEmpty()) return "No alliance";
        double secsLeft = timeRemaining.isPresent() ? timeRemaining.get().in(Units.Seconds) : -1;

        if (hubActiveForUs) {
            if (secsLeft >= 0 && secsLeft <= 5) return "HUB OURS - ENDING IN " + String.format("%.0f", secsLeft) + "s!";
            return "HUB OURS" + (secsLeft >= 0 ? " (" + String.format("%.0f", secsLeft) + "s left)" : "");
        } else {
            if (hubActiveNext) {
                Optional<Shift> next = HubTracker.getNextShift();
                double matchTime = HubTracker.getMatchTime();
                if (next.isPresent() && matchTime >= 0) {
                    double secsUntil = next.get().startTime - matchTime;
                    return "WAIT - Ours in " + String.format("%.0f", secsUntil) + "s";
                }
            }
            return "NOT OURS";
        }
    }

    private String getCurrentMode() {
        if (DriverStation.isAutonomousEnabled()) return "AUTO";
        if (DriverStation.isTeleopEnabled()) return "TELEOP";
        if (DriverStation.isTestEnabled()) return "TEST";
        if (DriverStation.isDisabled()) return "DISABLED";
        return "UNKNOWN";
    }
}
