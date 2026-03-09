package frc.robot.led;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.units.Units;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.HubTracker;

public class LEDSubsystem extends SubsystemBase {

    private final CANdle candle;
    private final Supplier<Pose2d> poseSupplier;
    private final XboxController controller;

    private static final double HUB_X = 8.27;
    private static final double HUB_Y = 4.10;

    public static final double SHOOT_LINE_DISTANCE = 4.0;

    private static final int LED_COUNT = 120;

    private boolean blinkState = false;
    private double lastBlinkTime = 0;

    private boolean wasInsideRange = false;
    private double rumbleEndTime = 0;

    public LEDSubsystem(Supplier<Pose2d> poseSupplier, XboxController controller) {
        candle = new CANdle(0);
        this.poseSupplier = poseSupplier;
        this.controller = controller;
    }

    @Override
    public void periodic() {

        Optional<Alliance> allianceOptional = DriverStation.getAlliance();

        if (DriverStation.getMatchTime() < 0) {
            var rainbow = new RainbowAnimation(0, 0)
                .withBrightness(0.7) 
                .withFrameRate(30);    

            candle.setControl(rainbow);            
            setRumble(0);
            return;
        }

        if (allianceOptional.isEmpty()) {
            setLED(0,0,0);
            setRumble(0);
            return;
        }

        Alliance alliance = allianceOptional.get();

        if (!HubTracker.isActive(alliance)) {
            setLED(0,0,0);
            setRumble(0);
            return;
        }

        int r = 0;
        int g = 0;
        int b = 0;

        if (alliance == Alliance.Red) {
            r = 255;
        } else {
            b = 255;
        }

        var timeRemainingOptional = HubTracker.timeRemainingInCurrentShift();

        if (timeRemainingOptional.isPresent() &&
            timeRemainingOptional.get().in(Units.Seconds) <= 5) {

            if (Timer.getFPGATimestamp() - lastBlinkTime > 0.08) {
                blinkState = !blinkState;
                lastBlinkTime = Timer.getFPGATimestamp();
            }

            if (blinkState) {
                setLED(r,g,b);
            } else {
                setLED(0,0,0);
            }

            setRumble(0);
            return;
        }

        Pose2d robotPose = poseSupplier.get();

        double dx = robotPose.getX() - HUB_X;
        double dy = robotPose.getY() - HUB_Y;

        double distance = Math.hypot(dx, dy);

        boolean insideShootRange = distance <= SHOOT_LINE_DISTANCE;

        if (insideShootRange && !wasInsideRange) {
            rumbleEndTime = Timer.getFPGATimestamp() + 0.25;
        }

        wasInsideRange = insideShootRange;

        if (Timer.getFPGATimestamp() < rumbleEndTime) {
            setRumble(1);
        } else {
            setRumble(0);
        }

        setLED(r,g,b);
    }

    private void setLED(int r, int g, int b) {
        candle.setControl(
            new SolidColor(0, LED_COUNT)
                .withColor(new RGBWColor(r,g,b))
        );
    }

    private void setRumble(double strength) {
        controller.setRumble(GenericHID.RumbleType.kBothRumble, strength);
    }
}