package frc.robot.led;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.util.HubTracker;
import frc.robot.vision.VisionConstants;

public class LEDSubsystem extends SubsystemBase {

    private final CANdle candle;
    private final Supplier<Pose2d> poseSupplier;

    private static final double HUB_X = VisionConstants.hubPosition().getX();
    private static final double HUB_Y = VisionConstants.hubPosition().getY();

    public static final double SHOOT_LINE_DISTANCE = 4.0;

    private static final int LED_COUNT = 120;

    private boolean blinkState = false;
    private double lastBlinkTime = 0;

    public LEDSubsystem(Supplier<Pose2d> poseSupplier) {
        candle = new CANdle(0);
        this.poseSupplier = poseSupplier;
    }

    @Override
    public void periodic() {

        Optional<Alliance> allianceOptional = DriverStation.getAlliance();

        if (allianceOptional.isEmpty()) {
            setLED(0,0,0);
            return;
        }

        Alliance alliance = allianceOptional.get();

        if (!HubTracker.isActive(alliance)) {
            setLED(0,0,0);
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

        Pose2d robotPose = poseSupplier.get();

        double dx = robotPose.getX() - HUB_X;
        double dy = robotPose.getY() - HUB_Y;

        double distance = Math.hypot(dx, dy);

        boolean insideShootRange = distance <= SHOOT_LINE_DISTANCE;

        if (insideShootRange) {

            if (Timer.getFPGATimestamp() - lastBlinkTime > 0.25) {
                blinkState = !blinkState;
                lastBlinkTime = Timer.getFPGATimestamp();
            }

            if (blinkState) {
                setLED(r,g,b);
            } else {
                setLED(0,0,0);
            }

        } else {

            setLED(r,g,b);

        }
    }

    private void setLED(int r, int g, int b) {

        candle.setControl(
            new SolidColor(0, LED_COUNT)
                .withColor(new RGBWColor(r, g, b))
        );

    }
}