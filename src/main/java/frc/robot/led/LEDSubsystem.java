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
import frc.robot.vision.VisionConstants;
import frc.robot.shooter.ShootOnMoveConstants;

public class LEDSubsystem extends SubsystemBase {

    private final CANdle candle;
    private final Supplier<Pose2d> poseSupplier;
    private final XboxController controller;

    

    public static final double SHOOT_LINE_DISTANCE = 4.0;

    private static final int LED_COUNT = 120;

    private final SolidColor solidColorRequest = new SolidColor(0, LED_COUNT);
    private final RainbowAnimation rainbowRequest = new RainbowAnimation(0, 0)
        .withBrightness(0.7)
        .withFrameRate(30);

    private static final RGBWColor COLOR_RED = new RGBWColor(255, 0, 0);
    private static final RGBWColor COLOR_BLUE = new RGBWColor(0, 0, 255);
    private static final RGBWColor COLOR_OFF = new RGBWColor(0, 0, 0);

    private boolean blinkState = false;
    private double lastBlinkTime = 0;

    private boolean wasInsideRange = false;
    private double rumbleEndTime = 0;

    private static final int CANDLE_CAN_ID = 40;

    public LEDSubsystem(Supplier<Pose2d> poseSupplier, XboxController controller) {
        candle = new CANdle(CANDLE_CAN_ID);
        this.poseSupplier = poseSupplier;
        this.controller = controller;
    }

    @Override
    public void periodic() {
        
        double HUB_X = VisionConstants.hubPosition().getX();
        double HUB_Y = VisionConstants.hubPosition().getY();

        Optional<Alliance> allianceOptional = DriverStation.getAlliance();

        Pose2d robotPose = poseSupplier.get();
        double dx = robotPose.getX() - HUB_X;
        double dy = robotPose.getY() - HUB_Y;
        double distance = Math.hypot(dx, dy);
        boolean insideShootRange = distance <= SHOOT_LINE_DISTANCE;

        if (insideShootRange && !wasInsideRange) {
            rumbleEndTime = Timer.getFPGATimestamp() + 0.25;
        }
        wasInsideRange = insideShootRange;

        if (DriverStation.getMatchTime() < 0) {
            candle.setControl(rainbowRequest); 
            setRumble(0);
            return;
        }

        if (allianceOptional.isEmpty()) {
            setLED(COLOR_OFF);
            setRumble(0);
            return;
        }

        Alliance alliance = allianceOptional.get();

        if (!HubTracker.isActive(alliance)) {
            setLED(COLOR_OFF);
            setRumble(0);
            return;
        }

        RGBWColor allianceColor = (alliance == Alliance.Red) ? COLOR_RED : COLOR_BLUE;

        var timeRemainingOptional = HubTracker.timeRemainingInCurrentShift();

        if (timeRemainingOptional.isPresent() &&
            timeRemainingOptional.get().in(Units.Seconds) <= 5) {

            if (Timer.getFPGATimestamp() - lastBlinkTime > 0.08) {
                blinkState = !blinkState;
                lastBlinkTime = Timer.getFPGATimestamp();
            }

            if (blinkState) {
                setLED(allianceColor);
            } else {
                setLED(COLOR_OFF);
            }

            setRumble(0);
            return;
        }

        if (Timer.getFPGATimestamp() < rumbleEndTime) {
            setRumble(1);
        } else {
            setRumble(0);
        }

        setLED(allianceColor);
    }

    private void setLED(RGBWColor color) {
        candle.setControl(solidColorRequest.withColor(color));
    }

    private void setRumble(double strength) {
        controller.setRumble(GenericHID.RumbleType.kBothRumble, strength);
    }
}