package frc.robot.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class ElevatorConstants {
    
 public static enum Setpoint {
        Ground(Rotations.of(-0.5)),
        levelOne(Rotations.of(-1));
        //levelTwo(Rotations.of(-2));

        /** The position target of the setpoint in angular units. */
        public final Angle target;
        /** The position target of the setpoint in linear units. */
        public final Distance targetDist;

        private Setpoint(Angle target) {
            this.target = target;
            this.targetDist = kDrumRadius.times(target.in(Radians));
        }
        private Setpoint(Distance target) {
            this.target = Radians.of(target.div(kDrumRadius).magnitude());
            this.targetDist = target;
        }
    }

    public static final double kP = 36;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0.2;
    public static final double kV = 1.08;
    public static final double kA = 0;
    public static final double kG = 0;

    public static final double kGearRatio = 20;
    public static final Distance kDrumRadius = Meters.of(0.027178);
    public static final Distance kMaxHeight = Meters.of(0);

    public static final int motorOneId = 1;
    public static final int motorTwoId = 2;

}
