package org.firstinspires.ftc.teamcode.constants;

public class slideConstants {
    public enum slidePos {
        // TODO: Set actual motor encoder positions
        INTAKING(    -31, 0.349142410),
        INTERMEDIATE(-31, 0.41), // Prevents claw from slamming into aligner
        BOTTOM(      -308, 0.71019120),
        MIDDLE(      -627, 0.71019120),
        TOP(         -1081, 0.71019120);

        // Enum values

        private final double slideAngle; //controls the position of the little motor at the bottom
        private final double slideMotor; //controls the motor that coils the string

        slidePos(double slideAngle, double slideMotor) {
            this.slideAngle = slideAngle;
            this.slideMotor = slideMotor;
        }

        public double getslideAngle() { return slideAngle; }
        public double slideMotor() { return slideMotor; }
    }

}
