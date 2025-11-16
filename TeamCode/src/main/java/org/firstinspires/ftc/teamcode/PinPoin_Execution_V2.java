package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
public class PinPoin_Execution_V2 {



    public class PinPoint_Execution {
        GoBildaPinpointDriver odo;

        // PID coefficients - tuned for two deadwheel odometry
        static final double kPx = 0.01, kIx = 0.0001, kDx = 0.0008;
        static final double kPy = 0.01, kIy = 0.0001, kDy = 0.0008;
        static final double kPz = 0.05, kIz = 0.001, kDz = 0.005;

        // Error tracking
        double lastXError = 0, lastYError = 0, lastZError = 0;
        double xErrorSum = 0, yErrorSum = 0, zErrorSum = 0;

        // Anti-windup limits
        static final double INTEGRAL_MAX = 0.5;
        static final double LOOP_TIME_MAX = 0.05; // 50ms safety limit

        // Constructor
        public PinPoint_Execution(GoBildaPinpointDriver odo) {
            this.odo = odo;
            // Configure odometry once in constructor
            odo.setOffsets(-100, 0, DistanceUnit.MM);
            odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
            odo.resetPosAndIMU();
        }


        public List<Double> move(double targetX, double targetY, double targetZ) {
            // Remove odometry configuration from here - done once in constructor
            Pose2D currentPos = odo.getPosition();

            // Calculate errors
            double xError = targetX - currentPos.getX(DistanceUnit.MM);
            double yError = targetY - currentPos.getY(DistanceUnit.MM);
            double zError = targetZ - currentPos.getHeading(AngleUnit.DEGREES);

            // Normalize heading error to [-180, 180]
            zError = normalizeHeading(zError);

            // Safely get loop time
            double loopTime = Math.min(odo.getLoopTime(), LOOP_TIME_MAX);

            // Calculate P (proportional) terms
            double xP = kPx * xError;
            double yP = kPy * yError;
            double zP = kPz * zError;

            // Calculate I (integral) terms with anti-windup
            xErrorSum += xError * loopTime;
            yErrorSum += yError * loopTime;
            zErrorSum += zError * loopTime;

            xErrorSum = Math.max(-INTEGRAL_MAX, Math.min(INTEGRAL_MAX, xErrorSum));
            yErrorSum = Math.max(-INTEGRAL_MAX, Math.min(INTEGRAL_MAX, yErrorSum));
            zErrorSum = Math.max(-INTEGRAL_MAX, Math.min(INTEGRAL_MAX, zErrorSum));

            double xI = kIx * xErrorSum;
            double yI = kIy * yErrorSum;
            double zI = kIz * zErrorSum;

            // Calculate D (derivative) terms
            double xD = kDx * (xError - lastXError) / loopTime;
            double yD = kDy * (yError - lastYError) / loopTime;
            double zD = kDz * (zError - lastZError) / loopTime;

            // Store errors for next iteration
            lastXError = xError;
            lastYError = yError;
            lastZError = zError;

            // Combine PID outputs
            double linXVel = xP + xI + xD;
            double linYVel = yP + yI + yD;
            double rotVel = zP + zI + zD;

            // Apply mecanum kinematics with rotation control
            double fLVelocity = linXVel - linYVel - rotVel;
            double fRVelocity = linXVel + linYVel + rotVel;
            double rLVelocity = linXVel + linYVel - rotVel;
            double rRVelocity = linXVel - linYVel + rotVel;

            List<Double> velocity = Arrays.asList(fLVelocity, fRVelocity, rLVelocity, rRVelocity);
            return velocity;
        }

        /**
         * Normalizes heading error to [-180, 180] range
         * Ensures shortest path rotation
         */
        private double normalizeHeading(double heading) {
            while (heading > 180) heading -= 360;
            while (heading < -180) heading += 360;
            return heading;
        }
    }

}
