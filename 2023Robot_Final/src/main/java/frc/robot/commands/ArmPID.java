package frc.robot.commands;

import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmPID extends CommandBase {
    private final Arm arm;

    double phi_degrees;
    double theta_degrees;

    // Flags to determine movement status
    boolean upperRotation_complete = false;
    boolean lowerRotation_complete = false;

    // Initialization variables for the two movements
    int upperRotate_init = 1;
    int lowerRotate_init = 1;

    // Running encoder counts
    double upperCount = 0;
    double lowerCount = 0;

    // Update flags for debugging output
    int upperUpdate = 0;
    int lowerUpdate = 0;

    // Upper and lower targets
    double upperTarget = 0;
    double lowerTarget = 0;

    // Time stamps for determination of the local "dt"
    double upperLast_time;
    double lowerLast_time;

    // Error stamps for integration and derivative
    // components of PID
    double upperLast_error;
    double lowerLast_error;

    // Gear reduction parameters
    final double upperGearReduction = 100.0;
    final double lowerGearReduction = 64.0;
    final double lowerChainReduction = 4.0;

    // Mechanical dimensions
    private double upperArm = 33.50;
    private double lowerArm = 34.625;
    private double pivotHeight = 15.5; // need a good number here

    // Key angle variables
    double Theta;
    double Phi;

    // Desired Coordinates
    double X;
    double Y;

    double upperDeadband = 1000.0;
    double lowerDeadband = 1000.0;

    double upperError_sum = 0.0;
    double upperError_rate = 0.0;

    double lowerError_sum = 0.0;
    double lowerError_rate = 0.0;

    private Delay delay;

    private Timer time;

    ArmPID(Arm arm, double xSetpoint, double ySetpoint) {
        this.arm = arm;

        arm.resetFalconEncoders();

        // Start the timer
        time.start();

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        System.out.println("\nArmPID command started.\n");

    }

    @Override
    public void execute() {
        if (lowerRotation_complete == false) {

            rotateLowerArm(theta_degrees);
        }
        if (upperRotation_complete == false) {

            rotateUpperArm(phi_degrees + theta_degrees);
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.setUpperMotor(0);
        arm.setLowerMotor(0);
        System.out.println("\nArmPID command ended.\n");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /////////////////////////////////////////////////////////////////
    // Function: double computeLowerDriveRatio()
    /////////////////////////////////////////////////////////////////
    //
    // Purpose: Given the initial gear reduction and the
    // hub/sproket ratio. This function
    // computes the net gear reduction for the lower
    // motor.
    //
    // Arguments:void
    //
    // Returns: The gear reduction as double
    //
    // Remarks: 256:1 with intended configuration
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double computeLowerDriveRatio() {
        double dtmp;

        dtmp = lowerGearReduction * lowerChainReduction;

        return (dtmp);

    }

    /////////////////////////////////////////////////////////////////
    // Function: double compute_LowerCounts(double degrees)
    /////////////////////////////////////////////////////////////////
    //
    // Purpose: Computes the encoder counts for a rotation
    // of the specified number of degrees of the
    // lower pivot mechanism.
    //
    // Arguments:double degrees.
    //
    // Returns: The number of encoder counts for the rotation
    // requested
    //
    // Remarks: Just to get an idea of the magnitude of the
    // counts for 1 degree of rotation:
    // Assume there is a gear reduction of the order of 256:1.
    // If this were true it would imply 1456.36 counts per
    // degree.
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double compute_LowerCounts(double degrees) {
        double dtmp;
        double motor_revs;
        double counts_per_degree;

        // To rotate 360 degrees requires
        motor_revs = computeLowerDriveRatio();
        counts_per_degree = 2048 * motor_revs / 360.0;

        dtmp = counts_per_degree * degrees;

        return (dtmp);
    }

    double LowerCounts_perDegree() {
        double motor_revs;
        double counts_per_degree;

        // To rotate 360 degrees requires
        motor_revs = computeLowerDriveRatio();
        counts_per_degree = 2048 * motor_revs / 360.0;
        return (counts_per_degree);
    }

    double computeLowerDegrees_fromCounts(double counts) {
        double degrees;

        degrees = counts / LowerCounts_perDegree();

        return (degrees);

    }

    /////////////////////////////////////////////////////////////////
    // Function: double computeUpperDriveRatio()
    /////////////////////////////////////////////////////////////////
    //
    // Purpose: Given the initial gear reduction and the
    // hub/sproket ratio. This function
    // computes the net gear reduction for the lower
    // motor.
    //
    // Arguments:void
    //
    // Returns: The gear reduction as double
    //
    // Remarks: 100:1 with intended configuration
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double computeUpperDriveRatio() {
        double dtmp;

        dtmp = upperGearReduction;

        return (dtmp);

    }

    /////////////////////////////////////////////////////////////////
    // Function: double compute_UpperCounts(double degrees)
    /////////////////////////////////////////////////////////////////
    //
    // Purpose: Computes the encoder counts for a rotation
    // of the specified number of degrees of the
    // upper pivot mechanism.
    //
    // Arguments:double degrees
    //
    // Returns: The number of encoder counts for the rotation
    // requested
    //
    // Remarks: Just to get an idea of the magnitude of the
    // counts for 1 degree of rotation:
    // Assume there is a gear reduction of the order of 100:1.
    // If this were true it would imply 568.89 counts per
    // degree.
    //
    // In this case (upper arm) "degrees" is the sum of
    // theta and phi (see diagram)
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double compute_UpperCounts(double degrees) {
        double dtmp;
        double motor_revs;
        double counts_per_degree;

        // To rotate 360 degrees requires
        motor_revs = computeUpperDriveRatio();
        counts_per_degree = 2048 * motor_revs / 360.0;

        dtmp = counts_per_degree * degrees;

        return (dtmp);
    }

    double upperCounts_perDegree() {
        double motor_revs;
        double counts_per_degree;

        // To rotate 360 degrees requires
        motor_revs = computeUpperDriveRatio();
        counts_per_degree = 2048 * motor_revs / 360.0;

        return (counts_per_degree);
    }

    double computeUpperDegrees_fromCounts(double counts) {
        double degrees;

        degrees = counts / upperCounts_perDegree();

        return (degrees);

    }

    /////////////////////////////////////////////////////////////////
    // Function: int computeArmAngles(double x, double y)
    /////////////////////////////////////////////////////////////////
    //
    // Purpose: Given the desired coordinates (x,y), this function
    // computes the angles theta and phi, the lower and
    // upper arm angle required to place the center of
    // the "grabber mechanism" at (x,y).
    //
    // Arguments:Accepts the desired coordinates (x,y)
    //
    // Returns: Returns zero normally, may return an error
    // code (TBD) in the event of a problem.
    //
    // Purpose: Demonstration of a numerical approach to solving an
    // algebraic system of two equations in two
    // unknowns. This is necessary when an analytic solution
    // is either impossible or difficult to obtain. The technique
    // depends on finding the minimum delta between something you
    // know (in this case x and y) and an expression involving one
    // of the two variables (in this case the angles phi and theta).
    //
    // The equation for x is:
    // x = LWR_ARM*sin(theta) + UPPR_ARM*sin(phi)
    //
    // The equation for y is:
    //
    // y = LWR_ARM*cos(theta) - UPPR_ARM*cos(phi) + LWR_PIVOT + WHL_RAD
    //
    // Solve the first for sin(theta) and substitute into the second
    // yields a result for y that has only phi as a variable. Use
    // a simple numeric technique to solve for phi and substitute
    // back into the equation for and solve (numerically) for theta.
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    int computeArmAngles(double x, double y) {
        int i = 0;
        double dtmp1 = 0;
        double dtmp2 = 0;
        double min_phi = 1000.0;
        double min_theta = 1000.0;
        double delta = 0.0;
        double min_delta = 1000.0;

        // limit phi to 90 degrees or PI/2 radians, solve for the angle phi that
        // results in the minimum delta
        if (x > 0.0) {
            for (i = 0; i < 1000; i++) {
                Phi = i * (Math.PI / 2000.0);

                dtmp1 = Math.sqrt(lowerArm * lowerArm - (x * x - 2.0 * x * upperArm * Math.sin(Phi) +
                        upperArm * upperArm * Math.sin(Phi) * Math.sin(Phi))) - upperArm * Math.cos(Phi) +
                        pivotHeight;

                delta = y - dtmp1;
                if (delta < 0.0)
                    delta *= -1.0;
                if (delta < min_delta) {
                    min_delta = delta;
                    min_phi = Phi;
                }
            }
            Phi = min_phi; // Phi determined
        } else if (x < 0.0) {
            for (i = 0; i < 1000; i++) {
                Phi = -(i * (Math.PI / 2000.0));

                dtmp1 = Math.sqrt(lowerArm * lowerArm - (x * x - 2.0 * x * upperArm * Math.sin(Phi) +
                        upperArm * upperArm * Math.sin(Phi) * Math.sin(Phi))) - upperArm * Math.cos(Phi) +
                        pivotHeight;

                delta = y - dtmp1;
                if (delta < 0.0)
                    delta *= -1.0;
                if (delta < min_delta) {
                    min_delta = delta;
                    min_phi = Phi;
                }
            }
            Phi = min_phi; // Phi determined
        }

        if (min_delta > 1.0) {
            System.out.printf("\nSolution for phi fails to converge.|n");
        } else {
            System.out.printf("\nmin_delta = %.6f\n", min_delta);
            System.out.printf("\nPhi = %.3f radians\n", Phi);
            System.out.printf("\nPhi = %.3f degrees\n", Phi * 180.0 / Math.PI);
        }

        // same technique, limit theta to 90 degrees or PI/2 radians, solve for the
        // angle theta
        // that results in the minimum delta

        // reassign min_delta
        min_delta = 1000.0;
        if (x > 0.0) {
            for (i = 0; i < 1000; i++) {
                Theta = i * (Math.PI / 2000.0);
                dtmp2 = lowerArm * Math.sin(Theta) + upperArm * Math.sin(Phi);
                delta = x - dtmp2;
                if (delta < 0.0)
                    delta *= -1.0;
                if (delta < min_delta) {
                    min_delta = delta;
                    min_theta = Theta;
                }
            }
            Theta = min_theta;
        } else if (x < 0.0) {
            for (i = 0; i < 1000; i++) {
                Theta = -(i * (Math.PI / 2000.0));
                dtmp2 = lowerArm * Math.sin(Theta) + upperArm * Math.sin(Phi);
                delta = x - dtmp2;
                if (delta < 0.0)
                    delta *= -1.0;
                if (delta < min_delta) {
                    min_delta = delta;
                    min_theta = Theta;
                }
            }
            Theta = min_theta;

        }

        if (min_delta > 1.0)
            System.out.printf("\nSolution for theta fails to converge\n");
        else {
            System.out.printf("\nmin_delta = %.6f\n", min_delta);
            System.out.printf("\nTheta = %.3f radians\n", Theta);
            System.out.printf("\nTheta = %.3f degrees\n", Theta * 180.0 / Math.PI);
        }
        return (0);
    }

    /////////////////////////////////////////////////////////////////
    // Function: double rotateUpperArm(double degrees)
    /////////////////////////////////////////////////////////////////
    //
    // Purpose: Rotates the upper arm to the desired angle
    //
    // Arguments:Accepts the desired angle in degrees
    //
    // Returns: The final error in degrees.
    //
    // Remarks: degrees = (Theta + Phi)*180.0/PI
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double rotateUpperArm(double degrees) {
        boolean debug = true;
        double error = 0;

        // These are used for the integral and derivative terms
        // and use the WPILib Timer class.
        double current_time = 0.0;
        double dt = 0;

        // our PID constants (always need final tuning)
        double kp = 0.00005;
        double ki = 0.0002;
        double kd = 0.0000;
        double power = 0.0;

        // First time through, get the initial position. We compute
        // the number of counts associated with the degrees and
        // compute our target count. If we have our motor inversion
        // state correctly set we add the computed counts to the
        // present position to rotate the drive clockwise. We start
        // the motor within this block.
        if (upperRotate_init == 1) {
            upperCount = arm.getUpperFalconEncoder();

            // Must rotate the upper arm theta + phi. The argument
            // "degrees" is the sum of phi and theta
            upperTarget = upperCount + compute_UpperCounts(degrees);
            error = upperTarget - upperCount;
            if (debug == true) {
                System.out.printf("\nUpper initial count = %.3f\n", upperCount);
                System.out.printf("\nUpper rotation target = %.3f\n", upperTarget);
            }

            power = kp * error;
            // Here's the clamp
            if (power > 0.5)
                power = 0.5;
            if (power < 0.05)
                power = 0.05;

            // Get it started
            arm.setUpperMotor(power);

            upperCount = 0;
            upperRotate_init = 0;
            upperLast_error = 0;
            upperError_sum = 0.0;
            upperError_rate = 0.0;

        }

        if (upperRotation_complete == false) {

            // Set the motor in motion, wait a bit and then read the encoder
            // and compute the error.
            if (upperCount < upperTarget) {

                current_time = time.get();

                delay.delay_milliseconds(5.0);

                upperCount = arm.getUpperFalconEncoder();

                error = upperTarget - upperCount; // In this case should be positive

                dt = current_time - upperLast_time;

                if (dt > 0.0) {
                    upperError_rate = (error - upperLast_error) / dt;
                }

                // Start off integrating when within some percentage of the target
                if (error < 0.5 * upperTarget)
                    upperError_sum += error * dt; // don't apply the integral term until closer to target

                power = kp * error + ki * upperError_sum + kd * upperError_rate;

                // Here's the clamp on the upper power
                if (power > 0.5)
                    power = 0.5;
                // Here is another clamp if we need to correct from an overshoot
                if (power < -0.2)
                    power = -0.2;

                arm.setUpperMotor(power);

                if (debug == true) {
                    System.out.printf("\nUpper error_sum = %.3f  error_rate = %.3f  dt = %.3f\n", upperError_sum,
                            upperError_rate, dt);
                    System.out.printf("\nUpper:kp*error = %.3f\n", kp * error);
                    System.out.printf("\nUpper:ki*error_sum = %.3f\n", ki * upperError_sum);
                    System.out.printf("\nUpper:power = %.3f\n", power);
                }

                // Record last time and last error
                upperLast_time = time.get();
                upperLast_error = error;
            }

            // Are we within the deadband for the turn? Have we overshot?
            // If either of these are true, stop the motor. We need to
            // set a flag that the motion has been completed or we will
            // continue to drive the motors on calls after we have
            // satisfied this condition. Since "error" has function
            // scope it is set to zero every time this function is called
            // 2/9/2023: Interesting if we modify the if statement to
            // require that we are within the deadband. Seems to improve
            // our convergence.
            // if ((Math.abs(error)<turn_deadband)||(error<0.0)) {
            if (Math.abs(error) < upperDeadband) {
                arm.setUpperMotor(0);
                upperRotation_complete = true;
                System.out.printf("\nUpper Target = %.3f\n", upperTarget);
                System.out.printf("\nUpper Final count = %.3f  error = %.3f\n", upperCount, upperLast_error);

            }

            if ((debug == true) && (upperRotation_complete == false)) {
                upperUpdate++;
                if (upperUpdate == 10) {
                    System.out.printf("\nUpper count = %.3f  Upper error = %.3f\n", upperCount, error);
                    upperUpdate = 0;
                }
            }
        } // if(turn_motion_complete==false)

        return (computeUpperDegrees_fromCounts(upperLast_error));

    }

    /////////////////////////////////////////////////////////////////
    // Function:
    /////////////////////////////////////////////////////////////////
    //
    // Purpose:
    //
    // Arguments:
    //
    // Returns:
    //
    // Remarks: degrees = (theta)*180.0/PI
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double rotateLowerArm(double degrees) {
        boolean debug = false;
        double error = 0;

        // These are used for the integral and derivative terms
        // and use the WPILib Timer class.
        double current_time = 0.0;
        double dt = 0;

        // our PID constants (always need final tuning)
        double kp = 0.00005;
        double ki = 0.0002;
        double kd = 0.0000;
        double power = 0.0;

        // First time through, get the initial position. We compute
        // the number of counts associated with the degrees and
        // compute our target count. If we have our motor inversion
        // state correctly set we add the computed counts to the
        // present position to rotate the drive clockwise
        if (lowerRotate_init == 1) {
            lowerCount = arm.getLowerFalconEncoder();

            // Must rotate the upper arm theta + phi. The argument
            // "degrees" is the sum of phi and theta
            lowerTarget = lowerCount + compute_LowerCounts(degrees);
            error = lowerTarget - lowerCount;
            if (debug == true) {
                System.out.printf("\nLower initial count = %.3f\n", lowerCount);
                System.out.printf("\nLower rotation target = %.3f\n", lowerTarget);
            }

            power = kp * error;
            // Here's the clamp
            if (power > 0.5)
                power = 0.5;
            if (power < 0.05)
                power = 0.05;

            // Get it started
            arm.setLowerMotor(power);
            lowerCount = 0;
            lowerRotate_init = 0;
            lowerLast_error = 0;
            lowerError_sum = 0.0;
            lowerError_rate = 0.0;
        }

        if (lowerRotation_complete == false) {

            // Set the motor in motion, wait a bit and then read the encoder
            // and compute the error.
            if (lowerCount < lowerTarget) {

                current_time = time.get();

                delay.delay_milliseconds(5.0);

                lowerCount = arm.getLowerFalconEncoder();

                error = lowerTarget - lowerCount; // In this case should be positive

                dt = current_time - lowerLast_time;

                if (dt > 0.0) {
                    lowerError_rate = (error - lowerLast_error) / dt;
                }

                // Start off integrating when within some percentage of the target
                if (error < 0.5 * lowerTarget)
                    lowerError_sum += error * dt; // don't apply the integral term until closer to target

                power = kp * error + ki * lowerError_sum + kd * lowerError_rate;
                // Here's the clamp
                if (power > 0.5)
                    power = 0.5;
                if (power < -0.2)
                    power = -0.2;

                arm.setLowerMotor(power);

                if (debug == true) {
                    System.out.printf("\nLower: error_sum = %.3f  error_rate = %.3f  dt = %.3f", lowerError_sum,
                            lowerError_rate, dt);
                    System.out.printf("\nLower: kp*error = %.3f\n", kp * error);
                    System.out.printf("\nLower: ki*error_sum = %.3f\n", ki * lowerError_sum);
                    System.out.printf("\nLower: power = %.3f\n", power);
                }

                // Record last time and last error
                lowerLast_time = time.get();
                lowerLast_error = error;
            }

            // Are we within the deadband for the turn? Have we overshot?
            // If either of these are true, stop the motor. We need to
            // set a flag that the motion has been completed or we will
            // continue to drive the motors on calls after we have
            // satisfied this condition. Since "error" has function
            // scope it is set to zero every time this function is called
            // 2/9/2023: Interesting if we modify the if statement to
            // require that we are within the deadband. Seems to improve
            // our convergence.
            if (Math.abs(error) < lowerDeadband) {
                arm.setLowerMotor(0);
                lowerRotation_complete = true;
                System.out.printf("\nLower: target = %.3f\n", lowerTarget);
                System.out.printf("\nLower: final count = %.3f  error = %.3f\n", lowerCount, lowerLast_error);

            }

            if ((debug == true) && (lowerRotation_complete == false)) {
                lowerUpdate++;
                if (lowerUpdate == 10) {
                    System.out.printf("\nLower: count = %.3f  error = %.3f\n", lowerCount, error);
                    lowerUpdate = 0;
                }
            }
        } // if(turn_motion_complete==false)

        return (computeLowerDegrees_fromCounts(lowerLast_error));

    }
}
