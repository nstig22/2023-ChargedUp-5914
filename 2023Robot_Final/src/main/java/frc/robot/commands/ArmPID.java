package frc.robot.commands;

import frc.robot.subsystems.Arm;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmPID extends CommandBase {
    private final Arm arm;
    private double xSetpoint;
    private double ySetpoint;

    double phi_degrees;
    double theta_degrees;

    boolean LowLevelEnabled = false;
    boolean MidLevelEnabled = false;
    boolean UpperLevelEnabled = false;

    // Flags to determine movement status
    boolean upperRotation_complete = true;
    boolean lowerRotation_complete = true;
    boolean MoveComplete = true;
    boolean endCmd = false;

    // Initialization variables for the two movements
    int upperRotate_init = 1;
    int lowerRotate_init = 1;
    int moveInit = 1;

    // Running encoder counts
    double upperCount = 0;
    double lowerCount = 0;
    double upperDelta = 0;
    double lowerDelta = 0;

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
    final double upperGearReduction = 64.0;
    final double lowerGearReduction = 64.0;

    // lower chain reduction: 48 teeth vs 15 teeth
    final double lowerChainReduction = 48 / 15;

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

    double upperDeadband = 500.0;
    double lowerDeadband = 500.0;

    double upperError_sum = 0.0;
    double upperError_rate = 0.0;

    double lowerError_sum = 0.0;
    double lowerError_rate = 0.0;

    boolean RevLowLevelEnabled = false;
    boolean RevMidLevelEnabled = false;
    boolean RevUpperLevelEnabled = false;
    boolean ResetEnabled = false;

    private Delay delay;

    private Timer time;

    BooleanSupplier low;
    BooleanSupplier mid;
    BooleanSupplier high;

    public ArmPID(Arm arm, double xSetpoint, double ySetpoint) {
        this.arm = arm;
        this.xSetpoint = xSetpoint;
        this.ySetpoint = ySetpoint;

        // arm.resetFalconEncoders();

        time = new Timer();

        delay = new Delay();

        // Start the timer
        time.start();

        addRequirements(arm);
    }

    /*
     * public ArmPID(Arm arm, BooleanSupplier low, BooleanSupplier mid,
     * BooleanSupplier high){
     * this.arm = arm;
     * this.low = low;
     * this.mid = mid;
     * this.high = high;
     * 
     * time = new Timer();
     * 
     * delay = new Delay();
     * 
     * time.start();
     * 
     * addRequirements(arm);
     * }
     */

    @Override
    public void initialize() {
        System.out.println("\nArmPID command started.\n");

        MoveComplete = true;
        LowLevelEnabled = false;
        MidLevelEnabled = false;
        UpperLevelEnabled = false;

        X = xSetpoint;
        Y = ySetpoint;

        // computeArmAngles(X, Y);

        // phi_degrees = Phi * 180.0 / Math.PI;
        // theta_degrees = Theta * 180.0 / Math.PI;

        // System.out.printf("\nPhi = %.3f Theta = %.3f degrees", phi_degrees,
        // theta_degrees);
    }

    @Override
    public void execute() {
        // We detect a button press and enable the
        // appropriate movement. We want to make sure that any
        // movements in progress are complete.

        if (MoveComplete == true && endCmd == false) {
            moveInit = 1;
            MoveComplete = false;
            System.out.println("\nMoving arm\n");
            move2Position(X, Y);
        }

        /*
         * if (MoveComplete == true) {
         * 
         * MidLevelEnabled = true;
         * LowLevelEnabled = false;
         * UpperLevelEnabled = false;
         * moveInit = 1;
         * MoveComplete = false;
         * System.out.printf("\nSetting to mid level\n");
         * 
         * }
         * if (MidLevelEnabled == true) {
         * processMidLevel();
         * }
         * 
         * if (MoveComplete == true) {
         * 
         * MidLevelEnabled = false;
         * LowLevelEnabled = true;
         * UpperLevelEnabled = false;
         * moveInit = 1;
         * MoveComplete = false;
         * System.out.printf("\nSetting to low level\n");
         * 
         * }
         * if (LowLevelEnabled == true) {
         * processLowLevel();
         * }
         * 
         * if (MoveComplete == true) {
         * 
         * MidLevelEnabled = false;
         * LowLevelEnabled = false;
         * UpperLevelEnabled = true;
         * moveInit = 1;
         * MoveComplete = false;
         * System.out.printf("\nSetting to uppper level\n");
         * 
         * }
         * if (UpperLevelEnabled == true) {
         * processUpperLevel();
         * // arm.rotateUpperArm_CCW(20.0);
         * }
         */
    }

    @Override
    public void end(boolean interrupted) {
        arm.setUpperMotor(0);
        arm.setLowerMotor(0);

        moveInit = 1;

        endCmd = false;

        System.out.println("\nArmPID command ended.\n");
    }

    @Override
    public boolean isFinished() {
        if (endCmd) {
            return true;
        } else {
            return false;
        }
    }

    /*void processArmButtons() {

        // Here is the reset function
        if ((stick.getRawButtonPressed(5) == true) && (MoveComplete == true)) {
            MidLevelEnabled = false;
            LowLevelEnabled = false;
            UpperLevelEnabled = false;
            RevMidLevelEnabled = false;
            RevLowLevelEnabled = false;
            RevUpperLevelEnabled = false;
            ResetEnabled = true;
            moveInit = 1;
            MoveComplete = false;
            System.out.printf("\nButton 5 pressed.\n");
            reset();

        }
        if (ResetEnabled == true) {
            reset();
        }

        // Low position
        if ((stick.getRawButtonReleased(4) == true) &&
                (stick.getRawButtonPressed(1) == true) &&
                (MoveComplete == true)) {
            MidLevelEnabled = false;
            LowLevelEnabled = true;
            UpperLevelEnabled = false;
            RevMidLevelEnabled = false;
            RevLowLevelEnabled = false;
            RevUpperLevelEnabled = false;
            ResetEnabled = false;
            moveInit = 1;
            MoveComplete = false;
            System.out.printf("\nButton 4 released.\n");
            System.out.printf("\nButton 1 pressed.\n");

        }
        if (LowLevelEnabled == true) {
            processLowLevel();
        }

        // Mid position
        if ((stick.getRawButtonReleased(4) == true) &&
                (stick.getRawButtonPressed(2) == true) &&
                (MoveComplete == true)) {
            MidLevelEnabled = true;
            LowLevelEnabled = false;
            UpperLevelEnabled = false;
            RevMidLevelEnabled = false;
            RevLowLevelEnabled = false;
            RevUpperLevelEnabled = false;
            ResetEnabled = false;
            moveInit = 1;
            MoveComplete = false;
            System.out.printf("\nButton 4 released.\n");
            System.out.printf("\nButton 2 pressed.\n");

        }
        if (MidLevelEnabled == true) {
            processMidLevel();
        }

        // Upper position
        if ((stick.getRawButtonReleased(4) == true) &&
                (stick.getRawButtonPressed(3) == true) &&
                (MoveComplete == true)) {
            MidLevelEnabled = false;
            LowLevelEnabled = false;
            UpperLevelEnabled = true;
            RevMidLevelEnabled = false;
            RevLowLevelEnabled = false;
            RevUpperLevelEnabled = false;
            ResetEnabled = false;
            moveInit = 1;
            MoveComplete = false;
            System.out.printf("\nButton 4 released.\n");
            System.out.printf("\nButton 3 pressed.\n");

        }
        if (UpperLevelEnabled == true) {
            processUpperLevel();
        }

        // Reverse lower position
        if ((stick.getRawButtonPressed(4) == true) &&
                (stick.getRawButtonPressed(1) == true) &&
                (MoveComplete == true)) {

            MidLevelEnabled = false;
            LowLevelEnabled = false;
            UpperLevelEnabled = false;
            RevMidLevelEnabled = false;
            RevLowLevelEnabled = true;
            RevUpperLevelEnabled = false;
            ResetEnabled = false;
            moveInit = 1;
            MoveComplete = false;
            System.out.printf("\nButton 4 pressed.\n");
            System.out.printf("\nButton 1 pressed.\n");

        }
        if (RevLowLevelEnabled == true) {
            arm.processRevLowLevel();
        }

        // Reverse mid position
        if ((stick.getRawButtonPressed(4) == true) &&
                (stick.getRawButtonPressed(2) == true) &&
                (MoveComplete == true)) {

            MidLevelEnabled = false;
            LowLevelEnabled = false;
            UpperLevelEnabled = false;
            RevMidLevelEnabled = true;
            RevLowLevelEnabled = false;
            RevUpperLevelEnabled = false;
            ResetEnabled = false;
            moveInit = 1;
            MoveComplete = false;
            System.out.printf("\nButton 4 pressed.\n");
            System.out.printf("\nButton 2 pressed.\n");

        }
        if (RevMidLevelEnabled == true) {
            processRevMidLevel();
        }

        // Reverse upper position
        if ((stick.getRawButtonPressed(4) == true) &&
                (stick.getRawButtonPressed(3) == true) &&
                (MoveComplete == true)) {

            MidLevelEnabled = false;
            LowLevelEnabled = false;
            UpperLevelEnabled = false;
            RevMidLevelEnabled = false;
            RevLowLevelEnabled = false;
            RevUpperLevelEnabled = true;
            ResetEnabled = false;
            moveInit = 1;
            MoveComplete = false;
            System.out.printf("\nButton 4 pressed.\n");
            System.out.printf("\nButton 3 pressed.\n");

        }
        if (RevUpperLevelEnabled == true) {
            processRevUpperLevel();
        }

    }*/

    /****************************************************************/
    // These function relate to the mechanical gear reductions
    // and encoder counts associated with degrees of movement
    // and vice-versa
    /****************************************************************/

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

    // Computes the counts per one degree of movement
    // for the Lower drive
    double LowerCounts_perDegree() {
        double motor_revs;
        double counts_per_degree;

        // To rotate 360 degrees requires
        motor_revs = computeLowerDriveRatio();
        counts_per_degree = 2048 * motor_revs / 360.0;
        return (counts_per_degree);
    }

    // Computes the degrees of movement from the
    // lower motor encoder.
    double computeLowerDegrees_fromCounts(double counts) {
        double degrees;

        degrees = counts / LowerCounts_perDegree();

        return (degrees);

    }

    /////////////////////////////////////////////////////////////////
    // Function: double computeUpperDriveRatio()
    /////////////////////////////////////////////////////////////////
    //
    // Purpose: Given the gear reduction. This function
    // computes the gear reduction for the upper
    // motor.
    //
    // Arguments:void
    //
    // Returns: The gear reduction as double
    //
    // Remarks: 64:1 with intended configuration
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
    // Assume there is a gear reduction of the order of 64:1.
    // If this were true it would imply 364.1 counts per
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
    // Function: int process***Level()
    /////////////////////////////////////////////////////////////////
    //
    // Purpose: Processes the arm movements to specific coordinates
    //
    // Arguments:void
    //
    // Returns: zero, may be modified at a future date to return
    // an error code of some sort.
    //
    // Remarks: Coordinates for the various levels TBD.
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    int reset() {
        if (MoveComplete == false) {
            X = 0.0;
            Y = 17.625; // This is the height above the floor
                        // of the centerline of the grabber

            move2Position(X, Y);
        }
        return (0);
    }

    int processLowLevel() {
        if (MoveComplete == false) {
            X = 26.0;
            Y = 6.5;

            move2Position(X, Y);
        }
        return (0);
    }

    int processRevLowLevel() {
        if (MoveComplete == false) {
            X = -26.0;
            Y = 6.5;

            move2Position(X, Y);
        }
        return (0);
    }

    int processMidLevel() {
        if (MoveComplete == false) {
            X = 30;
            Y = 24;

            move2Position(X, Y);
        }
        return (0);
    }

    int processRevMidLevel() {
        if (MoveComplete == false) {
            X = -30;
            Y = 24;

            move2Position(X, Y);
        }
        return (0);
    }

    int processUpperLevel() {
        if (MoveComplete == false) {
            X = 30;
            Y = 36;

            move2Position(X, Y);
        }
        return (0);
    }

    int processRevUpperLevel() {
        if (MoveComplete == false) {
            X = -30;
            Y = 36;

            move2Position(X, Y);
        }
        return (0);
    }

    /////////////////////////////////////////////////////////////////
    // Function: double move2Position(double x, double y)
    /////////////////////////////////////////////////////////////////
    //
    // Purpose: Moves arm grabber to coordinates (x,y)
    //
    // Arguments:Accepts the coordinates (x,y) as doubles
    //
    // Returns: Zero
    //
    // Remarks:
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    int move2Position(double x, double y) {
        double upper_degrees = 0;
        double lower_degrees = 0;

        System.out.println("\nmove2Position calleed\n");

        if (moveInit == 1) {

            // Compute the existing location, and the
            // upper and lower targets in degrees.
            // Compute the number of degrees of movement
            // required to hit the new position

            // Determine the desired theta and phi
            computeArmAngles(x, y);

            // Where are we in terms of theta and phi? We
            // read the counts and convert to degrees
            upperCount = arm.getUpperFalconEncoder();
            // upper_degrees is the sum of theta and phi
            upper_degrees = computeUpperDegrees_fromCounts(upperCount);

            lowerCount = arm.getLowerFalconEncoder();
            // lower_degrees is just theta
            lower_degrees = computeLowerDegrees_fromCounts(lowerCount);

            upperDelta = ((Theta + Phi) * 180.0 / Math.PI) - upper_degrees;
            lowerDelta = (Theta * 180.0 / Math.PI) - lower_degrees;

            System.out.printf("\nTheta= %.3f degrees\n", Theta * 180.0 / Math.PI);
            System.out.printf("\nPhi = %.3f degrees\n", Phi * 180.0 / Math.PI);
            System.out.printf("\nupper_degrees = %.3f\n", upper_degrees);
            System.out.printf("\nlower_degrees = %.3f\n", lower_degrees);
            System.out.printf("\nupperDelta = %.3f\n", upperDelta);
            System.out.printf("\nlowerDelta = %.3f\n", lowerDelta);

            upperRotate_init = 1;
            lowerRotate_init = 1;

            upperRotation_complete = false;
            lowerRotation_complete = false;

            moveInit = 0;
        }

        // Move to position
        if (lowerRotation_complete == false) {
            rotateLowerArm(lowerDelta);
        }

        if (upperRotation_complete == false) {
            rotateUpperArm(upperDelta);
        }

        if ((lowerRotation_complete == true) && (upperRotation_complete == true)) {
            MoveComplete = true;
            System.out.printf("\nMovement Complete\n");
        }
        return (0);
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
    // Arguments:Accepts the desired coordinates (x,y). Note that
    // the origin of the coordinate system is the center
    // of the robot length for x and the ground for y.
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
    // 3/1/2023: Allowed for +/-PI/2 range in the angle phi.
    // This allows computation of x coordinates close to the
    // robot near the ground.
    //
    // 3/12/2023: Note that for negative x it is possible
    // that theta could be negative and phi positive. It is
    // also possible for positive x that theta is positive
    // while phi is negative. Not obvious but true.
    // Added boolean debug to enable debugging output
    // during testing. Can be eliminated with debug=false.
    // Added a final test to limit error in recomputation
    // of x and y to 0.100 inches.
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    int computeArmAngles(double x, double y) {
        boolean debug = true;
        int i = 0;
        double dtmp1 = 0;
        double dtmp2 = 0;
        double min_phi = 1000.0;
        double min_theta = 1000.0;
        double delta = 0.0;
        double min_delta = 1000.0;

        System.out.println("\ncomputeArmAngles callsd\n");

        // limit phi to +/-90 degrees or +/-PI/2 radians, solve for the angle phi that
        // results in the minimum delta
        for (i = -1000; i < 1000; i++) {
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

        if (min_delta > 1.0) {
            System.out.printf("\nSolution for phi fails to converge.|n");
        } else {
            if (debug == true) {
                System.out.printf("\nmin_delta = %.6f\n", min_delta);
                System.out.printf("\nPhi = %.3f radians\n", Phi);
                System.out.printf("\nPhi = %.3f degrees\n", Phi * 180.0 / Math.PI);
            }
        }

        // Same technique, limit theta to +/-90 degrees or +/-PI/2 radians.
        // Solve for the angle theta that results in the minimum delta

        // reassign min_delta
        min_delta = 1000.0;

        for (i = -1000; i < 1000; i++) {
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

        if (min_delta > 1.0)
            System.out.printf("\nSolution for theta fails to converge\n");
        else {
            if (debug == true) {
                System.out.printf("\nmin_delta = %.6f\n", min_delta);
                System.out.printf("\nTheta = %.3f radians\n", Theta);
                System.out.printf("\nTheta = %.3f degrees\n", Theta * 180.0 / Math.PI);
            }
        }

        // Test. Limit error to 0.1 inches for either x or y. Using the
        // two angles, compute x and y.
        if (debug == true)
            System.out.printf("\n\nTesting Solution, should reproduce submitted coordinates.\n");

        dtmp1 = lowerArm * Math.sin(Theta) + upperArm * Math.sin(Phi);
        dtmp2 = lowerArm * Math.cos(Theta) - upperArm * Math.cos(Phi) + pivotHeight;

        System.out.printf("\nx = % .3f  y = % .3f\n", dtmp1, dtmp2);

        if ((Math.abs(x - dtmp1) > 0.1) || (Math.abs(y - dtmp2) > 0.1)) {
            return (1);
        } else {
            return (0);
        }
    }

    /////////////////////////////////////////////////////////////////
    // Function: double rotateUpperArm(double degrees)
    /////////////////////////////////////////////////////////////////
    //
    // Purpose: For the sum of theta and phi (degrees), this
    // function will decide on the basis of the sign
    // of the sum which direction to rotate the upper
    // arm.
    //
    // Arguments:double degrees
    //
    // Returns: The error of the movement in degrees
    //
    // Remarks:
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double rotateUpperArm(double degrees) {

        double error = 0.0;

        if (upperRotation_complete == false) {

            if (degrees > 0.0) {
                error = rotateUpperArm_CW(degrees);
            } else if (degrees < 0.0) {
                degrees *= -1.0;
                error = rotateUpperArm_CCW(degrees);
            }
        }

        return (error);

    }

    /////////////////////////////////////////////////////////////////
    // Function: double rotateUpperArm_CW(double degrees)
    /////////////////////////////////////////////////////////////////
    //
    // Purpose: Rotate the upper arm clockwise the number of
    // degrees specified in the argument.
    //
    // Arguments: The number of degrees - must be a postive
    // number. This is taken care of in the previous
    // function.
    //
    // Returns: The error in degrees.
    //
    // Remarks:
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double rotateUpperArm_CW(double degrees) {
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
            if (power < 0.1)
                power = 0.2;

            // Get it started
            arm.setUpperMotor(power);

            upperRotate_init = 0;
            upperLast_error = 0;
            upperError_sum = 0.0;
            upperError_rate = 0.0;
            upperLast_time = 0.0;

            upperRotation_complete = false;

        }

        if (upperRotation_complete == false) {

            // Set the motor in motion, wait a bit and then read the encoder
            // and compute the error.
            if (upperCount < upperTarget) {

                current_time = time.get();

                delay.delay_milliseconds(5.0);

                upperCount = arm.getUpperFalconEncoder();

                error = upperTarget - upperCount; // In this case should be positive

                if ((Math.abs(error) < upperDeadband) || (error < 0.0)) {
                    arm.setUpperMotor(0);
                    upperRotation_complete = true;
                    System.out.printf("\nUpper Target = %.3f\n", upperTarget);
                    System.out.printf("\nUpper Final count = %.3f  error = %.3f\n", upperCount, error);
                    return (computeUpperDegrees_fromCounts(error));
                }

                if (upperLast_time != 0.0)
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
                if (power < 0.1)
                    power = 0.2;

                arm.setUpperMotor(power);

                if (debug == true) {
                    if (upperUpdate == 5) {
                        System.out.printf("\nUpper count = %.3f  Upper error = %.3f\n", upperCount, error);
                        System.out.printf("\nUpper error_sum = %.3f  error_rate = %.3f  dt = %.3f\n", upperError_sum,
                                upperError_rate, dt);
                        System.out.printf("\nUpper:kp*error = %.3f\n", kp * error);
                        System.out.printf("\nUpper:ki*error_sum = %.3f\n", ki * upperError_sum);
                        System.out.printf("\nUpper:power = %.3f\n", power);
                        upperUpdate = 0;
                    }
                    upperUpdate++;
                }

            } // if(upperCount<upperTarget)

            // Record last time and last error
            upperLast_time = time.get();
            upperLast_error = error;

        } // if(turn_motion_complete==false)

        return (computeUpperDegrees_fromCounts(upperLast_error));

    }

    /////////////////////////////////////////////////////////////////
    // Function: double rotateUpperArm_CCW(double degrees)
    /////////////////////////////////////////////////////////////////
    //
    // Purpose: Rotate the upper arm counter-clockwise the number of
    // degrees specified in the argument.
    //
    // Arguments: The number of degrees - must be a postive
    // number. This is taken care of in the
    // rotateUpperArm( ... ) function.
    //
    // Returns: The error in degrees.
    //
    // Remarks:
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double rotateUpperArm_CCW(double degrees) {
        boolean debug = false;
        double error = 0;

        // These are used for the integral and derivative terms
        // and use the WPILib Timer class.
        double current_time = 0.0;
        double dt = 0;

        // our PID constants (always need final tuning)
        double kp = 0.00001;
        double ki = 0.0002;
        double kd = 0.0000;
        double power = 0.0;

        // First time through, get the initial position. We compute
        // the number of counts associated with the degrees and
        // compute our target count. If we have our motor inversion
        // state correctly set we add the computed counts to the
        // present position to rotate the drive counter clockwise. We start
        // the motor within this block.
        if (upperRotate_init == 1) {
            upperCount = arm.getUpperFalconEncoder();

            // Must rotate the upper arm theta + phi. The argument
            // "degrees" is the sum of phi and theta
            upperTarget = upperCount - compute_UpperCounts(degrees);
            error = upperCount - upperTarget;
            if (debug == true) {
                System.out.printf("\nUpper initial count = %.3f\n", upperCount);
                System.out.printf("\nUpper rotation target = %.3f\n", upperTarget);
            }

            power = kp * error;
            // Here's the clamp
            if (power > 0.5)
                power = 0.5;
            if (power < 0.1)
                power = 0.1;

            // Get it started
            arm.setUpperMotor(-power);

            upperRotate_init = 0;
            upperLast_error = 0;
            upperError_sum = 0.0;
            upperError_rate = 0.0;
            upperLast_time = 0.0;

            upperRotation_complete = false;

        }

        if (upperRotation_complete == false) {

            // Set the motor in motion, wait a bit and then read the encoder
            // and compute the error.
            if (upperCount > upperTarget) {

                current_time = time.get();

                delay.delay_milliseconds(5.0);

                upperCount = arm.getUpperFalconEncoder();

                error = upperCount - upperTarget; // In this case should be positive

                if ((Math.abs(error) < upperDeadband) || (error < 0.0)) {
                    arm.setUpperMotor(0);
                    upperRotation_complete = true;
                    System.out.printf("\nUpper Target = %.3f\n", upperTarget);
                    System.out.printf("\nUpper Final count = %.3f  error = %.3f\n", upperCount, error);
                    return (computeUpperDegrees_fromCounts(error));
                }

                if (upperLast_time != 0.0)
                    dt = current_time - upperLast_time;

                if (dt > 0.0)
                    upperError_rate = (error - upperLast_error) / dt;

                System.out.printf("\ndt=%.3f\n", dt);

                // Start off integrating when within some percentage of the target
                // Note the use of the absolute value of the target for
                // determination of the error sum.
                if (error < Math.abs(0.5 * upperTarget))
                    upperError_sum += error * dt; // don't apply the integral term until closer to target

                power = kp * error + ki * upperError_sum + kd * upperError_rate;

                // Here's the clamp on the upper power
                if (power > 0.5)
                    power = 0.5;
                if (power < 0.1)
                    power = 0.1;

                arm.setUpperMotor(-power);

                if (debug == true) {
                    if (upperUpdate == 5) {
                        System.out.printf("\nUpper count = %.3f  Upper error = %.3f\n", upperCount, error);
                        System.out.printf("\nUpper error_sum = %.3f  error_rate = %.3f  dt = %.3f\n",
                                upperError_sum, upperError_rate, dt);
                        System.out.printf("\nUpper:kp*error = %.3f\n", kp * error);
                        System.out.printf("\nUpper:ki*error_sum = %.3f\n", ki * upperError_sum);
                        System.out.printf("\nUpper:power = %.3f\n", power);
                        upperUpdate = 0;
                    }
                    upperUpdate++;
                }

                // Record last time and last error
                // upperLast_time=time.get();
                // upperLast_error=error;
            } // if(upperCount<upperTarget)
              // Record last time and last error
            upperLast_time = time.get();
            upperLast_error = error;

        } // if(turn_motion_complete==false)

        return (computeUpperDegrees_fromCounts(upperLast_error));
        // return(upperLast_error);

    }

    /////////////////////////////////////////////////////////////////
    // Function: double rotateLowerArm(double degrees) {
    /////////////////////////////////////////////////////////////////
    //
    // Purpose: Rotates the lower arm the number of degrees
    // specified in the argument. "degrees" can be
    // positive or negative, the direction is
    // determined by the sign.
    //
    // Arguments:double degrees
    //
    // Returns: The error in degrees
    //
    // Remarks:
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double rotateLowerArm(double degrees) {

        double error = 0.0;

        if (lowerRotation_complete == false) {

            if (degrees > 0.0) {
                error = rotateLowerArm_CW(degrees);
            } else if (degrees < 0.0) {
                degrees *= -1.0;
                error = rotateLowerArm_CCW(degrees);
            }
        }

        return (error);

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
    // Remarks:
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double rotateLowerArm_CCW(double degrees) {
        boolean debug = false;
        double error = 0;

        // These are used for the integral and derivative terms
        // and use the WPILib Timer class.
        double current_time = 0.0;
        double dt = 0;

        // our PID constants (always need final tuning)
        double kp = 0.00002;
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

            // Must rotate the lower arm theta. In this case
            // (negative x coordinate), the target will be less
            // than the present count
            lowerTarget = lowerCount - compute_LowerCounts(degrees);
            error = lowerCount - lowerTarget;
            if (debug == true) {
                System.out.printf("\nLower initial count = %.3f\n", lowerCount);
                System.out.printf("\nLower rotation target = %.3f\n", lowerTarget);
            }

            power = kp * error;
            // Here's the clamp
            if (power > 0.5)
                power = 0.5;
            if (power < 0.1)
                power = 0.1;

            // Get it started
            arm.setLowerMotor(-power);
            // lowerCount=0;
            lowerRotate_init = 0;
            lowerLast_error = 0;
            lowerError_sum = 0.0;
            lowerError_rate = 0.0;
            lowerLast_time = 0.0;

        }

        if (lowerRotation_complete == false) {

            // Set the motor in motion, wait a bit and then read the encoder
            // and compute the error.
            if (lowerCount > lowerTarget) {

                current_time = time.get();

                delay.delay_milliseconds(5.0);

                lowerCount = arm.getLowerFalconEncoder();

                error = lowerCount - lowerTarget; // In this case should be positive

                if ((Math.abs(error) < lowerDeadband) || (error < 0.0)) {
                    arm.setLowerMotor(0);
                    lowerRotation_complete = true;
                    System.out.printf("\nLower: target = %.3f\n", lowerTarget);
                    System.out.printf("\nLower: final count = %.3f  final error = %.3f\n", lowerCount, error);
                    return (computeLowerDegrees_fromCounts(error));
                }

                if (upperLast_time != 0.0)
                    dt = current_time - upperLast_time;

                if (dt > 0.0) {
                    lowerError_rate = (error - lowerLast_error) / dt;
                }

                // Start off integrating when within some percentage of the target
                if (error < Math.abs(0.5 * lowerTarget))
                    lowerError_sum += error * dt; // don't apply the integral term until closer to target

                power = kp * error + ki * lowerError_sum + kd * lowerError_rate;
                // Here's the clamp
                if (power > 0.5)
                    power = 0.5;
                if (power < 0.1)
                    power = 0.1;

                arm.setLowerMotor(-power);

                if (debug == true) {
                    if (lowerUpdate == 5) {
                        System.out.printf("\nLower: count = %.3f  error = %.3f\n", lowerCount, error);
                        System.out.printf("\nLower: error_sum = %.3f  error_rate = %.3f  dt = %.3f", lowerError_sum,
                                lowerError_rate, dt);
                        System.out.printf("\nLower: kp*error = %.3f\n", kp * error);
                        System.out.printf("\nLower: ki*error_sum = %.3f\n", ki * lowerError_sum);
                        System.out.printf("\nLower: power = %.3f\n", power);
                        lowerUpdate = 0;
                    }
                    lowerUpdate++;
                }

                // Record last time and last error
                // lowerLast_time=time.get();
                // lowerLast_error=error;
            }
            // Record last time and last error
            lowerLast_time = time.get();
            lowerLast_error = error;

        } // if(turn_motion_complete==false)

        return (computeLowerDegrees_fromCounts(lowerLast_error));

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
    // Remarks:
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double rotateLowerArm_CW(double degrees) {
        boolean debug = false;
        double error = 0;

        // These are used for the integral and derivative terms
        // and use the WPILib Timer class.
        double current_time = 0.0;
        double dt = 0;

        // our PID constants (always need final tuning)
        double kp = 0.00002;
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
            if (power < 0.1)
                power = 0.1;

            // Get it started
            arm.setLowerMotor(power);
            // lowerCount=0;
            lowerRotate_init = 0;
            lowerLast_error = 0;
            lowerError_sum = 0.0;
            lowerError_rate = 0.0;
            lowerLast_time = 0.0;

        }

        if (lowerRotation_complete == false) {

            // Set the motor in motion, wait a bit and then read the encoder
            // and compute the error.
            if (lowerCount < lowerTarget) {

                current_time = time.get();

                delay.delay_milliseconds(5.0);

                lowerCount = arm.getLowerFalconEncoder();

                error = lowerTarget - lowerCount; // In this case should be positive

                if ((Math.abs(error) < lowerDeadband) || (error < 0.0)) {
                    arm.setLowerMotor(0);
                    lowerRotation_complete = true;
                    System.out.printf("\nLower: target = %.3f\n", lowerTarget);
                    System.out.printf("\nLower: final count = %.3f  final error = %.3f\n", lowerCount, error);
                    return (computeLowerDegrees_fromCounts(error));
                }

                if (upperLast_time != 0.0)
                    dt = current_time - upperLast_time;

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
                if (power < 0.1)
                    power = 0.1;

                arm.setLowerMotor(power);

                if (debug == true) {
                    if (lowerUpdate == 5) {
                        System.out.printf("\nLower: count = %.3f  error = %.3f\n", lowerCount, error);
                        System.out.printf("\nLower: error_sum = %.3f  error_rate = %.3f  dt = %.3f", lowerError_sum,
                                lowerError_rate, dt);
                        System.out.printf("\nLower: kp*error = %.3f\n", kp * error);
                        System.out.printf("\nLower: ki*error_sum = %.3f\n", ki * lowerError_sum);
                        System.out.printf("\nLower: power = %.3f\n", power);
                        lowerUpdate = 0;
                    }
                    lowerUpdate++;
                }

                // Record last time and last error
                // lowerLast_time=time.get();
                // lowerLast_error=error;
            }
            // Record last time and last error
            lowerLast_time = time.get();
            lowerLast_error = error;

        } // if(turn_motion_complete==false)

        return (computeLowerDegrees_fromCounts(lowerLast_error));

    }

    /**************************************************************/
    // The following two functions are legacy functions that
    // were used during development. They will eventually be
    // discarded
    /**************************************************************/

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

    double rotateUpperArm_original(double degrees) {
        boolean debug = true;
        double error = 0;

        // These are used for the integral and derivative terms
        // and use the WPILib Timer class.
        double current_time = 0.0;
        double dt = 0;

        // our PID constants (always need final tuning)
        double kp = 0.000005;
        double ki = 0.00002;
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

                if (Math.abs(error) < upperDeadband) {
                    arm.setUpperMotor(0);

                    upperRotation_complete = true;
                    System.out.printf("\nUpper Target = %.3f\n", upperTarget);
                    System.out.printf("\nUpper Final count = %.3f  error = %.3f\n", upperCount, error);
                    return (computeUpperDegrees_fromCounts(error));
                }

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

                arm.setUpperMotor(power);

                if (debug == true) {
                    if (upperUpdate == 5) {
                        System.out.printf("\nUpper count = %.3f  Upper error = %.3f\n", upperCount, error);
                        System.out.printf("\nUpper error_sum = %.3f  error_rate = %.3f  dt = %.3f\n", upperError_sum,
                                upperError_rate, dt);
                        System.out.printf("\nUpper:kp*error = %.3f\n", kp * error);
                        System.out.printf("\nUpper:ki*error_sum = %.3f\n", ki * upperError_sum);
                        System.out.printf("\nUpper:power = %.3f\n", power);
                        upperUpdate = 0;
                    }
                    upperUpdate++;
                }

                // Record last time and last error
                upperLast_time = time.get();
                upperLast_error = error;
            } // if(upperCount<upperTarget)

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
    // Remarks:
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////

    double rotateLowerArm_original(double degrees) {
        boolean debug = true;
        double error = 0;

        // These are used for the integral and derivative terms
        // and use the WPILib Timer class.
        double current_time = 0.0;
        double dt = 0;

        // our PID constants (always need final tuning)
        double kp = 0.000005;
        double ki = 0.00002;
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
            // if(power<0.05)power=0.05;

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

                if ((Math.abs(error) < lowerDeadband) || (error < 0.0)) {
                    arm.setLowerMotor(0);

                    lowerRotation_complete = true;
                    System.out.printf("\nLower: target = %.3f\n", lowerTarget);
                    System.out.printf("\nLower: final count = %.3f  final error = %.3f\n", lowerCount, error);
                    return (computeLowerDegrees_fromCounts(error));
                }

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

                arm.setLowerMotor(power);

                if (debug == true) {
                    if (lowerUpdate == 5) {
                        System.out.printf("\nLower: count = %.3f  error = %.3f\n", lowerCount, error);
                        System.out.printf("\nLower: error_sum = %.3f  error_rate = %.3f  dt = %.3f", lowerError_sum,
                                lowerError_rate, dt);
                        System.out.printf("\nLower: kp*error = %.3f\n", kp * error);
                        System.out.printf("\nLower: ki*error_sum = %.3f\n", ki * lowerError_sum);
                        System.out.printf("\nLower: power = %.3f\n", power);
                        lowerUpdate = 0;
                    }
                    lowerUpdate++;
                }

                // Record last time and last error
                lowerLast_time = time.get();
                lowerLast_error = error;
            }

        } // if(turn_motion_complete==false)

        return (computeLowerDegrees_fromCounts(lowerLast_error));

    }

}
