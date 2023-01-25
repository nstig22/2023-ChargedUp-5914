/////////////////////////////////////////////////////////////////
//  File:   SwerveDriveThread.java
/////////////////////////////////////////////////////////////////
//
//  Purpose:  Defines a runnable execution thread that could
//  be used in autonomous.  Different functions for movement
//  are defined within this thread class the reason being that
//  there is no repeat access to these functions as is normally
//  done in the main execution thread of a robotics application.
//  Use is made of while() loops in that the respective functions
//  are called only once.
//
//  Inception date: 1/12/2023
//
//  Remarks: Remaining is to put time limits within the while
//  loops of these functions with specific return codes and
//  actions that will prevent "hang" if someting unexpected
//  happens within the while() blocks.
//
//  Note also that the thread allocates a private class
//  instantiation of SwerveDrive.  This is to prevent
//  clashes between the instantiation in Robot.java and
//  this one.  The instantiation of SwerveDrive in 
//  Robot.java has been moved to telopInit() and the 
//  Robot.java version is also declared private to
//  Robot.java.  Don't exactly know how this will work out.
//
//  1/18/2023:  Functions are working.  Note: too many printouts
//  will make operation inconsistant.
//
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;

class SwerveDriveThread implements Runnable {

    private SwerveDrive drive;
    private ADXRS450_Gyro driveGyro;

    // Timing parame int
    volatile int isactive;

    private long thread_start_time = 0;
    private long thread_end_time = 0;
    private double thread_elapsed_time = 0.0;
    private int updateCounter = 0;

    // private double duration=3000; // duration in msec

    String name;
    Thread t;
    Runtime r = Runtime.getRuntime();
    private Delay delay;

    // Constructor
    SwerveDriveThread(String threadname) {

        drive = new SwerveDrive();
        driveGyro = new ADXRS450_Gyro();

        name = threadname;
        t = new Thread(this, name);

        System.out.printf("New thread: %s", name);

        delay = new Delay();

        driveGyro.calibrate();

        driveGyro.reset();

        double angle = driveGyro.getAngle();

        System.out.printf("\ndegrees = %.3f\n", angle);

        // Start the timer
        // start_time=System.nanoTime();
        isactive = 1;
        Robot.thread_is_active = true;
        t.start(); // Start the thread
    }

    // Here is the execution of the thread
    public void run() {

        // Two ways to go here - we could use an 'if' statement. That would probably
        // be safer in that we only intend to run this thread once. Using a while()
        // is dangerous in that we could be locked into an infinite loop if the
        // thread fails to terminate. 'isactive' is set to '1' in the thread
        // constructor and zeroed if we successfully interrupt the thread.
        while (isactive == 1) {

            turnRight(90, 0.2);

            /*
             * driveFwd(24.0);
             * 
             * delay.delay_milliseconds(100.0);
             * 
             * turnRight(90.0);
             * 
             * delay.delay_milliseconds(100.0);
             * 
             * driveFwd(34.5);
             * 
             * delay.delay_milliseconds(100.0);
             * 
             * return2Zero();
             * 
             * delay.delay_milliseconds(100.0);
             * 
             * driveFwd(24.0);
             * 
             * delay.delay_milliseconds(100.0);
             * 
             * turnLeft(90.0);
             * 
             * delay.delay_milliseconds(100.0);
             * 
             * driveFwd(34.5);
             * 
             * delay.delay_milliseconds(100.0);
             * 
             * return2Zero();
             * 
             * delay.delay_milliseconds(100.0);
             * 
             * driveFwd(24.0);
             * 
             * //turnLeft(180.0);
             * 
             * //delay.delay_milliseconds(100.0);
             */

            /*
             * delay.delay_milliseconds(100.0);
             * 
             * turnLeft(45.0);
             * 
             * delay.delay_milliseconds(100.0);
             * 
             * driveReverse(24.0);
             * 
             * delay.delay_milliseconds(100.0);
             * 
             * turnRight(90.0);
             * 
             * delay.delay_milliseconds(100.0);
             * 
             * return2Zero();
             */

            // interrupt (terminate) the thread
            t.interrupt();

            // Wait for the thread to complete. Use of the
            // join function will let us know when the thread
            // has completed execution.
            try {
                t.join();
            } catch (InterruptedException e) {

                System.out.printf("%s Interrupted", name);

                // kill the motors
                stop();

                Robot.thread_is_active = false;

                isactive = 0; // exit the while() loop

            }

            // Output time to reach this point. Should be stability
            // time plus duration.
            thread_end_time = System.nanoTime();
            thread_elapsed_time = thread_end_time - thread_start_time;
            thread_elapsed_time *= 1e-6; // convert to milliseconds
            System.out.printf("Thread Elapsed time = %.3 msec", thread_elapsed_time);

            // Ok, lets create an escape hatch in the event that the
            // thread

        } // while(isactive==1)

        // brute force release the memory
        r.gc();
        return;
    }

    /////////////////////////////////////////////////////////////////
    // Function: double turnRight(double degrees)
    /////////////////////////////////////////////////////////////////
    //
    // Purpose: Rotates the swerve drive in a clockwise direction
    //
    // Arguments:double degrees
    //
    // Returns: The error in counts as double
    //
    // Remarks: Rotating right (clockwise) implies
    // increasing counts. This version is used within the thread
    // and incorporates a while() loop. Unlike the functions
    // used in teleOP() this function gets called once.
    //
    // 1/18/23: A thought: when this function is called,
    // it is executed with one pass. There is no need for
    // an "init" variable.
    //
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double turnRight(double degrees) {
        boolean debug = false;

        double count = 0;
        double target = 0;

        long start_time = 0;
        long end_time = 0;
        double elapsed_time = 0.0;
        double error;

        // Determine position, compute target counts
        count = drive.falcon_turn.getSelectedSensorPosition(0);
        target = count + drive.compute_countsDegrees(degrees);

        if (debug == true) {
            System.out.printf("Right Turn target = %.3f  count = %.3f\n", target, count);
        }

        start_time = System.nanoTime();

        while (count < (target + drive.turn_deadband)) {
            drive.falcon_turn.set(ControlMode.PercentOutput, 0.2);

            delay.delay_milliseconds(20.0);

            count = drive.falcon_turn.getSelectedSensorPosition(0);

            error = target - count; // In this case should be positive

            if (debug == true) {
                updateCounter++;
                if (updateCounter == 10) {
                    System.out.printf("count = %.3f\n", count);
                    System.out.printf("error = %.3f\n", error);
                    updateCounter = 0;
                }
            }

            // Escape hatch
            end_time = System.nanoTime();
            elapsed_time = end_time - start_time;
            elapsed_time *= 1e-6; // convert to milliseconds
            if (elapsed_time > 2000.0) {
                System.out.printf("Elapsed Time within turnRight() has exceeded limits.");
                stop();
                break;
            }

        }

        // Are we within the deadband for the turn? Have we overshot?
        // If either of these are true, stop the motor, read the position,
        // and return the error (turn_target-count)
        if ((Math.abs(target - count) < drive.turn_deadband) || (count > target)) {
            stop();
            count = drive.falcon_turn.getSelectedSensorPosition(0);
            updateCounter = 0;
            return (target - count);
        }

        // If we have reached this point it would indicate an error of some sort. Either
        // we exited the while loop because of a hang or we failed to converge within
        // the
        // deadband.
        return (999.999);

    }

    double turnRight_ABS(double degrees) {
        boolean debug = false;

        double count = 0;
        double target = 0;

        long start_time = 0;
        long end_time = 0;
        double elapsed_time = 0.0;
        double error;

        // Determine position, compute target counts
        count = drive.enc_abs.getAbsolutePosition();
        target = count + drive.compute_countsDegrees_ABS(degrees);

        if (debug == true) {
            System.out.printf("Right Turn target = %.3f  count = %.3f\n", target, count);
        }

        start_time = System.nanoTime();

        while (count < (target + drive.turn_deadband)) {
            drive.falcon_turn.set(ControlMode.PercentOutput, 0.2);

            delay.delay_milliseconds(20.0);

            count = drive.enc_abs.getAbsolutePosition();

            error = target - count; // In this case should be positive

            if (debug == true) {
                updateCounter++;
                if (updateCounter == 10) {
                    System.out.printf("count = %.3f\n", count);
                    System.out.printf("error = %.3f\n", error);
                    updateCounter = 0;
                }
            }

            // Escape hatch
            end_time = System.nanoTime();
            elapsed_time = end_time - start_time;
            elapsed_time *= 1e-6; // convert to milliseconds
            if (elapsed_time > 2000.0) {
                System.out.printf("Elapsed Time within turnRight() has exceeded limits.");
                stop();
                break;
            }

        }

        // Are we within the deadband for the turn? Have we overshot?
        // If either of these are true, stop the motor, read the position,
        // and return the error (turn_target-count)
        if ((Math.abs(target - count) < drive.turn_deadband) || (count > target)) {
            stop();
            count = drive.enc_abs.getAbsolutePosition();
            updateCounter = 0;
            return (target - count);
        }

        // If we have reached this point it would indicate an error of some sort. Either
        // we exited the while loop because of a hang or we failed to converge within
        // the
        // deadband.
        return (999.999);

    }

    // Function to turn right using the gyro
    void turnRight(double degrees, double speed) {

        boolean debug = true;
        long start_time = 0;
        long end_time = 0;
        double elapsed_time = 0.0;
        double angle = driveGyro.getAngle();
        int count = 0;

        driveGyro.reset();

        if (debug == true) {
            System.out.printf("\nangle = %.3f", driveGyro.getAngle());
        }

        turnRight(45);

        while (angle < 90) {
            drive.falcon_drive.set(ControlMode.PercentOutput, 0.1);

            delay.delay_milliseconds(20.0);

            angle = driveGyro.getAngle();

            delay.delay_milliseconds(20.0);

            if (debug == true) {
                if (count == 5) {
                    System.out.printf("\nangle = %.3f\n", angle);
                    count = 0;
                }
            }

            count++;

            // Escape hatch
            end_time = System.nanoTime();
            elapsed_time = end_time - start_time;
            elapsed_time *= 1e-6; // convert to milliseconds
            if (elapsed_time > 2000.0) {
                System.out.printf("Elapsed Time within turnRight() has exceeded limits.");
                stop();
                break;
            }
        }

        delay.delay_milliseconds(200.0);

        return2Zero();

    }

    // Function to turn right using the gyro
    void turnRight_ABS(double degrees, double speed) {

        boolean debug = true;
        long start_time = 0;
        long end_time = 0;
        double elapsed_time = 0.0;
        double angle = driveGyro.getAngle();
        int count = 0;

        driveGyro.reset();

        if (debug == true) {
            System.out.printf("\nangle = %.3f", driveGyro.getAngle());
        }

        turnRight(45);

        while (angle < 90) {
            drive.falcon_drive.set(ControlMode.PercentOutput, 0.1);

            delay.delay_milliseconds(20.0);

            angle = driveGyro.getAngle();

            delay.delay_milliseconds(20.0);

            if (debug == true) {
                if (count == 5) {
                    System.out.printf("\nangle = %.3f\n", angle);
                    count = 0;
                }
            }

            count++;

            // Escape hatch
            end_time = System.nanoTime();
            elapsed_time = end_time - start_time;
            elapsed_time *= 1e-6; // convert to milliseconds
            if (elapsed_time > 2000.0) {
                System.out.printf("Elapsed Time within turnRight() has exceeded limits.");
                stop();
                break;
            }
        }

        delay.delay_milliseconds(200.0);

        return2Zero();

    }

    double turnLeft(double degrees) {
        boolean debug = false;

        double count = 0;
        double target = 0;
        long start_time = 0;
        long end_time = 0;
        double elapsed_time = 0.0;
        double error;

        // Get the initial position, compute the number of counts

        count = drive.falcon_turn.getSelectedSensorPosition(0);
        target = count - drive.compute_countsDegrees(degrees);

        updateCounter = 0;

        start_time = System.nanoTime();

        if (debug == true) {
            System.out.printf("Left Turn target = %.3f  count =%.3f\n", target, count);
        }

        while (count > target) {
            drive.falcon_turn.set(ControlMode.PercentOutput, -0.2);

            delay.delay_milliseconds(20.0);

            count = drive.falcon_turn.getSelectedSensorPosition(0);

            error = target - count;

            if (debug == true) {
                updateCounter++;
                if (updateCounter == 10) {
                    System.out.printf("count = %.3f\n", count);
                    System.out.printf("error = %.3f\n", error);
                    updateCounter = 0;
                }
            }

            // Escape hatch
            end_time = System.nanoTime();
            elapsed_time = end_time - start_time;
            elapsed_time *= 1e-6; // convert to milliseconds
            if (elapsed_time > 2000.0) {
                System.out.printf("Elapsed Time within turnRight() has exceeded limits.");
                stop();
                break;
            }

        }

        // Are we within the deadband for the turn? Have we overshot?
        // If either of these are true, stop the motor, read the position,
        // and return the error (turn_target-count)
        if ((Math.abs(target - count) < drive.turn_deadband) || (count < target)) {
            stop();
            count = drive.falcon_turn.getSelectedSensorPosition(0);
            updateCounter = 0;
            return (target - count);
        }

        // If we have reached this point it would indicate an error of some sort. Either
        // we exited the while loop because of a hang or we failed to converge within
        // the
        // deadband. Note that the return value here is negative - just so we might
        // be able to tell which function timed out.
        return (-999.999);

    }

    double turnLeft_ABS(double degrees) {
        boolean debug = false;

        double count = 0;
        double target = 0;
        long start_time = 0;
        long end_time = 0;
        double elapsed_time = 0.0;
        double error;

        // Get the initial position, compute the number of counts

        count = drive.enc_abs.getAbsolutePosition();
        target = count - drive.compute_countsDegrees_ABS(degrees);

        updateCounter = 0;

        start_time = System.nanoTime();

        if (debug == true) {
            System.out.printf("Left Turn target = %.3f  count =%.3f\n", target, count);
        }

        while (count > target) {
            drive.falcon_turn.set(ControlMode.PercentOutput, -0.2);

            delay.delay_milliseconds(20.0);

            count = drive.enc_abs.getAbsolutePosition();

            error = target - count;

            if (debug == true) {
                updateCounter++;
                if (updateCounter == 10) {
                    System.out.printf("count = %.3f\n", count);
                    System.out.printf("error = %.3f\n", error);
                    updateCounter = 0;
                }
            }

            // Escape hatch
            end_time = System.nanoTime();
            elapsed_time = end_time - start_time;
            elapsed_time *= 1e-6; // convert to milliseconds
            if (elapsed_time > 2000.0) {
                System.out.printf("Elapsed Time within turnRight() has exceeded limits.");
                stop();
                break;
            }

        } // while( ... )

        // Are we within the deadband for the turn? Have we overshot?
        // If either of these are true, stop the motor, read the position,
        // and return the error (turn_target-count)
        if ((Math.abs(target - count) < drive.turn_deadband) || (count < target)) {
            stop();
            count = drive.enc_abs.getAbsolutePosition();
            return (target - count);
        }

        // If we have reached this point it would indicate an error of some sort. Either
        // we exited the while loop because of a hang or we failed to converge within
        // the
        // deadband. Note that the return value here is negative - just so we might
        // be able to tell which function timed out.
        return (-999.999);

    }

    /////////////////////////////////////////////////////////////////
    // Function: int return2Zero()
    /////////////////////////////////////////////////////////////////
    //
    // Purpose: Assuming we start the robot with the wheels
    // pointing straight forward, on init, we set the encoder
    // count to zero. All turns from that point on are
    // referenced to the current position with 'zero' being
    // the home position. So, our target in counts is zero.
    // If the current measured position is positive, we wish
    // to rotate in a direction to reduce the measured count
    // to zero. Likewise, if negative we wish to increase the
    // measured count to zero.
    //
    // Arguments:void
    //
    // Returns: 0
    //
    // Remarks:TBD: This function is the version called within the
    // thread. It is not subject to watchdog issues as it
    // operates in parallel (time sliced) with the main thread.
    // The operations turnRight() and turnLeft(0) values are
    // examined to determine if these functions have timed out
    // within their respective while() loops.
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double return2Zero() {
        double count = 0;
        double degrees;
        double return_val;

        // Get the position
        count = drive.falcon_turn.getSelectedSensorPosition(0);
        delay.delay_milliseconds(20);

        System.out.printf("\nInitial Count = %.3f\n", count);

        degrees = count / drive.counts_perDegree();

        System.out.printf("\nDegrees=%.3f\n", degrees);

        if (count > 0.0) {

            return_val = turnLeft(Math.abs(degrees));
            if (return_val == 999.999) {
                System.out.printf("Timeout: Return of turnLeft() = %.3lf", return_val);
            }

        } else if (count < 0.0) {

            return_val = turnRight(Math.abs(degrees));
            if (return_val == -999.999) {
                System.out.printf("Timeout: Return of turnRight() = %.3lf", return_val);
            }

        }
        return (0);

    }

    double return2Zero_ABS() {
        double count = 0;
        double degrees;
        double return_val;

        // Get the position
        count = drive.enc_abs.getAbsolutePosition();
        delay.delay_milliseconds(20);

        System.out.printf("\nInitial Count = %.3f\n", count);

        degrees = count / drive.counts_perDegree_ABS();

        System.out.printf("\nDegrees=%.3f\n", degrees);

        if (count > 0.0) {

            return_val = turnLeft_ABS(Math.abs(degrees));
            if (return_val == 999.999) {
                System.out.printf("Timeout: Return of turnLeft() = %.3lf", return_val);
            }

        } else if (count < 0.0) {

            return_val = turnRight_ABS(Math.abs(degrees));
            if (return_val == -999.999) {
                System.out.printf("Timeout: Return of turnRight_ABS() = %.3lf", return_val);
            }

        }
        return (0);

    }

    double driveFwd(double inches) {
        boolean debug = false;
        double count = 0;
        double target = 0;

        long start_time = 0;
        long end_time = 0;
        double elapsed_time = 0.0;
        double error;

        // Get the initial position, compute the number of counts
        count = drive.falcon_drive.getSelectedSensorPosition(0);
        target = count + drive.compute_countsDistance(inches);
        updateCounter = 0;

        start_time = System.nanoTime();

        if (debug == true) {
            System.out.printf("Fwd Drive target = %.3f count = %.3f\n", drive.drive_target, drive.count);
        }

        while (count < (target + drive.drive_deadband)) {
            drive.falcon_drive.set(ControlMode.PercentOutput, 0.2);

            delay.delay_milliseconds(20.0);

            count = drive.falcon_drive.getSelectedSensorPosition(0);

            error = target - count; // In this case should be positive
            if (debug == true) {
                updateCounter++;
                if (updateCounter == 10) {
                    System.out.printf("count = %.3f\n", count);
                    System.out.printf("error = %.3f\n", error);
                    updateCounter = 0;
                }
            }

            // Escape hatch
            end_time = System.nanoTime();
            elapsed_time = end_time - start_time;
            elapsed_time *= 1e-6; // convert to milliseconds
            if (elapsed_time > 2000.0) {
                System.out.printf("Elapsed Time within driveFwd() has exceeded limits.");
                stop();
                break;
            }

        }

        // Are we within the deadband for the turn? Have we overshot?
        // If either of these are true, stop the motor, read the position,
        // and return the error (turn_target-count)
        if ((Math.abs(target - count) < drive.drive_deadband) || (count > target)) {
            stop();
            count = drive.falcon_drive.getSelectedSensorPosition(0);
            delay.delay_milliseconds(20);
            updateCounter = 0;
            return (target - count);
        }

        // If we have reached this point it would indicate an error of some sort. Either
        // we exited the while loop because of a hang or we failed to converge within
        // the
        // deadband.
        return (999.999);

    }

    double driveReverse(double inches) {
        boolean debug = false;

        double count = 0;
        double target = 0;
        long start_time = 0;
        long end_time = 0;
        double elapsed_time = 0.0;
        double error;

        // Get the initial position, compute the number of counts
        count = drive.falcon_drive.getSelectedSensorPosition(0);
        target = count - drive.compute_countsDistance(inches);

        if (debug == true) {
            System.out.printf("RevDrive target = %.3f count = %.3f\n", target, count);
        }

        updateCounter = 0;

        start_time = System.nanoTime();

        while (count > target) {
            drive.falcon_drive.set(ControlMode.PercentOutput, -0.2);

            delay.delay_milliseconds(20.0);

            count = drive.falcon_drive.getSelectedSensorPosition(0);

            error = target - count;

            if (debug == true) {
                updateCounter++;
                if (updateCounter == 10) {
                    System.out.printf("count = %.3f\n", drive.count);
                    System.out.printf("error = %.3f\n", error);
                    updateCounter = 0;
                }
            }

            // Escape hatch
            end_time = System.nanoTime();
            elapsed_time = end_time - start_time;
            elapsed_time *= 1e-6; // convert to milliseconds
            if (elapsed_time > 2000.0) {
                System.out.printf("Elapsed Time within turnRight() has exceeded limits.");
                stop();
                break;
            }

        }

        // Are we within the deadband for the turn? Have we overshot?
        // If either of these are true, stop the motor, read the position,
        // and return the error (turn_target-count)
        if ((Math.abs(target - count) < drive.drive_deadband) || (count < target)) {
            stop();
            count = drive.falcon_drive.getSelectedSensorPosition(0);
            updateCounter = 0;
            return (target - count);
        }

        // If we have reached this point it would indicate an error of some sort. Either
        // we exited the while loop because of a hang or we failed to converge within
        // the
        // deadband. Note that the return value here is negative - just so we might
        // be able to tell which function timed out.
        return (-999.999);

    }

    /////////////////////////////////////////////////////////////////
    // Function: void stop()
    /////////////////////////////////////////////////////////////////
    //
    // Purpose: kills the motors once the thread is terminated.
    //
    // Arguments:
    //
    // Returns:
    //
    // Remarks:
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    private void stop() {
        drive.falcon_drive.set(ControlMode.PercentOutput, 0.0);
        drive.falcon_turn.set(ControlMode.PercentOutput, 0.0);
    }
}