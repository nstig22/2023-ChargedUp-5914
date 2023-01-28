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
//  1/28/2023:  Turning the wheels is governed by the absolute
//  encoder.  Moving forward it would be beneficial to align the
//  turning wheel with the absolute encoder.  There may be another
//  way in software using the "offset" function - not sure about
//  this.
//
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

package frc.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;


class SwerveDriveThread implements Runnable {

    private SwerveDrive drive;
    private ADXRS450_Gyro driveGyro;
    
    //  Timing parame int  
    volatile int isactive;

    private long thread_start_time=0;
    private long thread_end_time=0;
    private double thread_elapsed_time=0.0;
    private int updateCounter=0;
    
   // private double duration=3000;  //  duration in msec
  	
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
        
        System.out.printf("New thread: %s",name);
    
		delay = new Delay();

        driveGyro.calibrate();

        driveGyro.reset();

        double angle = driveGyro.getAngle();

        System.out.printf("\nInitial Gyro angle = %.3f degrees\n", angle);

        //  Start the timer
		thread_start_time=System.nanoTime();
        isactive=1;
        Robot.thread_is_active=true;
		t.start(); // Start the thread
	}

    //  Here is the execution of the thread
	public void run() {

        double initial_count;
        double count;
        double error=0;

        drive.falcon_drive.set(ControlMode.PercentOutput, 0.1);

               		
       //  Two ways to go here - we could use an 'if' statement.  That would probably
       //  be safer in that we only intend to run this thread once.  Using a while()
       //  is dangerous in that we could be locked into an infinite loop if the
       //  thread fails to terminate.  'isactive' is set to '1' in the thread
       //  constructor and zeroed if we successfully interrupt the thread.
       
       while(isactive==1) {

       
            initial_count=drive.enc_abs.getAbsolutePosition();

            delay.delay_milliseconds(20);

            System.out.printf("\nInitial Absolute Encoder Value = %.3f\n",initial_count);

            //error=turn2Position_ABS(190.0);

            delay.delay_milliseconds(20);

            //turnRight(90.0, 20.0,0.1);

            error=turnLeft(90.0, 45.0,0.1);

            System.out.printf("\nLeft turn Error = %.3f degrees\n",error);

            count=drive.enc_abs.getAbsolutePosition();

            System.out.printf("\nAbsolute Encoder Value = %.3f\n",count);

            //drive.falcon_drive.set(ControlMode.PercentOutput, 0.1);

            //delay.delay_milliseconds(2000);

            //error=turn2Position_ABS(0.0);

            //delay.delay_milliseconds(20);

            //System.out.printf("\nError = %.3f degrees\n",error);

            //delay.delay_milliseconds(20.0);
   
                        
            //  interrupt (terminate)  the thread 
            t.interrupt();
            
                                
            // Wait for the thread to complete.  Use of the
            // join function will let us know when the thread
            // has completed execution.
            try {
                t.join();
            } catch (InterruptedException e) {
                
                System.out.printf("%s Interrupted",name);

                //  kill the motors
                stop();

                Robot.thread_is_active=false;
                
                isactive=0; //  exit the while() loop
                
            }
                    
	
        }  //  while(isactive==1)

        //  Output time to reach this point.  Should be stability
        //  time plus duration.
        thread_end_time=System.nanoTime();
        thread_elapsed_time=thread_end_time-thread_start_time;
        thread_elapsed_time*=1e-6;  //  convert to milliseconds
        System.out.printf("Thread Elapsed time = %.3 msec",thread_elapsed_time);
        

        //  brute force release the memory
        r.gc();
        return;
    }  




    /////////////////////////////////////////////////////////////////
    //  Function: double turn2Position_ABS(double degrees)
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Orients the turning wheel to compass points (0-360)
    //
    //  Arguments:Accepts a double representing the target azuimuth
    //  angle.
    //
    //  Returns: The error in degrees as double
    //
    //  Remarks:The fact that no rollover is accounted for creates
    //  some complexity when crossing through 0/360.  This function
    //  determines the shortest path to achieve the target.  For
    //  example reaching 135 degrees from a position of 350 would
    //  best be accomplished by a clockwise movement of 145 degrees
    //  verses a counterclockwise movement of 225 degrees.
    //
    //  Currently the time limit to complete the turn is 2 seconds.
    //  This protects against getting locked into an infinite while()
    //  loop.
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double turn2Position_ABS(double degrees)
    {
        boolean debug = true;

        boolean time_exceeded = false;

        double count=0;
        double position=0;
        double target=0;

        long start_time=0;
        long end_time=0;
        double elapsed_time=0.0;
        double error=0.0;

        //  Determine position, compute target counts
        count=drive.enc_abs.getAbsolutePosition(); 
        //  convert to degrees
        position=count*360.0;  
        target=degrees/360.0;  //  abs encoder target

        if(debug==true)  {
            System.out.printf("\nABS Current Position = %.3f  degrees\n",position);
            System.out.printf("\nABS Current position in encoder units = %.3f\n",count);
            System.out.printf("\nABS Target in encoder units = %.3f\n",target);
        }
       

        start_time=System.nanoTime(); 

        //  Have to treat the case of passing through 360 degrees
        //  There are two ways to get to the target value which will
        //  be a number between 0.0 and 1.0.  Assume for argument sake
        //  that we are currently at 280 degrees and want to reposition
        //  at 20 degrees.  If we rotate clockwise the delta is
        //  80 + 20 = 100 degrees.  If we rotate counterclockwise the
        //  delta is 280 - 20 or 260 degrees.  So to determine the 
        //  direction of movement there are two cases distinguished by
        //  whether the difference in encoder value is greater or less
        //  than 0.5.

        //  First case:  the target is less than the present position
        if((target-count)<0.0)  {
            //  Next, how big is the difference?
            //  In this case we want to rotate clockwise
            if(Math.abs(target-count)>0.5)  {
                while(Math.abs(target-count)>drive.abs_enc_deadband) {

                    drive.falcon_turn.set(ControlMode.PercentOutput, 0.2);

                    delay.delay_milliseconds(20.0);
            
                    count=drive.enc_abs.getAbsolutePosition();    
                    
                    end_time=System.nanoTime();
                    elapsed_time=end_time-start_time;
                    elapsed_time*=1e-6;  //  convert to milliseconds
                    if(elapsed_time>2000.0)  {
                        System.out.printf("Elapsed Time within turnRight() has exceeded limits.");
                        time_exceeded=true;
                        stop();
                        break;
                    }

                }           

            }  else  {
                while(Math.abs(target-count)>drive.abs_enc_deadband) {

                    drive.falcon_turn.set(ControlMode.PercentOutput, -.2);

                    delay.delay_milliseconds(20.0);
            
                    count=drive.enc_abs.getAbsolutePosition();    
                    
                    end_time=System.nanoTime();
                    elapsed_time=end_time-start_time;
                    elapsed_time*=1e-6;  //  convert to milliseconds
                    if(elapsed_time>2000.0)  {
                        System.out.printf("Elapsed Time within turnRight() has exceeded limits.");
                        time_exceeded=true;
                        stop();
                        break;
                    }

                } 

            }  

            //  Stop the turn motor, compute the error
            stop();
            error=(target-count)*360.0;
            
            //  Next situation:  The target is greater than the initial count.  Normally
            //  we would travel clockwise - unless of course, it's quicker to got the other
            //  way
        }
        if((target-count)>0.0)  {

            if(Math.abs(target-count)>0.5)  {
                while(Math.abs(target-count)>drive.abs_enc_deadband) {

                    drive.falcon_turn.set(ControlMode.PercentOutput, -0.2);

                    delay.delay_milliseconds(20.0);
            
                    count=drive.enc_abs.getAbsolutePosition();    
                    
                    end_time=System.nanoTime();
                    elapsed_time=end_time-start_time;
                    elapsed_time*=1e-6;  //  convert to milliseconds
                    if(elapsed_time>2000.0)  {
                        System.out.printf("Elapsed Time within turnRight() has exceeded limits.");
                        time_exceeded=true;
                        stop();
                        break;
                    }

                }           

            }  else  {
                while(Math.abs(target-count)>drive.abs_enc_deadband) {

                    drive.falcon_turn.set(ControlMode.PercentOutput, .2);

                    delay.delay_milliseconds(20.0);
            
                    count=drive.enc_abs.getAbsolutePosition();    
                    
                    end_time=System.nanoTime();
                    elapsed_time=end_time-start_time;
                    elapsed_time*=1e-6;  //  convert to milliseconds
                    if(elapsed_time>2000.0)  {
                        System.out.printf("Elapsed Time within turnRight() has exceeded limits.");
                        time_exceeded=true;
                        stop();
                        break;
                    }

                } 

            }
            stop();
            error=(target-count)*360.0;
            
        }

        if(time_exceeded==true)return(-999.999);
        else  return(error);
          

    }


    /////////////////////////////////////////////////////////////////
    //  Function: turnRight(double degrees, double wheel_angle,double speed)  {
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Using the ADXRS450_Gyro gyroscope and the
    //  previous turning wheel function we turn the robot assembly
    //  to the right until the gyro reads degrees.  The flow
    //  is to turn the turning wheel to "wheel_angle" and drive the unit
    //  forward until the target "degrees" is reached
    //
    //  Arguments:double degrees: The desired outcome of the gyro
    //            relative to the starting position.
    //            double wheel_angle:  The angle to which the turning
    //            wheel is turned.  Higher values will make for 
    //            sharper turns.
    //            double speed:  the setting for the drive motor.
    //
    //  Returns:  The error value (target-actual).
    //
    //  Remarks:
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double turnRight(double degrees, double wheel_angle,double speed)  {

        boolean debug=true;
        
        double angle = driveGyro.getAngle();  //  What is our current heading?
        double target_angle;
        int count = 0;
        double wheel_position;

        //  Escape hatch
        long start_time=0;
        long end_time=0;
        double elapsed_time=0.0;

        double error=0;

        start_time=System.nanoTime();

        target_angle=angle + degrees;

        if(debug==true)  {
            System.out.printf("\n Gyro angle = %.3f\n", driveGyro.getAngle());
            System.out.printf("\n Target angle = %.3f\n", target_angle);
        }

        //  Assumed is that wheels are pointed straight forward
        //  The sharpness of the turn (radius) will depend on the wheel
        //  angle
        turn2Position_ABS(wheel_angle);
        wheel_position=drive.enc_abs.getAbsolutePosition();    
        System.out.printf("\nABS Wheel Position = %.3f degrees\n", wheel_position*360.0);

        while (angle < target_angle){

            //  start forward motion
            drive.falcon_drive.set(ControlMode.PercentOutput, 0.1);

            delay.delay_milliseconds(20.0);

            angle = driveGyro.getAngle();

            delay.delay_milliseconds(20.0);

            if(debug==true)  {
                if (count == 2){
                    System.out.printf("\nGyro angle = %.3f  target = %.3f\n", angle,target_angle);
                    count = 0;
                }
                count++;
            }

           

             //  Escape hatch
             end_time=System.nanoTime();
             elapsed_time=end_time-start_time;
             elapsed_time*=1e-6;  //  convert to milliseconds
             if(elapsed_time>5000.0)  {
                 System.out.printf("Elapsed Time within turnRight() has exceeded limits.");
                 stop();
                 break;
             }
        }  //  while( ... )
        
        error=target_angle-angle;

        delay.delay_milliseconds(20.0);

        //  Straighten the wheels
        turn2Position_ABS(0.0);

        return(error);

    }

    /////////////////////////////////////////////////////////////////
    //  Function: turnLeft(double degrees, double wheel_angle,double speed)  {
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Using the ADXRS450_Gyro gyroscope and the
    //  previous turning wheel function we turn the robot assembly
    //  to the right until the gyro reads degrees.  The flow
    //  is to turn the turning wheel to "wheel_angle" and drive the unit
    //  forward until the target "degrees" is reached
    //
    //  Arguments:double degrees: The desired outcome of the gyro
    //            relative to the starting position.
    //            double wheel_angle:  The angle to which the turning
    //            wheel is turned.  Higher values will make for 
    //            sharper turns.
    //            double speed:  the setting for the drive motor.
    //
    //  Returns:  The error value (target-actual).
    //
    //  Remarks:
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double turnLeft(double degrees, double wheel_angle,double speed)  {

        boolean debug=true;
        
        double angle = driveGyro.getAngle();  //  What is our current heading?
        double target_angle;
        double abs_count;
        int count = 0;

        //  Escape hatch
        long start_time=0;
        long end_time=0;
        double elapsed_time=0.0;

        double error=0;

        target_angle=angle - degrees;

        if(debug==true)  {
            System.out.printf("\nGyro angle = %.3f degrees\n", driveGyro.getAngle());
            System.out.printf("\nGyro Target = %.3f degrees\n", target_angle);
        }

        //  Assumed is that wheels are pointed straight forward
        //  The sharpness of the turn (radius) will depend on the wheel
        //  angle.  For a left turn the will be in the opposite direction.
        //  E.G. to turn the wheels counterclockwise 20 degrees means
        //  340 degrees (360-20).
        wheel_angle=360.0-wheel_angle;
        turn2Position_ABS(wheel_angle);

        abs_count=drive.enc_abs.getAbsolutePosition();   
        System.out.printf("\nABS position = %.3f degrees\n", abs_count*360.0);

        start_time=System.nanoTime();

        while (angle > target_angle){

            //  start forward motion
            drive.falcon_drive.set(ControlMode.PercentOutput, 0.1);

            delay.delay_milliseconds(20.0);

            angle = driveGyro.getAngle();

            delay.delay_milliseconds(20.0);

            if(debug==true)  {
                if (count == 2){
                    System.out.printf("\nGyro angle = %.3f\n", angle);
                    count = 0;
                }
                count++;
            }

            

             //  Escape hatch
             end_time=System.nanoTime();
             elapsed_time=end_time-start_time;
             elapsed_time*=1e-6;  //  convert to milliseconds
             if(elapsed_time>5000.0)  {
                 System.out.printf("Elapsed Time within turnLeft() has exceeded limits.");
                 stop();
                 break;
             }
        }

        delay.delay_milliseconds(200.0);
        error=target_angle-angle;

        //  Straighten the wheels
        turn2Position_ABS(0.0);

        return(error);        

    }





    
    
    

   
 

/////////////////////////////////////////////////////////////////
    //  Function: void stop()
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  kills the motors once the thread is terminated.
    //
    //  Arguments:
    //
    //  Returns:
    //
    //  Remarks:
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////

    double driveFwd(double inches)
    {
        boolean debug=false;
        double count=0;
        double target=0;

        long start_time=0;
        long end_time=0;
        double elapsed_time=0.0;
        double error;
        
        //  Get the initial position, compute the number of counts
        count=drive.falcon_drive.getSelectedSensorPosition(0);           
        target=count+drive.compute_countsDistance(inches);  
        updateCounter=0;    

        start_time=System.nanoTime();

        if(debug==true)  {
            System.out.printf("Fwd Drive target = %.3f count = %.3f\n",drive.drive_target,drive.count);
        }

             
        while (count < (target+drive.drive_deadband)){
            drive.falcon_drive.set(ControlMode.PercentOutput, 0.2);

            delay.delay_milliseconds(20.0);
    
            count=drive.falcon_drive.getSelectedSensorPosition(0);

            error = target - count;  //  In this case should be positive
            if(debug==true)  {
                updateCounter++;
                if (updateCounter == 10){
                    System.out.printf("count = %.3f\n",count);
                    System.out.printf("error = %.3f\n",error);
                    updateCounter = 0;
                }
            }

            //  Escape hatch
            end_time=System.nanoTime();
            elapsed_time=end_time-start_time;
            elapsed_time*=1e-6;  //  convert to milliseconds
            if(elapsed_time>2000.0)  {
                System.out.printf("Elapsed Time within driveFwd() has exceeded limits.");
                stop();
                break;
            }
            
        }
        
        //  Are we within the deadband for the turn?  Have we overshot?
        //  If either of these are true, stop the motor, read the position,
        //  and return the error (turn_target-count)
        if ((Math.abs(target-count)<drive.drive_deadband)||(count > target)) {
            stop();
            count=drive.falcon_drive.getSelectedSensorPosition(0);
            delay.delay_milliseconds(20);
            updateCounter=0;
            return(target-count);
        }
   
        //  If we have reached this point it would indicate an error of some sort.  Either
        //  we exited the while loop because of a hang or we failed to converge within the
        //  deadband.
        return(999.999);

    }

    /////////////////////////////////////////////////////////////////
    //  Function: void stop()
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  kills the motors once the thread is terminated.
    //
    //  Arguments:
    //
    //  Returns:
    //
    //  Remarks:
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double driveReverse(double inches)
    {
        boolean debug=false;

        double count=0;
        double target=0;
        long start_time=0;
        long end_time=0;
        double elapsed_time=0.0;
        double error;

        // Get the initial position, compute the number of counts
        count=drive.falcon_drive.getSelectedSensorPosition(0);           
        target=count-drive.compute_countsDistance(inches);         
        
        if(debug==true)  {
            System.out.printf("RevDrive target = %.3f count = %.3f\n",target,count); 
        }  
        
        updateCounter=0;

        start_time=System.nanoTime();
                
        while (count > target){
            drive.falcon_drive.set(ControlMode.PercentOutput, -0.2);

            delay.delay_milliseconds(20.0);
    
            count=drive.falcon_drive.getSelectedSensorPosition(0);

            error = target - count;
    
            if(debug==true)  {
                updateCounter++;
                if (updateCounter == 10){
                    System.out.printf("count = %.3f\n",drive.count);
                    System.out.printf("error = %.3f\n",error);
                    updateCounter = 0;
                }
            }

             //  Escape hatch
             end_time=System.nanoTime();
             elapsed_time=end_time-start_time;
             elapsed_time*=1e-6;  //  convert to milliseconds
            if(elapsed_time>2000.0)  {
                System.out.printf("Elapsed Time within turnRight() has exceeded limits.");
                stop();
                break;
            }

        }
        
        //  Are we within the deadband for the turn?  Have we overshot?
        //  If either of these are true, stop the motor, read the position,
        //  and return the error (turn_target-count)
        if ((Math.abs(target-count)<drive.drive_deadband)||(count < target)) {
            stop();
            count=drive.falcon_drive.getSelectedSensorPosition(0);
            updateCounter=0;
            return(target-count);
        }
   
        //  If we have reached this point it would indicate an error of some sort.  Either
        //  we exited the while loop because of a hang or we failed to converge within the
        //  deadband.  Note that the return value here is negative - just so we might
        //  be able to tell which function timed out.
        return(-999.999);

    }
   
    /////////////////////////////////////////////////////////////////
    //  Function: void stop()
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  kills the motors once the thread is terminated.
    //
    //  Arguments:
    //
    //  Returns:
    //
    //  Remarks:
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    private void stop()
    {
       drive.falcon_drive.set(ControlMode.PercentOutput,0.0);
       drive.falcon_turn.set(ControlMode.PercentOutput,0.0);
    }
}