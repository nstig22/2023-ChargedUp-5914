<<<<<<< HEAD
=======

>>>>>>> NewTesting
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
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

package frc.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;


class SwerveDriveThread implements Runnable {

    private SwerveDrive drive;
    
    //  Timing parame int  
    volatile int isactive;

    private long start_time=0;
    private long end_time=0;
    private double elapsed_time=0.0;
    private int updateCounter=0;

   // private double duration=3000;  //  duration in msec
  	
	String name;
	Thread t;
	Runtime r = Runtime.getRuntime();
	private Delay delay;

	// Constructor
	SwerveDriveThread(String threadname) {

        drive = new SwerveDrive();
        		
		name = threadname;
		t = new Thread(this, name);
        
         System.out.printf("New thread: %s",name);
    
		delay = new Delay();

        //  Start the timer
<<<<<<< HEAD
		//start_time=System.nanoTime();
=======
		start_time=System.nanoTime();
>>>>>>> NewTesting
        isactive=1;
        Robot.thread_is_active=true;
		t.start(); // Start the thread
	}

    //  Here is the execution of the thread
	public void run() {

               		
       //  Two ways to go here - we could use an 'if' statement.  That would probably
       //  be safer in that we only intend to run this thread once.  Using a while()
       //  is dangerous in that we could be locked into an infinite loop if the
       //  thread fails to terminate.  'isactive' is set to '1' in the thread
       //  constructor and zeroed if we successfully interrupt the thread.
       while(isactive==1) {

            turnRight(90.0);    

            delay.delay_milliseconds(1000.0);

            //  We need to reset our "init" flag.  The 
            //  previous function call had set it to zero.
            drive.rotate_init=1;
<<<<<<< HEAD
            turnLeft(90.0);
=======
            turnLeft(180.0);
>>>>>>> NewTesting

            delay.delay_milliseconds(1000.0);

            drive.rotate_init=1;
<<<<<<< HEAD
            turnLeft(270.0);  
=======
            turnRight(270.0);  
>>>>>>> NewTesting
            
            delay.delay_milliseconds(1000.0);
            
            drive.rotate_init=1;
            return2Zero();
<<<<<<< HEAD

            drive.rotate_init=1;
=======
>>>>>>> NewTesting
                    
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

            //  Output time to reach this point.  Should be stability
            //  time plus duration.
            end_time=System.nanoTime();
            elapsed_time=end_time-start_time;
            elapsed_time*=1e-6;  //  convert to milliseconds
            System.out.printf("Elapsed time = %.3 msec",elapsed_time);

            //  Ok, lets create an escape hatch in the event that the
            //  thread
           
	
        }  //  while(isactive==1)

        //  brute force release the memory
        r.gc();
        return;
    }  




    /////////////////////////////////////////////////////////////////
    //  Function:   double turnRight(double degrees)
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Rotates the swerve drive in a clockwise direction
    //
    //  Arguments:double degrees
    //
    //  Returns: The error in counts as double
    //
    //  Remarks: Rotating right (clockwise) implies
    //  increasing counts. This version is used within the thread
    //  and incorporates a while() loop.  Unlike the functions
    //  used in teleOP() this function gets called once.
    //
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double turnRight(double degrees)
    {
        double error;

        //  First time through, get the initial position
        if(drive.rotate_init==1)  {
            drive.initial_count=drive.falcon_turn.getSelectedSensorPosition(0);
            // In this case our target should be higher than the present
            // position - could in fact be positive.  The result of the
            // computation of degree counts will be positive.
            drive.turn_target=drive.initial_count + drive.compute_countsDegrees(degrees);
            drive.rotate_init=0;

            System.out.printf("Initial count = %.3f\n", drive.initial_count);
            System.out.printf("Turn target = %.3f\n",drive.turn_target);
<<<<<<< HEAD
            start_time=System.nanoTime();
=======
>>>>>>> NewTesting
        }
        
        while (drive.count < (drive.turn_target+drive.turn_deadband)){
            drive.falcon_turn.set(ControlMode.PercentOutput, 0.5);

            delay.delay_milliseconds(20.0);
    
            drive.count=drive.falcon_turn.getSelectedSensorPosition(0);

            error = drive.turn_target - drive.count;  //  In this case should be positive
    
            updateCounter++;
            if (updateCounter == 5){
                System.out.printf("count = %.3f\n",drive.count);
                System.out.printf("error = %.3f\n",error);
                updateCounter = 0;
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
<<<<<<< HEAD
            
=======

>>>>>>> NewTesting
        }
        
        //  Are we within the deadband for the turn?  Have we overshot?
        //  If either of these are true, stop the motor, read the position,
        //  and return the error (turn_target-count)
        if ((Math.abs(drive.turn_target-drive.count)<drive.turn_deadband)||(drive.count > drive.turn_target)) {
            stop();
            drive.count=drive.falcon_turn.getSelectedSensorPosition(0);
            updateCounter=0;
            return(drive.turn_target-drive.count);
        }
   
        //  If we have reached this point it would indicate an error of some sort.  Either
        //  we exited the while loop because of a hang or we failed to converge within the
        //  deadband.
        return(999.999);

    }

    double turnLeft(double degrees)
    {
        double error;

        //  First time through, get the initial position
        if(drive.rotate_init==1)  {
            drive.initial_count=drive.falcon_turn.getSelectedSensorPosition(0);
            // In this case our target should be less than the present
            // position - could in fact be negative.  The result of the
            // computation of degree counts will be positive.
            drive.turn_target=drive.initial_count - drive.compute_countsDegrees(degrees);
            drive.rotate_init=0;

            System.out.printf("Initial count = %.3f\n", drive.initial_count);
            System.out.printf("Turn target = %.3f\n",drive.turn_target);
<<<<<<< HEAD
            start_time=System.nanoTime();
=======
>>>>>>> NewTesting
        }
        
        while (drive.count > drive.turn_target){
            drive.falcon_turn.set(ControlMode.PercentOutput, -0.5);

            delay.delay_milliseconds(20.0);
    
            drive.count=drive.falcon_turn.getSelectedSensorPosition(0);

            error = drive.turn_target - drive.count;
    
            updateCounter++;
            if (updateCounter == 5){
                System.out.printf("count = %.3f\n",drive.count);
                System.out.printf("error = %.3f\n",error);
                updateCounter = 0;
            }

            if(elapsed_time>2000.0)  {
                System.out.printf("Elapsed Time within turnRight() has exceeded limits.");
                stop();
                break;
            }

        }
        
        //  Are we within the deadband for the turn?  Have we overshot?
        //  If either of these are true, stop the motor, read the position,
        //  and return the error (turn_target-count)
        if ((Math.abs(drive.turn_target-drive.count)<drive.turn_deadband)||(drive.count < drive.turn_target)) {
            stop();
            drive.count=drive.falcon_turn.getSelectedSensorPosition(0);
            updateCounter=0;
            return(drive.turn_target-drive.count);
        }
   
        //  If we have reached this point it would indicate an error of some sort.  Either
        //  we exited the while loop because of a hang or we failed to converge within the
        //  deadband.  Note that the return value here is negative - just so we might
        //  be able to tell which function timed out.
        return(-999.999);

    }

    /////////////////////////////////////////////////////////////////
    //  Function:  int return2Zero()
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Assuming we start the robot with the wheels 
    //  pointing straight forward, on init, we set the encoder
    //  count to zero.  All turns from that point on are 
    //  referenced to the current position with 'zero' being
    //  the home position.  So, our target in counts is zero.
    //  If the current measured position is positive, we wish
    //  to rotate in a direction to reduce the measured count
    //  to zero.  Likewise, if negative we wish to increase the
    //  measured count to zero.
    //
    //  Arguments:void
    //
    //  Returns: 0
    //
    //  Remarks:TBD: This function is the version called within the 
    //  thread.  It is not subject to watchdog issues as it 
    //  operates in parallel (time sliced) with the main thread.
    //  The operations turnRight() and turnLeft(0) values are
    //  examined to determine if these functions have timed out
    //  within their respective while() loops.
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double return2Zero()
    {
        double error=0;
        double degrees;
        double return_val;

        //  First time through, get the initial position
        if(drive.zeroInit == 1)  {
            drive.initial_count=drive.falcon_turn.getSelectedSensorPosition(0);
            // In this case our target should be less than the present
            // position - could in fact be negative.  The result of the
            // computation of degree counts will be positive.
            drive.turn_target=0.0;
            error=drive.turn_target-drive.initial_count;
            drive.zeroInit = 0;
            drive.rotate_init=1;
        }

        degrees=error/drive.counts_perDegree();

<<<<<<< HEAD
        if(drive.initial_count<0.0)  {
=======
        if(drive.initial_count>0.0)  {
>>>>>>> NewTesting
        
            return_val=turnLeft(degrees);
            if(return_val==999.999)  {
                System.out.printf("Timeout: Return of turnLeft() = %.3lf",return_val);
            }
            
<<<<<<< HEAD
        }  else if(drive.initial_count>0.0)  {
=======
        }  else if(drive.initial_count<0.0)  {
>>>>>>> NewTesting
           
            return_val=turnRight(degrees);
            if(return_val==-999.999)  {
                System.out.printf("Timeout: Return of turnRight() = %.3lf",return_val);
            }

        }
        return(0);

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