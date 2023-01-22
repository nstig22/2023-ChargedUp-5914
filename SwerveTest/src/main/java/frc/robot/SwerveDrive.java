/////////////////////////////////////////////////////////////////////
//  File:  SwerveDrive.java
/////////////////////////////////////////////////////////////////////
//
//  Purpose:  Defines the class for the swerve drive.  Includes
//  the mechanical parameters for the drive, e.g., two Falcon
//  motors, various gear reduction parameters, wheel diameter,
//  CAN addresses, etc..  Functions for turning, moving are
//  also included.
//
//  Remarks:  1/12/2023:  The rotateRight() and rotateLeft()
//  functions went through significant debugging.  Some 
//  modification has taken place since then and needs to
//  be retested.
//
//  1/16/2023:  Need an absolute encoder on the turn motor.  In
//  matches it is needed to set the turning motor in the "zero"
//  position.
//
//
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

public class SwerveDrive extends Robot{

    //Debug code
    int zeroInit = 1;
    int updateCounter = 0;
    double count;
    boolean motion_complete;

    private int turn_ID=7;
    private int drive_ID=6;

    int rotate_init=1;
    int drive_init=1;

    //  Gear reduction boxes
    private double turn_reduce=20.0;
    private double drive_reduce=4.0;

    //  Ring and pinion teeth
    private double turn_pinion=38.0;
    private double turn_ring=80.0;
    private double drive_pinion=38.0;
    private double drive_ring=64.0;

    private double wheel_diameter=4.0;

    private double counts_per_degree;

    private Delay delay;

    WPI_TalonFX falcon_turn;
    WPI_TalonFX falcon_drive;

    double initial_count;

    double turn_target;
    double turn_power=0.5;
    double turn_deadband=100.0;

    double drive_target;
    double drive_power=0.2;
    double drive_deadband=1000.0;  //  ~ 1/2"

    //  Default Constructor
    SwerveDrive()
    {
        delay=new Delay();

        falcon_turn = new WPI_TalonFX(turn_ID);   
        falcon_turn.setNeutralMode(NeutralMode.Brake);
        falcon_turn.setInverted(true);
        falcon_turn.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0, 50);

        falcon_drive = new WPI_TalonFX(drive_ID);   
        falcon_drive.setNeutralMode(NeutralMode.Brake);
        falcon_drive.setInverted(false);
        falcon_drive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0, 50);

         //  Zero encoders
         falcon_turn.setSelectedSensorPosition(0);
         //  Zero encoders
         falcon_drive.setSelectedSensorPosition(0);
    }

    //  Alternate constructor allowing input of turn and drive CAN ids
    SwerveDrive(int turn_id,int drive_id)
    {
        delay=new Delay();

        //  Assign private member variables to arguments.
        turn_ID=turn_id;
        drive_ID=drive_id;

        falcon_turn = new WPI_TalonFX(turn_id);   
        falcon_turn.setNeutralMode(NeutralMode.Brake);
        falcon_turn.setInverted(true);
        falcon_turn.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0, 50);

        falcon_drive = new WPI_TalonFX(drive_id);   
        falcon_drive.setNeutralMode(NeutralMode.Brake);
        falcon_drive.setInverted(false);
        falcon_drive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0, 50);

         //  Zero encoders
         falcon_turn.setSelectedSensorPosition(0);
         //  Zero encoders
         falcon_drive.setSelectedSensorPosition(0);
    }


    /////////////////////////////////////////////////////////////////
    //  Function:  double computeFinalTurnRatio()
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Computes net gear reduction for the turn mechanism
    //
    //  Arguments:void
    //
    //  Returns:
    //
    //  Remarks:  33.7:1
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double computeFinalTurnRatio()
    {   
        double dtmp;

        dtmp=turn_reduce*turn_ring/turn_pinion;

        return(dtmp);
        
    }


    /////////////////////////////////////////////////////////////////
    //  Function:  double computeFinalDriveRatio()
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Given the initial gear reduction and the pinion
    //            and ring gear teeth counts.  This function
    //            computes the net gear reduction.
    //
    //  Arguments:void
    //
    //  Returns:  The gear reduction as double
    //
    //  Remarks:  8.4:1 with intended configuration
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double computeFinalDriveRatio()
    {   
        double dtmp;

        dtmp=drive_reduce*drive_ring/drive_pinion;

        return(dtmp);
        
    }

    /////////////////////////////////////////////////////////////////
    //  Function:  double compute_countsDistance(double distance)
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Computes the number of encoder counts to achieve
    //            the submitted distance in inches.
    //
    //  Arguments:double distance (in inches)
    //
    //  Returns:  Returns the number of counts as double
    //
    //  Remarks:  Assuming a 4" wheel diameter we have 1369 counts
    //            per inch.
    //
    /////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////  distance in inches
    double compute_countsDistance(double distance)
    {
        double dtmp;
        double counts_per_rev;  //  encoder counts per wheel revolution  
        double wheel_revs;      //  number of drive wheel rotations to acheive distance
        
        wheel_revs=distance/(Math.PI*wheel_diameter);
        counts_per_rev=2048*computeFinalDriveRatio();

        dtmp=wheel_revs*counts_per_rev;
        return(dtmp);

    }


    /////////////////////////////////////////////////////////////////
    //  Function:  double compute_countsDegrees(double degrees)
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Computes the encoder counts for a rotation
    //            of the specified number of degrees.
    //
    //  Arguments:double degrees
    //
    //  Returns:  The number of encoder counts for the rotation
    //
    //  Remarks:  Just to get an idea of the magnitude of the 
    //            counts for 1 degree of rotation:
    //
    //            For a full turn of the rotating mechanism (360)
    //            Assume there is a gear reduction of the order of 33.7:1.
    //            If this were true it would imply 191.7 counts per
    //            degree.
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double compute_countsDegrees(double degrees)
    {
        double dtmp;
        double motor_revs;
        
        //  To rotate 360 degrees requires 
        motor_revs=computeFinalTurnRatio();
        counts_per_degree=2048*motor_revs/360.0;

        dtmp=counts_per_degree*degrees;
     
        return(dtmp);
    }

    double counts_perDegree()
    {
        double motor_revs;

        motor_revs=computeFinalTurnRatio();
        counts_per_degree=2048*motor_revs/360.0;

        return(counts_per_degree);
    }


    /////////////////////////////////////////////////////////////////
    //  Function:   double rotateRight(double degrees)
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Rotates the swerve drive in a clockwise direction
    //
    //  Arguments:double degrees
    //
    //  Returns: The error in counts as double
    //
    //  Remarks: At this point we assume that rotating right implies
    //  increasing counts.  It is intended that this function is
    //  called multiple times in either teleop or auto modes.
    //  
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double rotateRight(double degrees)
    {
        double error=0.0;

        //  First time through, get the initial position.  We compute
        //  the number of counts associated with the degrees and
        //  compute our target count.  If we have our motor inversion
        //  state correctly set we add the computed counts to the
        //  present position to rotate the drive clockwise
        if(rotate_init==1)  {
            turn_target=compute_countsDegrees(degrees);
            System.out.printf("Turn target = %.3f\n",turn_target);
            updateCounter=0;
            rotate_init=0;            
        }

        if(motion_complete==false)  {      
            //  Set the motor in motion, wait a bit and then read the encoder
            //  and compute the error.
            if (count < turn_target){
                falcon_turn.set(ControlMode.PercentOutput, 0.5);

                delay.delay_milliseconds(20.0);
        
                count=falcon_turn.getSelectedSensorPosition(0);

                error = turn_target - count;  //  In this case should be positive
            }
            
            //  Are we within the deadband for the turn?  Have we overshot?
            //  If either of these are true, stop the motor.  We need to
            //  set a flag that the motion has been completed or we will
            //  continue to drive the motors on calls after we have
            //  satisfied this condition.  Since "error" has function
            //  scope it is set to zero every time this function is called
            if ((Math.abs(error)<turn_deadband)||(error<0.0)) {
                falcon_turn.set(ControlMode.PercentOutput, 0.0);
                motion_complete=true; 

            }  

            updateCounter++;
            if (updateCounter == 2){
                System.out.printf("\ncount = %.3f  error = %.3f\n",count,error);
                updateCounter = 0;
            }
        }  //  if(motion_complete==false)
        return(error);
  
    }

    /////////////////////////////////////////////////////////////////
    //  Function:  double rotateLeft(double degrees)
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Rotates the swerve drive in a counter-clockwise 
    //  direction the number of degrees specified.
    //
    //  Arguments:double degrees
    //
    //  Returns: The error in  counts as double
    //
    //  Remarks: At this point we assume that rotating left implies
    //  decreasing counts.  
    //  Note that if called in TeleOp or Autonomous repeat
    //  calls are made every 20msec or so.
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double rotateLeft(double degrees)
    {
        double error=0.0;

        //  We compute the number of counts associated with the degrees and
        //  compute our target count.  If we have our motor inversion
        //  state correctly set we subtract the computed counts from the
        //  present position to rotate the drive counter-clockwise
        if(rotate_init==1)  {
            turn_target=compute_countsDegrees(degrees);
            turn_target*=-1.0;
            System.out.printf("Turn target = %.3f\n",turn_target);
            updateCounter=0;
            rotate_init=0;            
        }

        if(motion_complete==false)  {     
            //  Set the motor in motion, wait a bit and then read the encoder
            //  and compute the error.  We keep driving the motor as long
            //  as the measured encoder counts are greater than the target
            //  value.
            if (count > turn_target)  {
                falcon_turn.set(ControlMode.PercentOutput, -0.5);

                delay.delay_milliseconds(20.0);
        
                count=falcon_turn.getSelectedSensorPosition(0);
                //  Compute the error.  In this case it should be negative.
                error = turn_target - count; 
            }
            
            //  Are we within the deadband for the turn?  Have we overshot?
            //  If either of these are true, stop the motor.
            if ((Math.abs(error)<turn_deadband)||(error>0.0)) {
                falcon_turn.set(ControlMode.PercentOutput, 0.0);
                motion_complete=true;
            }

            updateCounter++;
            if (updateCounter == 2){
                System.out.printf("\ncount = %.3f  error = %.3f\n",count,error);
                updateCounter = 0;
            }
        }  //  if(motion_complete==false)
   
        return(error);

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
    //  Remarks:  This function does not provide benefit unless we
    //  started the robot with it's turning wheel pointed straight
    //  forward.  This is a selling point for an absolute encoder.
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    int return2Zero()
    {
        double error=0;
        double degrees;

        //  First time through, get the initial position
        if(zeroInit == 1)  {
            initial_count=falcon_turn.getSelectedSensorPosition(0);
            // In this case our target should be less than the present
            // position - could in fact be negative.  The result of the
            // computation of degree counts will be positive.
            turn_target=0.0;
            error=turn_target-initial_count;
            System.out.printf("initial_count = %.3f  error = %.3f\n",initial_count,error);
            zeroInit = 0;
            rotate_init=1;
        }

        degrees=error/counts_perDegree();
        motion_complete=false;

        if(initial_count>0.0)  {
        
            rotateLeft(degrees);
            
        }  else if(initial_count<0.0)  {
           
            rotateRight(degrees);

        }
        return(0);

    }

    /////////////////////////////////////////////////////////////////
    //  Function:  double moveFwd(double inches)
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Drives the swerve assembly forward the specified
    //  number of inches.
    //
    //  Arguments:Accepts the distance in inches as double.
    //
    //  Returns: The error between intention and execution.
    //
    //  Remarks:  1/10/2023: At this writing, it is assumed that 
    //  clockwise rotation of the motor will drive the robot forward from
    //  it's current position.  It might be the opposite
    //  when the various gear reductions are taken into account.
    //
    //  Note that movement is referenced to the current position.
    //
    //  1/17/2023:  Interesting observation.  Although in the
    //  init phase we set the count to the initial position once
    //  we turn on the motor it would appear that the count begins
    //  at zero.
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double moveFwd(double inches)
    {
        double error=0;

       
        if(drive_init==1)  {
            drive_target=compute_countsDistance(inches);        
            System.out.printf("drive_target = %.3f\n",drive_target);
            drive_init=0;
            updateCounter=0;
            motion_complete=false;
        }

        
        if(motion_complete==false)  {
            // Turn on the drive motor 
            falcon_drive.set(ControlMode.PercentOutput, drive_power);
            delay.delay_milliseconds(25.0);

            count=falcon_drive.getSelectedSensorPosition(0);
            error=drive_target-count;

            //  Hard stop if we are near target or have gone past
            if((Math.abs(error)<drive_deadband) || (error<0.0)) {
                falcon_drive.set(ControlMode.PercentOutput,0.0);
                motion_complete=true;
            }

            updateCounter++;
            if (updateCounter == 5){
                System.out.printf("\ncount = %.3f  error = %.3f\n",count,error);
                updateCounter = 0;
            }
        }
        return(error);

    }


    /////////////////////////////////////////////////////////////////
    //  Function:  double moveReverse(double inches)
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Drives the swerve assembly rearward the specified
    //  number of inches.
    //
    //  Arguments:Accepts the distance in inches as double.
    //
    //  Returns: The error between intention and execution.
    //
    //  Remarks:  1/10/2023: At this writing, it is assumed that 
    //  counter-clockwise rotation of the motor will drive the 
    //  robot in reverse from it's current postion.  It might be 
    //  the opposite when the various gear reductions are taken 
    //  into account.
    //
    //  Note that movement is referenced to the current position.
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double moveReverse(double inches)
    {
        double error=0;

        if(drive_init==1)  {
            drive_target=compute_countsDistance(inches); 
            drive_target*=-1.0;       
            System.out.printf("drive_target = %.3f\n",drive_target);
            drive_init=0;
            updateCounter=0;
            motion_complete=false;
        }

        if(motion_complete==false)  {

            falcon_drive.set(ControlMode.PercentOutput, -drive_power);

            delay.delay_milliseconds(10.0);

            count=falcon_drive.getSelectedSensorPosition(0);
            error=drive_target-count;

            //  Hard stop if we are near target or have gone past
            if((Math.abs(error)<drive_deadband) || (error>0.0)) {
                falcon_drive.set(ControlMode.PercentOutput,0.0);
                motion_complete=true;
            }

            updateCounter++;
            if (updateCounter == 5){
                System.out.printf("\ncount = %.3f  error = %.3f\n",count,error);
                updateCounter = 0;
            }
        }
        return(error);

    }
  
}

    /////////////////////////////////////////////////////////////////
    //  Function:
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:
    //
    //  Arguments:
    //
    //  Returns:
    //
    //  Remarks:
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////