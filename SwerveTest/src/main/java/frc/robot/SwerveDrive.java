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
    double count = 0;

    private int turn_ID=7;
    private int drive_ID=6;

    int rotate_init=1;
    int drive_init=1;

    //  Gear reduction boxes
    private double turn_reduce=20.0;
    private double drive_reduce=4.0;

    //  Ring and pinion teeth
    private double turn_pinion=33.0;
    private double turn_ring=80.0;
    private double drive_pinion=33.0;
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
    double drive_power=0.5;
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
        falcon_drive.setNeutralMode(NeutralMode.Coast);
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
        falcon_turn.setNeutralMode(NeutralMode.Coast);
        falcon_turn.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0, 50);

        falcon_drive = new WPI_TalonFX(drive_id);   
        falcon_drive.setNeutralMode(NeutralMode.Coast);
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
    //  increasing counts.  
    //  Note that if called in TeleOp or Autonomous repeat
    //  calls will be made every 20msec or so.
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double rotateRight(double degrees)
    {
        double error;

        //  First time through, get the initial position
        if(rotate_init==1)  {
            initial_count=falcon_turn.getSelectedSensorPosition(0);
            // In this case our target should be higher than the present
            // position - could in fact be positive.  The result of the
            // computation of degree counts will be positive.
            turn_target=initial_count + compute_countsDegrees(degrees);
            rotate_init=0;

            System.out.printf("Initial count = %.3f\n", initial_count);
            System.out.printf("Turn target = %.3f\n",turn_target);
        }
        
        if (count < turn_target){
            falcon_turn.set(ControlMode.PercentOutput, 0.5);

            delay.delay_milliseconds(20.0);
    
            count=falcon_turn.getSelectedSensorPosition(0);

            error = turn_target - count;  //  In this case should be positive
    
            updateCounter++;
            if (updateCounter == 5){
                System.out.printf("count = %.3f\n",count);
                System.out.printf("error = %.3f\n",error);
                updateCounter = 0;
            }

        }
        
        //  Are we within the deadband for the turn?  Have we overshot?
        //  If either of these are true, stop the motor, read the position,
        //  and return the error (turn_target-count)
        if ((Math.abs(turn_target-count)<turn_deadband)||(count > turn_target)) {
            falcon_turn.set(ControlMode.PercentOutput, 0.0);
            delay.delay_milliseconds(20.0);
            count=falcon_turn.getSelectedSensorPosition(0);
            return(turn_target-count);
        }
   
        return(0.0);

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
    //  Returns: Zero until finished in which case the error in 
    //  counts as double
    //
    //  Remarks: At this point we assume that rotating left implies
    //  decreasing counts.  
    //  Note that if called in TeleOp or Autonomous repeat
    //  calls will be made every 20msec or so.
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double rotateLeft(double degrees)
    {
        double error;

        //  First time through, get the initial position
        if(rotate_init==1)  {
            initial_count=falcon_turn.getSelectedSensorPosition(0);
            // In this case our target should be less than the present
            // position - could in fact be negative.  The result of the
            // computation of degree counts will be positive.
            turn_target=initial_count - compute_countsDegrees(degrees);
            rotate_init=0;

            System.out.printf("Initial count = %.3f\n", initial_count);
            System.out.printf("Turn target = %.3f\n",turn_target);
        }
        
        if (count > turn_target){
            falcon_turn.set(ControlMode.PercentOutput, -0.5);

            delay.delay_milliseconds(20.0);
    
            count=falcon_turn.getSelectedSensorPosition(0);

            error = turn_target - count;
    
            updateCounter++;
            if (updateCounter == 5){
                System.out.printf("count = %.3f\n",count);
                System.out.printf("error = %.3f\n",error);
                updateCounter = 0;
            }

        }
        
        //  Are we within the deadband for the turn?  Have we overshot?
        //  If either of these are true, stop the motor, read the position,
        //  and return the error (turn_target-count)
        if ((Math.abs(turn_target-count)<turn_deadband)||(count < turn_target)) {
            falcon_turn.set(ControlMode.PercentOutput, 0.0);
            delay.delay_milliseconds(20.0);
            count=falcon_turn.getSelectedSensorPosition(0);
            return(turn_target-count);
        }
   
        return(0.0);

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
    //  Remarks:TBD: are the directions correct?
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
            zeroInit = 0;
            rotate_init=1;
        }

        degrees=error/counts_perDegree();

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
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double moveFwd(double inches)
    {
        double initial_count;
        double count;
        double error;

        //  First time through, get the initial position
        if(drive_init==1)  {
            initial_count=falcon_drive.getSelectedSensorPosition(0);
            drive_target=initial_count +  compute_countsDistance(inches);
            drive_init=0;
        }

        falcon_drive.set(ControlMode.PercentOutput, drive_power);

        delay.delay_milliseconds(10.0);

        count=falcon_drive.getSelectedSensorPosition(0);
        error=drive_target-count;

        //  Reduce motor power as we approach target.  Clamp
        //  at 0.1 until we reach within the deadband. '5'
        //  is chosen as the deadband multiplier and may be
        //  changed with experimentation.
        if((Math.abs(error)<5*drive_deadband) && (error>0.0)) {
            if(drive_power>0.1)drive_power-=0.1;
            falcon_drive.set(ControlMode.PercentOutput,drive_power);
        }

        //  Hard stop if we are near target or have gone past
        else if((Math.abs(error)<drive_deadband) || (error<0.0)) {
            falcon_drive.set(ControlMode.PercentOutput,0.0);
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
        double initial_count;
        double count;
        double error;

        //  First time through, get the initial position
        if(drive_init==1)  {
            initial_count=falcon_drive.getSelectedSensorPosition(0);
            drive_target=initial_count -  compute_countsDistance(inches);
            drive_init=0;
        }

        falcon_drive.set(ControlMode.PercentOutput, -drive_power);

        delay.delay_milliseconds(10.0);

        count=falcon_drive.getSelectedSensorPosition(0);
        error=drive_target-count;

        //  Reduce motor power as we approach target.  Clamp
        //  at 0.1 until we reach within the deadband. '5'
        //  is chosen as the deadband multiplier and may be
        //  changed with experimentation.
        if((Math.abs(error)<5*drive_deadband) && (error<0.0)) {
            if(drive_power>0.1)drive_power-=0.1;
            falcon_drive.set(ControlMode.PercentOutput,-drive_power);
        }

        //  Hard stop if we are near target or have gone past
        else if((Math.abs(error)<drive_deadband) || (error>0.0)) {
            falcon_drive.set(ControlMode.PercentOutput,0.0);
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