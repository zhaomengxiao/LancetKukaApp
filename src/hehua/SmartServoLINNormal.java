package hehua;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.text.SimpleDateFormat;
import java.util.Date;

import com.kuka.common.StatisticTimer;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.common.ThreadUtil;
//import com.kuka.connectivity.motionModel.smartServoLIN.ISmartServoLINRuntime;
//import com.kuka.connectivity.motionModel.smartServoLIN.SmartServoLIN;
import com.kuka.med.deviceModel.LBRMed;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;

import functions.Functions;

/**
 * This sample activates a SmartServoLIN motion in position control mode, sends a sequence of Cartesian set points,
 * describing a sine function in z-direction and evaluates the statistic timing.
 */
public class SmartServoLINNormal
{
    private LBRMed m_robot;
    
    //private ISmartServoLINRuntime _smartServoLINRuntime = null;

    // Tool Data
    
    private static final int NUM_RUNS = 600;
    private static final double AMPLITUDE = 70;
    private static final double FREQENCY = 0.6;

    private static final int MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT = 1000;

    private StatisticTimer m_timer = null;
    
    private DragLineOnY       m_drawLine = null;
    
    private double m_lastX = 0.0;
    private double m_lastY = 0.0;
    private double m_lastZ = 0.0;
    
    public SmartServoLINNormal(LBRMed rob)
    {
    	m_robot = rob;
    	
    	initialize();
    }
    
    public void setNewPoint(double x, double y, double z)
    {
    	System.out.println("setNewPoint " + x + " " + y + " " + z);
    	m_lastX = x;
    	m_lastY = y;
    	m_lastZ = z;
    }
        
    public void initialize()
    {
    	System.out.println("SmartServoLINNormal initialize");
    	
    	Frame initialPosition = m_robot.getCurrentCartesianPosition(m_robot.getFlange());
    	
    	m_lastX = initialPosition.getX();
    	m_lastY = initialPosition.getY();
    	m_lastZ = initialPosition.getZ();
    	System.out.println("initialPosition " + initialPosition.getX() + " " + initialPosition.getY() + " " + initialPosition.getZ());
    	
        //SmartServoLIN aSmartServoLINMotion = new SmartServoLIN(initialPosition);
        //aSmartServoLINMotion.setMinimumTrajectoryExecutionTime(20e-3);

        System.out.println("Starting the SmartServoLIN in position control mode");
        //m_robot.getFlange().moveAsync(aSmartServoLINMotion);
        //m_robot.moveAsync(aSmartServoLINMotion);

        System.out.println("Get the runtime of the SmartServoLIN motion");
        //smartServoLINRuntime = aSmartServoLINMotion.getRuntime();

        m_timer = new StatisticTimer();

        // Start the smart servo lin sine movement
        //m_timer = startSineMovement(_smartServoLINRuntime, m_timer);        
    }
    
    public void StopMotion()
    {
    	System.out.println("Stop the SmartServoLIN motion");
        //_smartServoLINRuntime.stopMotion();
    }    
    /*
    private StatisticTimer startSineMovement(ISmartServoLINRuntime smartServoLINRuntime, StatisticTimer timing)
    {
        try
        {
        	System.out.println("startSineMovement");
        	Frame aFrame = smartServoLINRuntime.getCurrentCartesianDestination(m_robot.getFlange());
        	                    	
            for (int i = 0; i < NUM_RUNS; ++i)
            {
                final OneTimeStep aStep = timing.newTimeStep();

                ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);

                // Update the smart servo LIN runtime
                smartServoLINRuntime.updateWithRealtimeSystem();
                                
                // Compute the sine function
                //Frame destFrame = new Frame(aFrame);
                Frame destFrame = aFrame.copy();
                destFrame.setX(m_lastX);
                destFrame.setY(m_lastY);
                destFrame.setZ(m_lastZ);
                //System.out.println(new SimpleDateFormat("HH:mm:ss:SSS").format(new Date()) + "destFrame " + destFrame.getX() + " " + destFrame.getY() + " " + destFrame.getZ());                
                
                // Set new destination
                smartServoLINRuntime.setDestination(destFrame);
                aStep.end();
            }
        	System.out.println("startSineMovement end");
        }
        catch (Exception e)
        {
        	System.out.println(e.getLocalizedMessage());
            e.printStackTrace();
        }
        return timing;
    }*/
}
