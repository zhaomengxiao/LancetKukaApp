package hehua;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.text.SimpleDateFormat;
import java.util.Date;

import hehua.DrawLine.LINE_MODEL;

import com.kuka.common.StatisticTimer;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.motionModel.smartServoLIN.ISmartServoLINRuntime;
import com.kuka.connectivity.motionModel.smartServoLIN.SmartServoLIN;
import com.kuka.med.deviceModel.LBRMed;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
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
public class SmartServoLINSimpleMotion
{
    private LBRMed m_robot;
    private Tool _toolAttachedToLBR;
    private LoadData _loadData;
    private ISmartServoLINRuntime _smartServoLINRuntime = null;

    // Tool Data
    private static final String TOOL_FRAME = "toolFrame";
    private static final double[] TRANSLATION_OF_TOOL = { 0, 0, 100 };
    private static final double MASS = 0;
    private static final double[] CENTER_OF_MASS_IN_MILLIMETER = { 0, 0, 100 };

    private static final int NUM_RUNS = 600;
    private static final double AMPLITUDE = 70;
    private static final double FREQENCY = 0.6;

    private static final int MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT = 80;

    private StatisticTimer m_timer = null;
    
    private DragLineOnY       m_drawLine = null;
    
    private double m_lastX = 0.0;
    private double m_lastY = 0.0;
    private double m_lastZ = 0.0;
    
    private double m_dstX = 0.0;
    private double m_dstY = 0.0;
    private double m_dstZ = 0.0;
    
    private double m_d = 0.0;
    
    private double m_totalChangeY = 0.0;
    
    private boolean m_bDone = false;
    
    public SmartServoLINSimpleMotion(LBRMed rob)
    {
    	m_robot = rob;
    	
    	//initialize();
    }
    
    public void setDrawLine(DragLineOnY line)
    {
    	m_drawLine = line;
    }
    
    public void setDistanceAndDstPosition(double d, double x, double y, double z)
    {
    	m_d = d;
    	m_dstX = x;
    	m_dstY = y;
    	m_dstZ = z;
    }
    
    
    public boolean initialize()
    {
    	System.out.println("SmartServoLINSimpleMotion initialize");
    	
    	AbstractFrame initialPosition = m_robot.getCurrentCartesianPosition(m_robot.getFlange());
    	
    	// check distination position
    	if (Math.abs(m_dstX) < 0.000001 && Math.abs(m_dstY) < 0.000001 && Math.abs(m_dstZ) < 0.000001)
    	{
    		
    	}
    	else
    	{
    		/*if(Functions.isPathViaSingularPoint(m_robot, m_dstX, m_dstY, m_dstZ)){
    			System.out.println("goal can not be reached!");
    			return false;
    		}*/    		
    	}

    	Frame fr = (Frame)initialPosition;
    	System.out.println("initialPosition " + fr.getX() + " " + fr.getY() + " " + fr.getZ());
    	
        // Create a new smart servo linear motion
        SmartServoLIN aSmartServoLINMotion = new SmartServoLIN(initialPosition);

        aSmartServoLINMotion.setMinimumTrajectoryExecutionTime(20e-3);

        System.out.println("Starting the SmartServoLIN in position control mode");
        m_robot.getFlange().moveAsync(aSmartServoLINMotion);
        //m_robot.moveAsync(aSmartServoLINMotion);

        System.out.println("Get the runtime of the SmartServoLIN motion");
        _smartServoLINRuntime = aSmartServoLINMotion.getRuntime();

        m_timer = new StatisticTimer();

        // Start the smart servo lin sine movement
        if (m_d > 0)
        {
        	m_timer = startSineMovement_distance_Auto(_smartServoLINRuntime, m_timer);
        }        

        //ThreadUtil.milliSleep(1000);

        //System.out.println("Print statistic timing");
        //System.out.println(getClass().getName() + _smartServoLINRuntime.toString());

        

        // Statistic Timing of sine movement loop
        /*if (timing.getMeanTimeMillis() > 150)
        {
        	System.out.println("Statistic Timing is unexpected slow, you should try to optimize TCP/IP Transfer");
        	System.out.println("Under Windows, you should play with the registry, see the e.g. user manual");
        }*/
        
        return true;
    }

    
    public void StopMotion()
    {
    	System.out.println("Stop the SmartServoLIN motion");
        _smartServoLINRuntime.stopMotion();
    }
    
    public void run(Frame fr)
    {
    	System.out.println("SmartServoLINSimpleMotion run");
    	
    	if (fr == null) return;
    	
    	System.out.println("SmartServoLINSimpleMotion run 1");
    	
    	final OneTimeStep aStep = m_timer.newTimeStep();
    	
    	ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);
    	
    	// Update the smart servo LIN runtime
    	//_smartServoLINRuntime.updateWithRealtimeSystem();
        
        
    	// Set new destination
    	//_smartServoLINRuntime.setDestination(fr);
    	
    	aStep.end();
    	
    	System.out.println("SmartServoLINSimpleMotion run end");
    }

    private StatisticTimer startSineMovement(ISmartServoLINRuntime smartServoLINRuntime, StatisticTimer timing)
    {                        
        System.out.println("Do sine movement");
        try
        {
        	Frame aFrame = smartServoLINRuntime.getCurrentCartesianDestination(m_robot.getFlange());
        	
            double omega = FREQENCY * 2 * Math.PI * 1e-9;
            long startTimeStamp = System.nanoTime();
        	
            for (int i = 0; i < NUM_RUNS; ++i)
            {
                final OneTimeStep aStep = timing.newTimeStep();
                // ///////////////////////////////////////////////////////
                // Insert your code here
                // e.g Visual Servoing or the like
                // Synchronize with the realtime system

                ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);

                // Update the smart servo LIN runtime
                smartServoLINRuntime.updateWithRealtimeSystem();

                double curTime = System.nanoTime() - startTimeStamp;
                double sinArgument = omega * curTime;

                // Compute the sine function
                Frame destFrame = new Frame(aFrame);
                double z = AMPLITUDE * Math.sin(sinArgument);
                destFrame.setZ(z);
                System.out.println("destFrame " + destFrame.getX() + " " + destFrame.getY() + " " + destFrame.getZ());                
                         
                for (int x = 0; x < 100; x++)
                {
                	Frame fr = new Frame(aFrame);
                	fr.setX(10);
                	
                }
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
    }
    
    private StatisticTimer startSineMovement2(ISmartServoLINRuntime smartServoLINRuntime, StatisticTimer timing)
    {        
        if (m_drawLine == null) return timing;
        
        Frame aFrame = smartServoLINRuntime.getCurrentCartesianDestination(m_robot.getFlange());
        
        System.out.println("Do sine movement");
        System.out.println("aFrame pos " + aFrame.getX() + " " + aFrame.getY() + " " + aFrame.getZ());
        try
        {
        	boolean tag = true;
        	int v = 0;
        	for (int i = 1; i < 30; i++)
            {        		        		
                final OneTimeStep aStep = timing.newTimeStep();

                ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);

                // Update the smart servo LIN runtime
                smartServoLINRuntime.updateWithRealtimeSystem();
                                
                Frame newFr = new Frame(aFrame);
                
                if (i > 20 && i < 25)
                {
                	
                }
                else
                {
                	if (tag) v += 2;
                    else v -= 2;                    
                }
                
                newFr.setY(v);
                System.out.println("getCurrentPos " + newFr.getX() + " " + newFr.getY() + " " + newFr.getZ());
                
                        
                try
                {
                	smartServoLINRuntime.setDestination(newFr);                        
                }
                catch (Exception e)
                {
                	
                }
                                
                // Set new destination
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
    }
    
    private StatisticTimer startSineMovement3(ISmartServoLINRuntime smartServoLINRuntime, StatisticTimer timing)
    {        
        if (m_drawLine == null) return timing;
        
        Frame aFrame = smartServoLINRuntime.getCurrentCartesianDestination(m_robot.getFlange());
        
        System.out.println("Do sine movement 3");
        System.out.println("aFrame pos " + aFrame.getX() + " " + aFrame.getY() + " " + aFrame.getZ());
        
        double totalChangeY = 0;
        int nTimes = 300;
        try
        {        	
        	while(true)
            {        		        
        		if (nTimes < 1) break;
        		
                final OneTimeStep aStep = timing.newTimeStep();

                ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);

                // Update the smart servo LIN runtime
                smartServoLINRuntime.updateWithRealtimeSystem();
                
                //Frame fr = m_drawLine.getCurrentPos();
                //double aixValue = m_drawLine.getCurrentAixValue();
                
                Frame destFrame = new Frame(aFrame);
                                      
                //int changeY = (int)(aixValue - aFrame.getY());
                
                double changeY = 0;
                Vector vecF = m_robot.getExternalForceTorque(m_robot.getFlange()).getForce();
                if (vecF.getY() > 4)
                {
                	changeY = 4;                	
                	nTimes--;
                }
                
                totalChangeY += changeY;
                destFrame.setY(totalChangeY);
                                                
                if (changeY != 0)
                	System.out.println(new SimpleDateFormat("HH:mm:ss:SSS").format(new Date()) + "destFrame " + destFrame.getX() + " " + destFrame.getY() + " " + destFrame.getZ());
                                        
                try
                {
                	smartServoLINRuntime.setDestination(destFrame);                        
                }
                catch (Exception e)
                {
                	System.out.println("setDestination " + e.getMessage());
                }
                
                // Set new destination
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
    }
    
    private StatisticTimer startSineMovement_distance_Auto(ISmartServoLINRuntime smartServoLINRuntime, StatisticTimer timing)
    {           
        Frame aFrame = smartServoLINRuntime.getCurrentCartesianDestination(m_robot.getFlange());
        
        System.out.println("startSineMovement_distance_Auto");
        System.out.println("aFrame pos " + aFrame.getX() + " " + aFrame.getY() + " " + aFrame.getZ());
        
        double totalChangeY = 0;
        boolean bDone = false;
        try
        {        	
        	while(!bDone)
            {	
                final OneTimeStep aStep = timing.newTimeStep();

                ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);

                // Update the smart servo LIN runtime
                smartServoLINRuntime.updateWithRealtimeSystem();
                                                
                Frame destFrame = new Frame(aFrame);
                
                double changeY = 0;
                Vector vecF = m_robot.getExternalForceTorque(m_robot.getFlange()).getForce();
                if (vecF.getY() > 4)
                {
                	changeY = 4;             	
                }
                /*else if (vecF.getZ() < -4)
                {
                	System.out.println("startSineMovement_distance break manual");
                	break;
                }*/
                
                
                if (totalChangeY + changeY >= m_d) 
                {
                	bDone = true;
                	
                	if (totalChangeY + changeY > m_d)
                	{
                		changeY = m_d - totalChangeY;
                	}
                }
                
                totalChangeY += changeY;
                destFrame.setY(totalChangeY);
                                                
                if (changeY != 0)
                	System.out.println(new SimpleDateFormat("HH:mm:ss:SSS").format(new Date()) + "destFrame " + destFrame.getX() + " " + destFrame.getY() + " " + destFrame.getZ());
                                        
                try
                {
                	// Set new destination
                	smartServoLINRuntime.setDestination(destFrame);                        
                }
                catch (Exception e)
                {
                	System.out.println("setDestination " + e.getMessage());
                }                
                
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
    }
    
    public String startSineMovement_distance()
    {           
        Frame aFrame = _smartServoLINRuntime.getCurrentCartesianDestination(m_robot.getFlange());
        
        System.out.println("startSineMovement_distance");
        System.out.println("aFrame pos " + aFrame.getX() + " " + aFrame.getY() + " " + aFrame.getZ());
        
        String msg = "";
        try
        {        	
        	if(!m_bDone)
            {	
                final OneTimeStep aStep = m_timer.newTimeStep();

                //ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);

                // Update the smart servo LIN runtime
                _smartServoLINRuntime.updateWithRealtimeSystem();
                                                
                Frame destFrame = new Frame(aFrame);
                
                double changeY = 0;
                Vector vecF = m_robot.getExternalForceTorque(m_robot.getFlange()).getForce();
                if (vecF.getY() > 4)
                {
                	changeY = 4;             	
                }
                /*else if (vecF.getZ() < -4)
                {
                	System.out.println("startSineMovement_distance break manual");
                	break;
                }*/
                
                
                if (m_totalChangeY + changeY >= m_d) 
                {
                	m_bDone = true;
                	
                	if (m_totalChangeY + changeY > m_d)
                	{
                		changeY = m_d - m_totalChangeY;
                	}
                }
                
                m_totalChangeY += changeY;
                destFrame.setY(m_totalChangeY);
                                                
                if (changeY != 0)
                	System.out.println(new SimpleDateFormat("HH:mm:ss:SSS").format(new Date()) + "destFrame " + destFrame.getX() + " " + destFrame.getY() + " " + destFrame.getZ());
                                        
                try
                {
                	// Set new destination
                	_smartServoLINRuntime.setDestination(destFrame);                        
                }
                catch (Exception e)
                {
                	msg = "æŽ¥å?£å¼‚å¸¸ï¼Œè¯·æ£€æŸ¥å?‚æ•°å¹¶é‡?è¯•";
                	System.out.println("setDestination " + e.getMessage());
                }                
                
                aStep.end();
            }
        	
        	if (m_bDone)
        	{
        		msg = "è¿?åŠ¨å®Œæˆ?";
        	}
        	
        	System.out.println("startSineMovement end");
        }
        catch (Exception e)
        {
        	System.out.println(e.getLocalizedMessage());
            e.printStackTrace();
            msg = "æŽ¥å?£å¼‚å¸¸ï¼Œè¯·æ£€æŸ¥å?‚æ•°å¹¶é‡?è¯•";
            
        }
        return msg;
    }

    public String startSineMovement_move(double x, double y, double z)
    {           
        Frame aFrame = _smartServoLINRuntime.getCurrentCartesianDestination(m_robot.getFlange());
        
        System.out.println("startSineMovement_move");
        System.out.println("aFrame pos " + aFrame.getX() + " " + aFrame.getY() + " " + aFrame.getZ());
        
        String msg = "";
        try
        {
        	/*if(Functions.isPathViaSingularPoint(m_robot, x, y, z)){
    			System.out.println("goal can not be reached!");
    			msg = "ç›®æ ‡ä¸?å?¯è¾¾";
    			return msg;
    		}*/ 
        	
            final OneTimeStep aStep = m_timer.newTimeStep();

            // Update the smart servo LIN runtime
            _smartServoLINRuntime.updateWithRealtimeSystem();
                                            
            Frame destFrame = aFrame.copy();
                        
            destFrame.setY(x);
            destFrame.setY(y);
            destFrame.setY(z);                                            
            
            System.out.println(new SimpleDateFormat("HH:mm:ss:SSS").format(new Date()) + "destFrame " + destFrame.getX() + " " + destFrame.getY() + " " + destFrame.getZ());
                                    
            try
            {
            	// Set new destination
            	_smartServoLINRuntime.setDestination(destFrame);                        
            }
            catch (Exception e)
            {
            	msg = "æŽ¥å?£å¼‚å¸¸ï¼Œè¯·æ£€æŸ¥å?‚æ•°å¹¶é‡?è¯•";
            	System.out.println("setDestination " + e.getMessage());
            }                
            
            aStep.end();            
        	
        	if (m_bDone)
        	{
        		msg = "è¿?åŠ¨å®Œæˆ?";
        	}
        	
        	System.out.println("startSineMovement_move end");
        }
        catch (Exception e)
        {
        	System.out.println(e.getLocalizedMessage());
            e.printStackTrace();
            msg = "æŽ¥å?£å¼‚å¸¸ï¼Œè¯·æ£€æŸ¥å?‚æ•°å¹¶é‡?è¯•";
            
        }
        return msg;
    }
    
}
