package application;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import javax.inject.Inject;

import com.kuka.generated.ioAccess.SafeDataIOGroup;
import com.kuka.common.StatisticTimer;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.motionModel.smartServo.ServoMotion;
import com.kuka.connectivity.motionModel.smartServoLIN.ISmartServoLINRuntime;
import com.kuka.connectivity.motionModel.smartServoLIN.SmartServoLIN;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;

/**
 * This example activates a SmartServoLIN motion in Cartesian impedance control mode, sends a sequence of Cartesian set
 * points, describing a sine function in z-direction and modifies compliance parameters during the motion.
 * 
 */
public class motion_Y_Negtive extends RoboticsAPIApplication
{
	@Inject
	private TCPServerSendDataApplication _vi;
    private LBR _lbr;
    private Tool _toolAttachedToLBR;
    private LoadData _loadData;
    private ISmartServoLINRuntime _theSmartServoLINRuntime = null;
    public int s=1;
    // Tool Data
    private static final String TOOL_FRAME = "toolFrame";
    private static final double[] TRANSLATION_OF_TOOL = { 0, 0, 100 };
    private static final double MASS = 0;
    private static final double[] CENTER_OF_MASS_IN_MILLIMETER = { -44.99, 2.67, 120.35 };

    private static final int NUM_RUNS = 60000;
    private static final double AMPLITUDE = 70;
    private static final double FREQENCY = 0.6;
    private static final double[] MAX_TRANSLATION_VELOCITY = { 150, 150, 150 };
    private static final int MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT = 30;
    public double nintegral=0;
    public double nderivative=0;
    public double nPrevious_error=0;
    public double nOutput=0;
    public double nP=0.5;
    public double nI=0.02;
    public double nD=0;
    public double nDistance=0;
    
    
    @Override
    public void initialize()
    {
    	nDistance=0;
        _lbr = getContext().getDeviceFromType(LBR.class);
        // Create a Tool by Hand this is the tool we want to move with some mass
        // properties and a TCP-Z-offset of 100.
        _loadData = new LoadData();
        _loadData.setMass(MASS);
        _loadData.setCenterOfMass(
                CENTER_OF_MASS_IN_MILLIMETER[0], CENTER_OF_MASS_IN_MILLIMETER[1],
                CENTER_OF_MASS_IN_MILLIMETER[2]);
        _toolAttachedToLBR = new Tool("Tool_ForPlane1", _loadData);

        XyzAbcTransformation trans = XyzAbcTransformation.ofTranslation(
                TRANSLATION_OF_TOOL[0], TRANSLATION_OF_TOOL[1],
                TRANSLATION_OF_TOOL[2]);
        ObjectFrame aTransformation = _toolAttachedToLBR.addChildFrame(TOOL_FRAME
                + "(TCP)", trans);
        _toolAttachedToLBR.setDefaultMotionFrame(aTransformation);
        // Attach tool to the robot
        _toolAttachedToLBR.attachTo(_lbr.getFlange());
    }

    /**
     * Move to an initial Position WARNING: MAKE SURE, THAT the pose is collision free.
     */
    public void moveToInitialPosition()
    {
    	JointPosition jReady =_lbr.getCurrentJointPosition();
        _lbr.move(ptp(jReady.get(JointEnum.J1), jReady.get(JointEnum.J2), jReady.get(JointEnum.J3), jReady.get(JointEnum.J4), jReady.get(JointEnum.J5),
        		jReady.get(JointEnum.J6), jReady.get(JointEnum.J7)).setJointVelocityRel(0.1));
        System.out.println("getCurrentJointPosition");
        /* Note: The Validation itself justifies, that in this very time instance, the load parameter setting was
         * sufficient. This does not mean by far, that the parameter setting is valid in the sequel or lifetime of this
         * program */
        boolean bReady=false;
        while (bReady==false){
        	 try{
                 if (!ServoMotion.validateForImpedanceMode(_lbr))
                 {
                     getLogger()
                             .info("Validation of torque model failed - correct your mass property settings");
                     getLogger()
                             .info("Servo motion will be available for position controlled mode only, until validation is performed");
                 }
                 bReady=true;
             }
             catch (Exception e)
             {
             	ThreadUtil.milliSleep(500);
             	 System.out.println("try to restart");
             }
        }
    }

    @Override
    public void run()
    {
         getLogger().info("Move to start position.");
        moveToInitialPosition();

        // Initialize Cartesian impedance control mode
        final CartesianImpedanceControlMode cartImp = createCartImp();

        getLogger()
                .info("Sample Application - SmartServoLIN motion in cartesian impedance control mode");
        runSmartServoLINMotion(cartImp);
    }

    /**
     * Creates a smartServoLIN Motion with the given control mode and moves around.
     * 
     * @param controlMode
     *            the control mode which shall be used
     * @see {@link CartesianImpedanceControlMode}
     */

    protected void runSmartServoLINMotion(final IMotionControlMode controlMode)
    {
        AbstractFrame initialPosition = _lbr.getCurrentCartesianPosition(_lbr
                .getFlange());

        // Create a new smart servo linear motion
        SmartServoLIN aSmartServoLINMotion = new SmartServoLIN(initialPosition);

        aSmartServoLINMotion.setMaxTranslationVelocity(MAX_TRANSLATION_VELOCITY);
        aSmartServoLINMotion.setMinimumTrajectoryExecutionTime(20e-3);

        getLogger().info("Starting the SmartServoLIN in " + controlMode);
        _lbr.moveAsync(aSmartServoLINMotion.setMode(controlMode));

        getLogger().info("Get the runtime of the SmartServoLIN motion");
        _theSmartServoLINRuntime = aSmartServoLINMotion.getRuntime();

        StatisticTimer timing = new StatisticTimer();

        getLogger().info("Do sine movement");
        timing = startSineMovement(_theSmartServoLINRuntime, timing, controlMode);

//        ThreadUtil.milliSleep(500);

        // Print statistic timing
        getLogger().info(
                getClass().getName() + _theSmartServoLINRuntime.toString());

        getLogger().info("Stop the SmartServoLIN motion");
        _theSmartServoLINRuntime.stopMotion();

        // Statistic Timing of sine movement loop
        if (timing.getMeanTimeMillis() > 150)
        {
            getLogger()
                    .info("Statistic Timing is unexpected slow, you should try to optimize TCP/IP Transfer");
            getLogger()
                    .info("Under Windows, you should play with the registry, see the e.g. user manual");
        }
    }

    /**
     * Create the CartesianImpedanceControlMode class for motion parameterization.
     * 
     * @see {@link CartesianImpedanceControlMode}
     * @return the created control mode
     */
    private CartesianImpedanceControlMode createCartImp()
    {
        final CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();
        cartImp.parametrize(CartDOF.ROT).setStiffness(300.0);
//        cartImp.parametrize(CartDOF.C).setStiffness(50.0);
        cartImp.parametrize(CartDOF.TRANSL).setStiffness(5000.0);
//        cartImp.parametrize(CartDOF.Z).setStiffness(300.0);
        cartImp.parametrize(CartDOF.Y).setStiffness(50.0);
        return cartImp;
    }

    private StatisticTimer startSineMovement(
            ISmartServoLINRuntime theSmartServoLINRuntime,
            StatisticTimer timing, IMotionControlMode mode)
    {
        Frame aFrame = theSmartServoLINRuntime
                .getCurrentCartesianDestination(_lbr.getFlange());

        try
        {
            getLogger().info("Start SmartServoLIN sine movement");
            double omega = FREQENCY * 2 * Math.PI * 1e-9;
            long startTimeStamp = System.nanoTime();
            int i;
       	 Frame initialPosition = _lbr.getCurrentCartesianPosition(_lbr
                 .getFlange());
    	 Frame initialPosition1 = _lbr.getCurrentCartesianPosition(_lbr
                 .getFlange());
       	 Frame initialPosition2 = _lbr.getCurrentCartesianPosition(_lbr
                 .getFlange());
//    	 System.out.println("initialPosition"+initialPosition);
            
            double i1=0;
            double nAix4=0;
            //for (i = 0; i < NUM_RUNS; ++i)
            boolean bOnlyForPlane=false;
    	 while(Math.abs(nAix4)<115 && bOnlyForPlane==false && nDistance>-105)
            {
//    		 System.out.println(i1);
    		 bOnlyForPlane=_vi.MotionType();
//    		 
    		 if(bOnlyForPlane==true){
    			 System.out.println("11");
    		 }
    		 
    		 
    		 JointPosition jReady =_lbr.getCurrentJointPosition();
    		 nAix4=Math.toDegrees(jReady.get(JointEnum.J1));
            	
                final OneTimeStep aStep = timing.newTimeStep();
                // ///////////////////////////////////////////////////////
                // Insert your code here
                // e.g Visual Servoing or the like
                // Synchronize with the realtime system

//                ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);

                // Update the smart servo LIN runtime
                theSmartServoLINRuntime.updateWithRealtimeSystem();

                double curTime = System.nanoTime() - startTimeStamp;
                double sinArgument = omega * curTime;

                // Compute the sine function
                Frame destFrame = _lbr.getCurrentCartesianPosition(_lbr.getFlange());
                
             
       		 initialPosition.setX(initialPosition1.getX());
       		 initialPosition.setY(initialPosition1.getY());
       		 initialPosition.setZ(initialPosition1.getZ());
       		 initialPosition.setAlphaRad(initialPosition1.getAlphaRad());
       		 initialPosition.setBetaRad(initialPosition1.getBetaRad());
       		 initialPosition.setGammaRad(initialPosition1.getGammaRad());

       		double Distance=Math.sqrt( (initialPosition.getX()-destFrame.getX())*(initialPosition.getX()-destFrame.getX())+(initialPosition.getY()-destFrame.getY())*(initialPosition.getY()-destFrame.getY())+(initialPosition.getZ()-destFrame.getZ())*(initialPosition.getZ()-destFrame.getZ()));

       		i1++;
       		
       		
       		Frame cmdPos2 = _lbr.getCurrentCartesianPosition(_lbr.getFlange());
       	    Transformation DistanceToPlane=initialPosition.staticTransformationTo(cmdPos2);
       	    DistanceToPlane.getX();
       	    
       	    //setPid
       	    nintegral=nintegral+DistanceToPlane.getX();
       	    nderivative=DistanceToPlane.getX()-nPrevious_error;
       	    nOutput=nP*DistanceToPlane.getX()+nI*nintegral+nD*nderivative;
       	    nPrevious_error=DistanceToPlane.getX();
       	    
       	    
      		
      		
       		
       		if (i1 % 200 == 0){
           		System.out.println("DistanceToPlaneX:"+DistanceToPlane.getX()+"DistanceToPlaneY:"+DistanceToPlane.getY()+"DistanceToPlaneZ:"+DistanceToPlane.getZ());
           		System.out.println("initialPosition1:"+initialPosition);
           		
       		}
       		
       		if (i1 % 5 == 0){
       			theSmartServoLINRuntime.setDestination(initialPosition);
       		}
       		
       		
             if (i1 % 2 == 0)
             {
//            	 System.out.println("ss");
                 if (mode instanceof CartesianImpedanceControlMode)
                 {
   
               			 double nForceY=0;
               			 if(DistanceToPlane.getY()>0){
               				 if(Math.abs(DistanceToPlane.getY())<5){
               					nForceY=50;
               				 }
               				 else{
               					nForceY=50+20*(Math.abs(DistanceToPlane.getY())-5); 
               				 }
               			 }
               			 else{
               				 if(Math.abs(DistanceToPlane.getY())<90){
               					nForceY=50;
               				 }
               				 else{
               					nForceY=50+20*(Math.abs(DistanceToPlane.getY())-90); 
               				 }
               			 }
               			 if (nForceY>5000)
               			 {
               				nForceY=5000;
               			 }
                         final CartesianImpedanceControlMode cartImp = (CartesianImpedanceControlMode) mode;
                         nDistance=DistanceToPlane.getY();
                        cartImp.parametrize(CartDOF.Y).setStiffness(nForceY);

               			theSmartServoLINRuntime.changeControlModeSettings(cartImp);
                     
                 }
             }
             
             if(i1==10000){
            	 i1=0;
             }
             aStep.end();
            }
    	 nDistance=0;
//	    	Frame Ptest1 = _lbr.getCurrentCartesianPosition(_lbr.getFlange());
//	    	 _lbr.move(ptp(Ptest1).setJointVelocityRel(0.2));
        }
        catch (Exception e)
        {
            getLogger().error(e.getLocalizedMessage());
            e.printStackTrace();
        }
        return timing;
    }

    /**
     * Main routine, which starts the application.
     * 
     * @param args
     *            arguments
     */
    public static void main()
    {
    	
        final motion_Y_Negtive app = new motion_Y_Negtive();
        app.runApplication();
    }
}
