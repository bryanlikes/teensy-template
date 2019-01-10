#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.2.1

class PID
{


  public:

  //Constants used in some of the functions below
  #define AUTOMATIC	1
  #define MANUAL	0
  #define DIRECT  0
  #define REVERSE  1
  #define P_ON_M 0
  #define P_ON_E 1

  //commonly used functions **************************************************************************
    PID(int*, float*, int*,        // * constructor.  links the PID to the Input, Output, and 
        int, int, int, int, int);//   Setpoint.  Initial tuning parameters are also set here.
                                          //   (overload for specifying proportional mode)

    PID(int*, float*, int*,        // * constructor.  links the PID to the Input, Output, and 
        int, int, int, int);     //   Setpoint.  Initial tuning parameters are also set here
	
    void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

    bool Compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively


  //available but not commonly used functions ********************************************************
    void SetTunings(int, int,       // * While most users will set the tunings once in the 
                    int);         	    //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
    void SetTunings(int, int,       // * overload for specifying proportional mode
                    int, int);         	  

	void SetControllerDirection(int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.
    void SetSampleTime(int);              // * sets the frequency, in Microseconds, with which 
                                          //   the PID calculation is performed.  default is 100000
										  
										  
										  
  //Display functions ****************************************************************
	int GetKp();						  // These functions query the pid for interal values.
	int GetKi();						  //  they were created mainly for the pid front-end,
	int GetKd();						  // where it's important to know what is actually 
	int GetMode();						  //  inside the PID.
	int GetDirection();					  //

  private:
	
	int dispKp;				// * we'll hold on to the tuning parameters in user-entered 
	int dispKi;				//   format for display purposes
	int dispKd;				//
    
	int kp;                  // * (P)roportional Tuning Parameter
  int ki;                  // * (I)ntegral Tuning Parameter
  int kd;                  // * (D)erivative Tuning Parameter

	int controllerDirection;
	int pOn;

  int *myInput;              // * Pointers to the Input, Output, and Setpoint variables
  float *myOutput;             //   This creates a hard link between the variables and the 
  int *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                //   what these values are.  with pointers we'll just know.
			  
	unsigned long lastTime;
	unsigned long SampleTime;
	float error_old, error_i;

	double outMin, outMax;
	bool inAuto, pOnE;
};
#endif
