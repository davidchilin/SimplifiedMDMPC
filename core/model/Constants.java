package core.model;

import java.io.*;
import core.util.FileIO;

public class Constants {
	// static constants
	public static final double R = 8.314;
	public static final double H_r1 = -153552.8;
	public static final double H_r2 = -118407.2;
	public static final double H_r3 = 414148;
	public static final double[] T0 = {473, 473, 473};
	public static final double[] V = {1, 1, 1, 3, 1};
	public static final double ratio = 0.5;
	public static final double k = 0.8;

	public static final double F1 = 0.0071; // m^3/s, 80 mole/s
	public static final double F2 = 8.697e-4, F4 = 8.697e-4, F6 = 8.697e-4; // m^3/s, 17.64 mole/s
	public static final double F10 = 0.00231; // m^3/s, 15 mole/s
	public static final double Fr = 0.012; // m^3/s
	public static final double Fr1 = ratio*Fr;
	public static final double Fr2 = (1-ratio)*Fr;
	public static final double CA0 = 11264.1; // mole/m^3 0.8786 g/cm^3
	public static final double CB0 = 20282.1; // mole/m^3 0.5679 g/cm^3
	public static final double CC0 = 8174; // mole/m^3 0.8665 g/cm^3
	public static final double CD0 = 6485; // mole/m^3 0.869 g/cm^3
	public static final double HA0 = 74364.49, HB0 = 59110.13, HC0 = 20181.76, HD0 = -28899.88;
	public static final double Tref = 450;
	public static final double CpA = 184.6, CpB = 59.1, CpC = 247, CpD = 301.3;

	public static final double HvA = 30.73e3, HvB = 13.5e3, HvC = 42.26e3, HvD = 45.5e3;
	public static final double[] Q2 = {-44e5, -46e5, -47e5, 92e5, 59e5}; // setpoint steady-state input
	public static final double[] Q =  {-49e5, -48e5, -49e5, 88e5, 56e5}; // initial state input

	public static final double[] umax = {1/CB0, 1/CB0, 7.5e5, 5e5, 5e5, 6e5, 5e5}; //F4,6,Q1,2,3,4,5
	public static final double[] K = {3e3, 2e3, 3e3, 2e3, 8e3};
	public static final double[] tau = {5e2, 5e2, 5e2, 5e2, 5e2};
	public static final double h = 0.25;

	public static final double[] Pv = {	1,1,1,1,1, //was 100
										1,1,1,1,1, // was 100
										1,1,1,1,1, // was 1000
										1,1,1,1,1, // was 100
										1,1,1,1,1};  // was 100 
	public static final double[] Rc = {1e-5, 1e-5, 1e-5, 1e-5, 1e-5};
	public static final double[] Rc2 = {1e1, 1e1};

	public static final double[] Qc3 = {1,1,1,1,1e3,
										1,1,1,1,1e3,
										1,1,1,1,1e3,
										1,1,1,1,1e3,
										1,1,1,1,1e3};

	public static final double[] average = {
		0.09723058782177801, 3.723033120629459E-5,0.05758480768570673, 0.04703631096146602, 0.2855067234,
		0.1260114803158344, 0.0022186046, -0.2610399493940081,0.046355032412964826, 0.2058244179,
		0.17326306718232526,	0.0022451936, -0.11126016853880062,		0.011731060692115134, 0.2209345101,
		-0.7996301427717076, 0.008731975589383358, 0.43640451406565794,	0.14015988311593872, 0.3655487884,
		0.37786388465388193,4.997253828246685E-4, -0.30333303010199536,	0.006036139679915797,0.155081396  };
	
	public static final double[] stdev = { 0.6884122999285412, 0.005942076103779302,
		0.3041884665393345, 0.1434590842301248, 0.2560499037,
		0.7137035128835183, 0.0008950563, 0.36422836404192305,
		0.1436521811845303, 0.1615204313, 0.8546310108066848,
		0.000987725, 0.3147175515448892, 0.13847039574119177,
		0.1664008344, 1.6730965773445865, 0.03884048363808831,
		0.39897424540634907, 0.380068371132413, 0.2167065921,
		0.8597705914824046, 0.015304840586291604, 0.33949914034258133,
		0.06947871936520975, 0.1119531608 }; 
	
	public static final double[] Qc = {	1,1,1,1,1e3,
										1,1,1,1,1e3,
										1e1,1e1,1e1,1e1,3e3,
										1,1,1,1,1e3,
										1,1,1,1,1e3};
	public static final int ODE = 25;
	public static final double[] NoiseBounds = {0.25, 0.01, 0.1, 0.05, 0.25,
												0.25, 0.01, 0.1, 0.05, 0.25,
												0.25, 0.01, 0.1, 0.05, 0.25,
												0.25, 0.01, 0.1, 0.05, 0.25,
												0.25, 0.01, 0.1, 0.05, 0.25};
	// initial guess for steady state which is read from FTx.txt dynamicly
	double[] x0 = new double[ODE];
	double[] xset = new double[ODE];


	public static void main(String[] args){
		Constants cons = new Constants();
		double[] x0 = cons.getInitPoint();
		System.out.print(x0[0]+"\n");
		System.out.print(Constants.H_r1+"\n");
		System.out.print(Constants.umax[0]);
	}

	public double[] getNewInitPoint(double p){
		double[] xinit = getInitPoint();
		double[] xset = getSetPoint();
		double[] xi = new double[xinit.length];
		for(int i=0; i<xi.length; i++){
			xi[i] = xinit[i] + p*(xset[i] - xinit[i]);
		}
		return xi;
	}

	public double[] getInitPoint(){
		try{
			BufferedReader in = new FileIO().getFileBufferedReader("./data/states/InitPoint.txt");
			for(int i=0; i< x0.length; i++){
				String temp = in.readLine();
				x0[i] = Double.parseDouble(temp);
			}
			in.close();
			return x0;
		}catch(Exception e){
			e.printStackTrace();
			return null;
		}
	}
	public double[] getSetPoint(){
		try{
			BufferedReader in = new FileIO().getFileBufferedReader("./data/states/SetPoint.txt");
			for(int i=0; i< xset.length; i++){
				String temp = in.readLine();
				xset[i] = Double.parseDouble(temp);
			}
			in.close();
			return xset;
		}catch(Exception e){
			e.printStackTrace();
			return null;
		}
	}

}
