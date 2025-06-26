/* Stuff.java contains miscellaneous routines
NoiseGen - returns matrix with autocorrelated noise values with mean 0, ranging from 1 to -1
Mmult - Matrix multiplication
Equal - make matrix equal
Reverse - Reverse indexes in 2-index matrix
avg  - returns average of vector
sdKnuth - returns standard deviation of vector
EvalODE - RRS plant model
*/
package core.util;

import java.util.Random;
import core.util.PlotIt6;

public class Stuff {

	public static double[] NoiseGen(int size, double phi){
		double[] R = new double[size];
		Random x = new Random(7171);
		// System.currentTimeMillis()
		R[0]= Math.sin(x.nextGaussian())/2; // first time step no autocorelation

		for (int i=1;i<size;i++) 
			R[i]=phi*R[i-1]+Math.sin(x.nextGaussian())/2;
		
		return R;
	}
	
	// returns -1 ~< R ~< 1
	public static double[][] NoiseGen(int timesteps, int states, double phi){
		double[][] R = new double[states][timesteps];
		Random x = new Random(714);

		for(int j = 0;j<states;j++) 
			R[j][0]= Math.sin(x.nextGaussian())/2; // first time step no autocorelation
		
		for (int i=1;i<timesteps;i++) 
			for(int j = 0;j<states;j++) 
				R[j][i]= phi*R[j][i-1]+Math.sin(x.nextGaussian())/2;
				
		return R;
	}
	
	public static double[][] Mmult(double[][] x, double[][] z){
		if(x[0].length!=z.length) System.err.print("Matrix dimension mismatch.\n");
		
		int r = x.length;
		double[][] y = new double[r][z[0].length];
				
		double[] col = new double[z.length];
        for (int j = 0; j < r; j++) {
            for (int k = 0; k < z.length; k++)
            	col[k] = z[k][j];
            for (int i = 0; i < r; i++) {
                double[] row = x[i];
                double sum = 0.0;
                for (int k = 0; k < z.length; k++) 
                    sum += row[k] * col[k];
                
                y[i][j] = sum;
            }
        }
		return y;
	}
	
	public static double[] Equal(double[] x){
		double[] y = new double[x.length];
		for(int i=0;i<x.length;i++)
			y[i]=x[i];
		return y;
	}

// y=x; set matrix values equal	
	public static void Equal(double[] x, double[] y){
			for(int i=0;i<x.length;i++)
			y[i]=x[i];
	}
	public static double[][] Equal(double[][] x){
		double[][] y = new double[x.length][x[0].length];
		for(int i=0;i<x.length;i++)
			for(int j=0;j<x[0].length;j++)
				y[i][j]=x[i][j];
		return y;
	}

// y=x; set matrix values equal		
	public static void Equal(double[][] x, double[][] y){
		for(int i=0;i<x.length;i++)
			for(int j=0;j<x[0].length;j++)
				y[i][j]=x[i][j];
	}
	
	public static double[][][] Equal(double[][][] x){
		double[][][] y = new double[x.length][x[0].length][x[0][0].length];
		for(int i=0;i<x.length;i++)
			for(int j=0;j<x[0].length;j++)
				for(int k=0;k<x[0][0].length;k++)
				y[i][j][k]=x[i][j][k];
		return y;
	}

	// y[][]=x[][][b]
	public static double[][] Equal(double[][][] x, int b){
		double[][] y = new double[x.length][x[0].length];
		for(int i=0;i<x.length;i++)
			for(int j=0;j<x[0].length;j++)
				y[i][j]=x[i][j][b];
		return y;
	}
	
	// y[a][b]=y[b][a]
	public static double[][] Reverse(double[][] x){
		double[][] y = new double[x[0].length][x.length];
		for(int i=0;i<x[0].length;i++)
			for(int j=0;j<x.length;j++)
				y[i][j]=x[j][i];
		return y;
	}
	
	// y[]=x[][b]
	public static double[] Equal(double[][] x, int b){
		double[] y = new double[x.length];
		for(int i=0;i<x.length;i++)
				y[i]=x[i][b];
		return y;
	}
	
// y=x; set matrix values equal		
	public static void Equal(double[][][] x, double[][][] y){
		for(int i=0;i<x.length;i++)
			for(int j=0;j<x[0].length;j++)
				for(int k=0;k<x[0][0].length;k++)
				y[i][j][k]=x[i][j][k];
	}
	
	// y[][][b]=x[][]; set matrix values equal		
	public static void Equal(double[][] x, double[][][] y, int b){
		for(int i=0;i<x.length;i++)
			for(int j=0;j<x[0].length;j++)
				y[i][j][b]=x[i][j];
	}
	
	// y[][b]=x[]; set matrix values equal		
	public static void Equal(double[] x, double[][] y, int b){
		for(int i=0;i<x.length;i++)
			y[i][b]=x[i];
	}
	
	public static double sdKnuth ( double[] data ) // standard deviation for complete data set
    {
    final int n = data.length;
    if ( n < 2 )
       {
       return Double.NaN;
       }
    double avg = data[0];
    double sum = 0;
    for ( int i = 1; i < data.length; i++ )
       {
       double newavg = avg + ( data[i] - avg ) / ( i + 1 );
       sum += ( data[i] - avg ) * ( data [i] -newavg ) ;
       avg = newavg;
       }
    // Change to ( n - 1 ) to n if you have complete data instead of a sample.
    return Math.sqrt( sum / ( n) );
    }
	
	public static double avg ( double[] data )
	{
		double total = 0;
		for(double x: data)	total+=x;
		
		return total/data.length;
	}
	
	/**
	 * integrate CSTR ODEs: tspan, time interval; xkk, current state value; uk,
	 * current input value; wk, process noise
	 */
	static double h;
	
	public static double[] evalODE(double[] xkt, double[] usk, double uak) {
		double[] y = new double[12];
		double Cp = 0.231;
		double F01 = 4.998, F02 = 4.998;
		double V1 = 1, V2 = 0.5, V3 = 1;
		double R = 8.314;
		double T01 = 300, T02 = 300;
		double CA01 = 4, CA02 = 3;
		double DH1 = -5.0e4, DH2 = -5.3e4;
		double rho = 1000;
		double k01 = 3e6, k02 = 3e6;
		double E1 = 5e4, E2 = 5.5e4;
		double Hvap = 5;
		double Fr = 1.9;
		double a1 = 2, a2 = 1, a3 = 1.5, a4 = 3;
		double[] MW = { 50, 50, 50, 40 };

		try {
			double[] f = new double[12];
			double[] xABC = new double[3];
			double[] F = new double[3];
			double K, CAr, CBr, CCr;

			xABC[0] = MW[0] * xkt[9] / rho;
			xABC[1] = MW[1] * xkt[10] / rho;
			xABC[2] = MW[2] * xkt[11] / rho;
			K = a1 * xABC[0] + a2 * xABC[1] + a3 * xABC[2] + a4
					* (1 - xABC[0] - xABC[1] - xABC[2]);
			CAr = a1 * xkt[9] / K;
			CBr = a2 * xkt[10] / K;
			CCr = a3 * xkt[11] / K;
			F[0] = F01 + Fr;
			F[1] = F01 + F02 + Fr + uak;
			F[2] = F01 + F02 + uak;
			f[0] = Fr / V1 * xkt[8] + F01 / V1 * T01 - F[0] / V1 * xkt[0] - DH1
					/ rho / Cp * k01 * Math.exp(-E1 / R / xkt[0]) * xkt[1]
					- DH2 / rho / Cp * k02 * Math.exp(-E2 / R / xkt[0])
					* xkt[1] + usk[0] / rho / Cp / V1;
			f[1] = Fr / V1 * CAr + F01 / V1 * CA01 - F[0] / V1 * xkt[1] - k01
					* Math.exp(-E1 / R / xkt[0]) * xkt[1] - k02
					* Math.exp(-E2 / R / xkt[0]) * xkt[1];
			f[2] = Fr / V1 * CBr - F[0] / V1 * xkt[2] + k01
					* Math.exp(-E1 / R / xkt[0]) * xkt[1];
			f[3] = Fr / V1 * CCr - F[0] / V1 * xkt[3] + k02
					* Math.exp(-E2 / R / xkt[0]) * xkt[1];
			f[4] = (F02 + uak) / V2 * T02 + F[0] / V2 * xkt[0] - F[1] / V2
				* xkt[4] - DH1 / rho / Cp * k01
				* Math.exp(-E1 / R / xkt[4]) * xkt[5] - DH2 / rho / Cp
				* k02 * Math.exp(-E2 / R / xkt[4]) * xkt[5] + usk[1] / rho
				/ Cp / V2;
			f[5] = (F02 + uak) / V2 * CA02 + F[0] / V2 * xkt[1] - F[1] / V2
				* xkt[5] - k01 * Math.exp(-E1 / R / xkt[4]) * xkt[5] - k02
				* Math.exp(-E2 / R / xkt[4]) * xkt[5];
			f[6] = F[0] / V2 * xkt[2] - F[1] / V2 * xkt[6] + k01
				* Math.exp(-E1 / R / xkt[4]) * xkt[5];
			f[7] = F[0] / V2 * xkt[3] - F[1] / V2 * xkt[7] + k02
				* Math.exp(-E2 / R / xkt[4]) * xkt[5];
			f[8] = F[1] / V3 * (xkt[4] - xkt[8]) - Hvap * Fr / rho / Cp / V3
				+ usk[2] / rho / Cp / V3;
			f[9] = F[1] / V3 * xkt[5] - Fr / V3 * CAr - F[2] * xkt[9] / V3;
			f[10] = F[1] / V3 * xkt[6] - Fr / V3 * CBr - F[2] * xkt[10] / V3;
			f[11] = F[1] / V3 * xkt[7] - Fr / V3 * CCr - F[2] * xkt[11] / V3;
	
			for (int i = 0; i < 12; i++) {
				if (i % 4 == 0) {
					if(f[i]>3600) f[i]=3600; // max temp change 3.6K in 3.6 sec
					else if(f[i]<-3600) f[i]=-3600;
				}
				else {
					if(f[i]>40) f[i]=40; // max conc change 0.04 kmol in 3.6 sec
					else if(f[i]<-40) f[i]=-40;
				}
				y[i] = xkt[i] + f[i] * h;
				if (y[i] > 1100)
					y[i] = 1100; // max vessel temperature < 1100
				if (y[i] < 0) // bound on state > 0
					y[i] = 0;  
			}
			return y;
		} catch (Exception e) {
			System.err.print(e.toString());
			return null;
		}
	}
	
	public static boolean setH(double h) {
		Stuff.h = h;
		return true;
	}
	public double getH() {
		return Stuff.h;
	}
	
	public static void main(String[] args) {

		int size = 1000;
		double phi = 0.7;
		double[][] x = new double[1][size];
		double[] x2 = NoiseGen(size, phi);
		double[] t = new double[size];
		
		for(int i=0;i<size;i++) {
			t[i]=(double) (i)/size;
		    x[0][i] = x2[i];
			System.out.print(t[i] + ": " + x[0][i] + "\n");
		}
		new PlotIt6("Noise", x, t).plot();
		
		double[][] xx = {{1,3,5},{2,4,6}};
		double[][] zz = {{7,8},{9,10},{11,12}};
		
		double[][] yy=Mmult(xx,zz);
		System.out.print("\n...Done");
		
	}
}