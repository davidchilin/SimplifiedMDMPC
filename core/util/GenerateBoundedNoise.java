package core.util;

import core.model.Constants;
import java.util.Random;

public class GenerateBoundedNoise extends Random{
	private double[] bounds = Constants.NoiseBounds;
	private int ode = Constants.ODE;
	final static long serialVersionUID = 1;
	
	public static void main(String[] args){
		GenerateBoundedNoise gen = new GenerateBoundedNoise();
		double[][] ret = gen.generateNoise(2);
		double[] ret1 = ret[1];
		for(int i=0; i<ret1.length; i++){
			System.out.println(ret1[i]);
		}
	}
	
	public double[][] generateNoise(int cc){
		double[][] nret = new double[cc][ode];
		for(int i=0; i<cc; i++){
			for(int j=0; j<ode; j++){
				nret[i][j] = 2*(this.nextDouble()-0.5)*bounds[j];
			}
		}
		return nret;
	}
}
