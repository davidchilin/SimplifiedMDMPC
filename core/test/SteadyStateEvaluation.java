package core.test;

import java.io.BufferedWriter;

import core.model.Constants;
import core.model.Dynamics;
import core.util.*;
import core.test.util.Newton;
import core.test.util.NewtonFunction;

public class SteadyStateEvaluation{
	
	public static void main(String[] args){
		double[] xs = new SteadyStateEvaluation().evalSteadyState();
	
		for(int i=0; i<Constants.ODE; i++){
			System.out.print(i+": "+xs[i]);
			System.out.println();
		}
		
		try{
			BufferedWriter out = new FileIO().getFileBufferedWriter("./data/states/SetPoint.txt");
			String line = "";
			for(int i=0; i<xs.length; i++){
				line = xs[i]+"";
				out.write(line);
				out.newLine();
				out.flush();
			}
			out.close();
		}catch(Exception e){
			e.printStackTrace();
		}
		
	}

	public double[] evalSteadyState() {
		double[] x0 = new Constants().getSetPoint();
		Newton newton = new Newton(new SteadyStateFunctions());
		double[] xs = newton.findRoot(x0);
		System.out.println(newton.getIterCount());
		return xs;
	}

}

class SteadyStateFunctions extends NewtonFunction{
	public double[] eval_f(double[] xk){
		double[] uk = {0, 0, 0, 0, 0};
		double[] uk_f = {0, 0};
		double[] xret = new Dynamics().eval(xk, uk, uk_f);
		return xret;
	}
}
