package core.test;

import java.util.Date;

public class TimeDiffTest {
	  public static void main(String[] av) {
	    Date t1 = new Date();

	    long d1 = t1.getTime();
	    System.out.println("The time difference is: " + d1);
	    for(int i=0; i<10000; i++){
	    	System.out.print(i);
	    }
	    System.out.println();
	    long d2 = new Date().getTime();

	    System.out.println("The time difference is: " + (d2-d1)/1000);
	  }
}
