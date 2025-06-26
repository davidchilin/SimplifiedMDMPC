package core.util;

//import java.awt.Font;

import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
//import org.jfree.chart.annotations.XYTextAnnotation;
import org.jfree.chart.axis.AxisLocation;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.plot.CombinedDomainXYPlot;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.StandardXYItemRenderer;
import org.jfree.chart.renderer.xy.XYItemRenderer;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import org.jfree.ui.ApplicationFrame;
import org.jfree.ui.RefineryUtilities;

public class NicePlot extends ApplicationFrame {
    
	final static long serialVersionUID = 1;
	
    public NicePlot(final String title, double[][] x, double[] t, int[] c){
        super(title);
        final JFreeChart chart = createCombinedChart(x, t, c);
        final ChartPanel panel = new ChartPanel(chart, true, true, true, false, true);
        panel.setPreferredSize(new java.awt.Dimension(500, 100*c.length+70));
        setContentPane(panel);
    }
          
    public NicePlot(final String title, double[][] x, double[] t, int[] c, double[][] ranges){
        super(title);
        final JFreeChart chart = createCombinedChart(x, t, c, ranges);
        final ChartPanel panel = new ChartPanel(chart, true, true, true, false, true);
        panel.setPreferredSize(new java.awt.Dimension(500, 100*c.length+70));
        setContentPane(panel);
    }

    public void plot(){
        this.pack();
        RefineryUtilities.centerFrameOnScreen(this);
        this.setVisible(true);
    }
    
    private JFreeChart createCombinedChart(double[][] x, double[] t, int[] c) {
    	
    	final XYDataset[] data = new XYDataset[c.length]; 
        final XYItemRenderer[] renderer = new XYItemRenderer[c.length];
        final NumberAxis[] rangeAxis = new NumberAxis[c.length];
        final XYPlot[] subplot = new XYPlot[c.length];
        
    	for(int i=0; i<c.length; i++){
    		data[i] = createDataset1(x, t, c[i]);
            renderer[i] = new StandardXYItemRenderer();
            rangeAxis[i] = new NumberAxis("x"+c[i]);
            rangeAxis[i].setAutoRangeIncludesZero(false);
            subplot[i] = new XYPlot(data[i], null, rangeAxis[i], renderer[i]);
            subplot[i].setRangeAxisLocation(AxisLocation.BOTTOM_OR_LEFT);
    	}
        
        // parent plot...
        final CombinedDomainXYPlot plot = new CombinedDomainXYPlot(new NumberAxis("Time(s)"));
        plot.setGap(10.0);
        
        // add the subplots...
        for(int i=0; i<c.length; i++){
        	plot.add(subplot[i], 1);
        }
        plot.setOrientation(PlotOrientation.VERTICAL);

        // return a new chart containing the overlaid plot...
        return new JFreeChart("State(s) vs Time",
                              JFreeChart.DEFAULT_TITLE_FONT, plot, true);

    }

    private JFreeChart createCombinedChart(double[][] x, double[] t, int[] c, double[][] ranges) {
    	
    	final XYDataset[] data = new XYDataset[c.length]; 
        final XYItemRenderer[] renderer = new XYItemRenderer[c.length];
        final NumberAxis[] rangeAxis = new NumberAxis[c.length];
        final XYPlot[] subplot = new XYPlot[c.length];
        
    	for(int i=0; i<c.length; i++){
    		data[i] = createDataset1(x, t, c[i]);
            renderer[i] = new StandardXYItemRenderer();
            rangeAxis[i] = new NumberAxis("x"+c[i]);
            //rangeAxis[i].setAutoRangeIncludesZero(false);
            rangeAxis[i].setRange(ranges[i][0], ranges[i][1]);
            subplot[i] = new XYPlot(data[i], null, rangeAxis[i], renderer[i]);
            subplot[i].setRangeAxisLocation(AxisLocation.BOTTOM_OR_LEFT);
    	}
        
        // parent plot...
        final CombinedDomainXYPlot plot = new CombinedDomainXYPlot(new NumberAxis("Time(s)"));
        plot.setGap(10.0);
        
        // add the subplots...
        for(int i=0; i<c.length; i++){
        	plot.add(subplot[i], 1);
        }
        plot.setOrientation(PlotOrientation.VERTICAL);

        // return a new chart containing the overlaid plot...
        return new JFreeChart("State(s) vs Time",
                              JFreeChart.DEFAULT_TITLE_FONT, plot, true);

    }

    private XYDataset createDataset1(double[][] x, double[] t, int i) {

        // create dataset 1...
        final XYSeries series1 = new XYSeries("x"+i);

        for(int k=0; k<t.length; k++){
        	series1.add(t[k],x[k][i]);
        }
        final XYSeriesCollection collection = new XYSeriesCollection();
        collection.addSeries(series1);
        return collection;
    }


}