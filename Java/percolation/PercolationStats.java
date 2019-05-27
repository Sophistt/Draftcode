import edu.princeton.cs.algs4.StdStats;
import edu.princeton.cs.algs4.StdRandom;
import java.lang.Math;
import edu.princeton.cs.algs4.WeightedQuickUnionUF;

public class PercolationStats {
    private int[] index;
    private double[] open_number;
    private int Trial;


    public PercolationStats(int n, int trials) {
        Trial = trials;
        open_number = new double[trials];
        int row = 0;
        int col = 0;


        for (int i=0; i<trials; i++) {
            Percolation p = new Percolation(n);

            int count = 0;
            while (!p.percolates()) {
                row = StdRandom.uniform(1, n+1);
                col = StdRandom.uniform(1, n+1);
                p.open(row, col);
            }
            open_number[i] = (double) p.numberOfOpenSites() / (n * n);

        }

    }

    public double mean() {
        return StdStats.mean(open_number);
    }

    public double stddev() {
        return StdStats.stddev(open_number);
    }

    public double confidenceLo() {
        return mean() - stddev() / java.lang.Math.sqrt(Trial);
    }

    public double confidenceHi() {
        return mean() + stddev() / java.lang.Math.sqrt(Trial);
    }

    public static void main(String[] args) {
        int n = Integer.parseInt(args[0]);
        int T = Integer.parseInt(args[1]);
        PercolationStats ps = new PercolationStats(n, T);

        String confidence = ps.confidenceLo() + ", " + ps.confidenceHi();
        System.out.println("mean                    = " + ps.mean());
        System.out.println("stddev                  = " + ps.stddev());
        System.out.println("95% confidence interval = " + confidence);
    }
}
