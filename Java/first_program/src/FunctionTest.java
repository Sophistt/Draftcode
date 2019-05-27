import edu.princeton.cs.algs4.StdStats;
import edu.princeton.cs.algs4.StdRandom;

public class FunctionTest {
    private int[] a;

    public int value(int n)
    {
        return a[n-1];
    }

    public FunctionTest(int n)
    {
        a = new int[n];
        for (int i=0; i<n; i++)
        {
            a[i] = i + 1;
        }
    }

    public static void main(String[] arg) {

        FunctionTest arr = new FunctionTest(5);
        System.out.println(arr.value(1));
        System.out.println(StdRandom.discrete(arr.a));

    }
}
