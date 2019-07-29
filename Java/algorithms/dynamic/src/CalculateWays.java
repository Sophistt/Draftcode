import java.util.HashMap;
import java.util.Map;

public class CalculateWays {

  public static int getClimbWays(int n)
  {
    if (n < 1) return 0;
    if (n == 1) return 1;
    if (n == 2) return 2;

    return getClimbWays(n -1 ) + getClimbWays(n -2);
  }

  public static int getClimbWays(int n, HashMap<Integer, Integer>map)
  {
    int value;
    if (n < 1) return 0;
    if (n == 1) return 1;
    if (n == 2) return 2;

    if (map.containsKey(n))
    {
      return map.get(n);
    }
    else {
      value = getClimbWays(n -1, map) + getClimbWays(n - 2, map);
      map.put(n, value);
      return value;
    }

  }

  public static int dynamicWays(int n)
  {
    if (n < 1) return 0;
    if (n == 1) return 1;
    if (n == 2) return 2;

    int a = 1;
    int b = 2;
    int temp = 0;
    for (int i = 3; i <= n; i++)
    {
      temp = a + b;
      a = b;
      b = temp;
    }

    return temp;
  }

  public static void main(String[] args)
  {
    int result = 0;
    HashMap map = new HashMap<Integer, Integer>();
    long startTime = System.nanoTime();
    //result = getClimbWays(20);
    //result = getClimbWays(20, map);
    result = dynamicWays(20);
    long endTime = System.nanoTime();
    System.out.println("Time: " + (endTime - startTime) + "ns");
    System.out.print(result);
  }

}
