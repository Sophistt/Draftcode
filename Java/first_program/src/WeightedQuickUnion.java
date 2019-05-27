/*
* Hypothesis 1: The group has 10000 members.
* Hypothesis 2： the file saved the timestamp shows as followed
*
* 20160824 34 22
* 20160830 23 11
* 20170120 12 09
* 20180830 30 05
* ...
*
 */

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

public class WeightedQuickUnion {
  private int[] parent;
  private int[] size;
  private int count;

  public WeightedQuickUnion(int n) {
    count = n;
    parent = new int[n];
    size = new int[n];
    for (int i = 0; i < n; i++) {
      parent[i] = i;
      size[i] = 1;
    }
  }

  private void validate(int index) {
        int lng = parent.length;
        if (index < 0 || index >= lng ) {
            throw new IllegalArgumentException("index " + index + " is not between 0 and " + (lng-1));
        }
    }

    public int root(int index) {
        validate(index);
        while (parent[index] != index)
            index = parent[index];

        return index;
    }

    public boolean connected(int p, int q) { return root(p) == root(q); }

    public int count() { return  count; }

    public void union(int p, int q) {
        int rootP = root(p);
        int rootQ = root(q);
        if (rootP == rootQ) return;

        if ( size[rootP] >= size[rootQ]) {
            parent[rootQ] = rootP;
            size[rootP] += size[rootQ];
        }
        else {
            parent[rootP] = rootQ;
            size[rootQ] += size[rootP];
        }
        count--;
    }


    static public void main(String arg[])
    {
        int n = 10000;
        WeightedQuickUnion uq = new WeightedQuickUnion(n);

        String fileName = "";
        File file = new File(fileName);
        BufferedReader reader = null;

        try {
            reader = new BufferedReader(new FileReader(file));
            String tempString = null;

            while ((tempString = reader.readLine()) != null) {
                String [] arr = tempString.split(" ");
                int p = Integer.parseInt(arr[1]);
                int q = Integer.parseInt(arr[2]);

                uq.union(p, q);
                if (uq.count == 0) {
                    System.out.println(arr[0]);
                    break;
                }

            }
            reader.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
