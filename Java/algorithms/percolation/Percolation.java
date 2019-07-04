import edu.princeton.cs.algs4.WeightedQuickUnionUF;

public class Percolation {
    private int[][] grid;
    private int number_of_opensite;
    private int num;
    private int top;
    private int buttom;
    private WeightedQuickUnionUF wqu;

    public Percolation(int n) {
        num = n;
        top = 0;
        number_of_opensite = 0;
        buttom = n * n + 1;
        grid = new int[n][n];

        for (int i=0; i<n; i++) {
            for (int j=0; j<n; j++) {
                grid[i][j] = 0;
            }
        }

        wqu = new WeightedQuickUnionUF( n*n+2);
    }

    public boolean isOpen(int row, int col)
    {
        isValid(row, col);
        return grid[row -1][col -1] == 1;
    }

    public void open(int row, int col) {
        if (!isOpen(row, col)) {
            grid[row -1][col -1] = 1;
            number_of_opensite++;
        }

        if (row == 1) { wqu.union(xyTo1D(row, col), top); }

        if (row == num) { wqu.union(xyTo1D(row, col), buttom); }



        // connect to surrounded points
        if (row > 1 && isOpen(row-1, col)) { wqu.union(xyTo1D(row, col), xyTo1D(row-1, col)); }

        if (col > 1 && isOpen(row, col-1)) { wqu.union(xyTo1D(row, col), xyTo1D(row, col-1)); }

        if (row < num && isOpen(row+1, col)) { wqu.union(xyTo1D(row, col), xyTo1D(row+1, col)); }

        if (col < num && isOpen(row, col+1)) { wqu.union(xyTo1D(row, col), xyTo1D(row, col+1)); }


    }

    private void isValid(int row, int col) {
        if ((row < 1 || row > num) || (col < 1 || col > num)) {
            throw new IllegalArgumentException("Out of index: row = " + row + " col = " + col);
        }
    }

    public int numberOfOpenSites() {
        return number_of_opensite;
    }

    private int xyTo1D(int row, int col) {
        return (num * (row - 1) + col);
    }

    public boolean isFull(int row, int col) {
        isValid(row, col);
        return wqu.find(xyTo1D(row, col)) == wqu.find(0);
    }

    public boolean percolates() {
        return wqu.connected(top, buttom);
    }

    public static void main(String[] args) {
        Percolation p = new Percolation(10);

    }
}
