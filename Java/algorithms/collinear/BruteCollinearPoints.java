/* *****************************************************************************
 *  Name:
 *  Date:
 *  Description:
 **************************************************************************** */
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public class BruteCollinearPoints {
    private LineSegment[] segments;

    public BruteCollinearPoints(Point[] points) {
        if (points == null) throw new java.lang.NullPointerException("points is null in constructor");

        checkNullEntries(points);
        Point[] sortedPoints = points.clone();
        Arrays.sort(sortedPoints);


        checkDuplicatedEntries(sortedPoints);

        final int N = points.length;
        List<LineSegment> list = new LinkedList<>();

        for (int i = 0; i < points.length; i++) {
            Point pointI = sortedPoints[i];

            for (int j = i + 1; j < points.length - 2; j++) {
                Point pointJ = sortedPoints[j];
                double slopeIJ = pointI.slopeTo(pointJ);

                for (int k = j + 1; k < points.length - 1; k++) {
                    Point pointK = sortedPoints[k];
                    double slopeIK = pointI.slopeTo(pointK);
                    if (slopeIJ == slopeIK) {

                        for (int l = k + 1; l < points.length; l++) {
                            Point pointL = sortedPoints[l];
                            double slopeIL = pointI.slopeTo(pointL);
                            if (slopeIJ == slopeIL) {
                                LineSegment tempLine = new LineSegment(pointI, pointL);
                                if (!list.contains(tempLine)) list.add(tempLine);
                            }
                        }
                    }
                }
            }
        }
        segments = list.toArray(new LineSegment[0]);

    }

    public int numberOfSegments() {
        return segments.length;
    }

    public LineSegment[] segments() {
        return segments.clone();
    }

    private void checkNullEntries(Point[] points) {
        for (int i = 0; i < points.length; i++)
        {
            if (points[i] == null) throw new java.lang.NullPointerException();
        }
    }

    private void checkDuplicatedEntries(Point[] points) {
        for (int i = 0; i < points.length -1; i++)
        {
            if (points[i].compareTo(points[i + 1]) == 0)
                throw new java.lang.IllegalArgumentException("Duplicated entries in given points");
        }

    }


    public static void main(String[] args) {


    }

}
