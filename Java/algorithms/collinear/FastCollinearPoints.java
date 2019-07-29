import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public class FastCollinearPoints {
    private final LineSegment[] segments;

    public FastCollinearPoints(Point[] points) {
        if (points == null) throw new java.lang.NullPointerException("points is null in constructor");

        checkNullEntries(points);
        Point[] sortedPoints = points.clone();
        Arrays.sort(sortedPoints);
        checkDuplicatedEntries(sortedPoints);

        final List<LineSegment> maxLineSegments = new LinkedList<>();

        for (int i = 0; i < points.length; i++) {

            Point p = sortedPoints[i];
            Point[] pointsBySlope = sortedPoints.clone();
            Arrays.sort(pointsBySlope, p.slopeOrder());

            int x = 1;
            while (x < points.length) {

                LinkedList<Point> candidates = new LinkedList<>();
                final double SLOPE_REF = p.slopeTo(pointsBySlope[x]);
                do {
                    candidates.add(pointsBySlope[x++]);
                } while (x < points.length && p.slopeTo(pointsBySlope[x]) == SLOPE_REF);

                // Candidates have a max line segment if ...
                // 1. Candidates are collinear: At least 4 points are located
                //    at the same line, so at least 3 without "p".
                // 2. The max line segment is created by the point "p" and the
                //    last point in candidates: so "p" must be the smallest
                //    point having this slope comparing to all candidates.
                if (candidates.size() >= 3
                        && p.compareTo(candidates.peek()) < 0) {
                    Point min = p;
                    Point max = candidates.removeLast();
                    maxLineSegments.add(new LineSegment(min, max));
                }
            }
        }
        segments = maxLineSegments.toArray(new LineSegment[0]);
    }


    /**
     * The number of line segments.
     */
    public int numberOfSegments() {
        return segments.length;
    }

    /**
     * The line segments.
     */
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
