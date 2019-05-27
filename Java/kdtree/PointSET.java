/* *****************************************************************************
 *  Name:
 *  Date:
 *  Description: Use SET and Point to build a PointSET
 **************************************************************************** */

import edu.princeton.cs.algs4.Point2D;
import edu.princeton.cs.algs4.RectHV;
import edu.princeton.cs.algs4.SET;
import edu.princeton.cs.algs4.Stack;

public class PointSET {
    private SET<Point2D> set;

    public PointSET()
    {
        set = new SET<Point2D>();
    }

    /**
     * @return is the set empty?
     */
    public boolean isEmpty()
    {
        return size() == 0;
    }

    public int size()
    {
        return set.size();
    }

    public void insert(Point2D p)
    {
        set.add(p);
    }

    public boolean contains(Point2D p)
    {
        return set.contains(p);
    }

    public void draw()
    {
        for (Point2D p : set)
            p.draw();
    }

    public Iterable<Point2D> range(RectHV rect)
    {
        Stack<Point2D> stackPoint = new Stack<Point2D>();
        for (Point2D point : set)
        {
            if (rect.contains(point))
                stackPoint.push(point);
        }
        return stackPoint;
    }

    public Point2D nearest(Point2D p)
    {

        if (isEmpty())
            return null;
        Point2D nearestPoint = null;
        for (Point2D point : set)
        {
            if (nearestPoint == null)
                nearestPoint = point;
            if (p.distanceTo(nearestPoint) > p.distanceTo(point))
                nearestPoint = point;
        }
        return nearestPoint;

    }

    public static void main(String[] args) {

    }
}
