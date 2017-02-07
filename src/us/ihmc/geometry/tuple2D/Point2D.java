package us.ihmc.geometry.tuple2D;

import java.io.Serializable;

import us.ihmc.geometry.tuple2D.interfaces.Point2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;

public class Point2D extends Tuple2D<Point2D> implements Serializable, Point2DBasics<Point2D>
{
   private static final long serialVersionUID = -615943325053203074L;

   public Point2D()
   {
      super();
   }

   public Point2D(double x, double y)
   {
      super(x, y);
   }

   public Point2D(double[] pointArray)
   {
      super(pointArray);
   }

   public Point2D(Tuple2DReadOnly<?> other)
   {
      super(other);
   }
}
