package us.ihmc.geometry.tuple2D;

import java.io.Serializable;

import us.ihmc.geometry.interfaces.GeometryObject;
import us.ihmc.geometry.transform.interfaces.Transform;
import us.ihmc.geometry.tuple2D.interfaces.Point2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;

public class Point2D extends Tuple2D implements Serializable, Point2DBasics, GeometryObject<Point2D>
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

   public Point2D(Tuple2DReadOnly other)
   {
      super(other);
   }

   @Override
   public void set(Point2D other)
   {
      super.set(other);
   }

   public double distance(Point2DBasics other)
   {
      return Math.sqrt(distanceSquared(other));
   }

   public double distanceSquared(Point2DBasics other)
   {
      double dx = getX() - other.getX();
      double dy = getY() - other.getY();
      return dx * dx + dy * dy;
   }

   @Override
   public void applyTransform(Transform transform)
   {
      transform.transform(this);
   }

   public void applyTransform(Transform transform, boolean checkIfTransformInXYplane)
   {
      transform.transform(this);
   }

   @Override
   public boolean epsilonEquals(Point2D other, double epsilon)
   {
      return super.epsilonEquals(other, epsilon);
   }
}
