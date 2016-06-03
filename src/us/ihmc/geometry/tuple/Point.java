package us.ihmc.geometry.tuple;

import java.io.Serializable;

import us.ihmc.geometry.interfaces.GeometryObject;
import us.ihmc.geometry.transform.interfaces.Transform;
import us.ihmc.geometry.tuple.interfaces.PointBasics;
import us.ihmc.geometry.tuple.interfaces.PointReadOnly;
import us.ihmc.geometry.tuple.interfaces.TupleReadOnly;

public class Point extends Tuple implements Serializable, PointBasics, GeometryObject<Point>
{
   private static final long serialVersionUID = -1234830724104344277L;

   public Point()
   {
      super();
   }

   public Point(double x, double y, double z)
   {
      super(x, y, z);
   }

   public Point(double[] pointArray)
   {
      super(pointArray);
   }

   public Point(TupleReadOnly other)
   {
      super(other);
   }

   @Override
   public void set(Point other)
   {
      super.set(other);
   }

   public double distance(PointReadOnly other)
   {
      return Math.sqrt(distanceSquared(other));
   }

   public double distanceSquared(PointReadOnly other)
   {
      double dx = getX() - other.getX();
      double dy = getY() - other.getY();
      double dz = getZ() - other.getZ();
      return dx * dx + dy * dy + dz * dz;
   }

   @Override
   public void applyTransform(Transform transform)
   {
      transform.transform(this);
   }

   @Override
   public boolean epsilonEquals(Point other, double epsilon)
   {
      return super.epsilonEquals(other, epsilon);
   }
}
