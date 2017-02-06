package us.ihmc.geometry.tuple;

import java.io.Serializable;

import us.ihmc.geometry.interfaces.GeometryObject;
import us.ihmc.geometry.transform.interfaces.Transform;
import us.ihmc.geometry.tuple.interfaces.Point3DBasics;
import us.ihmc.geometry.tuple.interfaces.Point3DReadOnly;
import us.ihmc.geometry.tuple.interfaces.Tuple3DReadOnly;

public class Point extends Tuple implements Serializable, Point3DBasics, GeometryObject<Point>
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

   public Point(Tuple3DReadOnly other)
   {
      super(other);
   }

   @Override
   public void set(Point other)
   {
      super.set(other);
   }

   public double distance(Point3DReadOnly other)
   {
      return Math.sqrt(distanceSquared(other));
   }

   public double distanceSquared(Point3DReadOnly other)
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
