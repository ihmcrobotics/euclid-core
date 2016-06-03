package us.ihmc.geometry.tuple;

import java.io.Serializable;

import us.ihmc.geometry.interfaces.GeometryObject;
import us.ihmc.geometry.transform.interfaces.Transform;
import us.ihmc.geometry.tuple.interfaces.PointBasics;
import us.ihmc.geometry.tuple.interfaces.PointReadOnly;
import us.ihmc.geometry.tuple.interfaces.TupleReadOnly;

public class Point32 extends Tuple32 implements Serializable, PointBasics, GeometryObject<Point32>
{
   private static final long serialVersionUID = 5142616577127976269L;

   public Point32()
   {
      super();
   }

   public Point32(float x, float y, float z)
   {
      super(x, y, z);
   }

   public Point32(float[] pointArray)
   {
      super(pointArray);
   }

   public Point32(TupleReadOnly other)
   {
      super(other);
   }

   @Override
   public void set(Point32 other)
   {
      super.set(other);
   }

   public float distance(PointReadOnly other)
   {
      return (float) Math.sqrt(distanceSquared(other));
   }

   public float distanceSquared(PointReadOnly other)
   {
      double dx = getX() - other.getX();
      double dy = getY() - other.getY();
      double dz = getZ() - other.getZ();
      return (float) (dx * dx + dy * dy + dz * dz);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      transform.transform(this);
   }

   @Override
   public boolean epsilonEquals(Point32 other, double epsilon)
   {
      return super.epsilonEquals(other, epsilon);
   }
}
