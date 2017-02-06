package us.ihmc.geometry.tuple3D;

import java.io.Serializable;

import us.ihmc.geometry.interfaces.GeometryObject;
import us.ihmc.geometry.transform.interfaces.Transform;
import us.ihmc.geometry.tuple3D.interfaces.Point3DBasics;
import us.ihmc.geometry.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DReadOnly;

public class Point32 extends Tuple32 implements Serializable, Point3DBasics, GeometryObject<Point32>
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

   public Point32(Tuple3DReadOnly other)
   {
      super(other);
   }

   @Override
   public void set(Point32 other)
   {
      super.set(other);
   }

   public float distance(Point3DReadOnly other)
   {
      return (float) Math.sqrt(distanceSquared(other));
   }

   public float distanceSquared(Point3DReadOnly other)
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
