package us.ihmc.geometry.tuple;

import java.io.Serializable;

import us.ihmc.geometry.interfaces.GeometryObject;
import us.ihmc.geometry.transform.interfaces.Transform;
import us.ihmc.geometry.tuple.interfaces.Tuple3DReadOnly;
import us.ihmc.geometry.tuple.interfaces.VectorBasics;
import us.ihmc.geometry.tuple.interfaces.VectorReadOnly;

public class Vector extends Tuple implements Serializable, VectorBasics, GeometryObject<Vector>
{
   private static final long serialVersionUID = -8494204802892437237L;

   public Vector()
   {
      super();
   }

   public Vector(double x, double y, double z)
   {
      super(x, y, z);
   }

   public Vector(double[] vectorArray)
   {
      super(vectorArray);
   }

   public Vector(Tuple3DReadOnly other)
   {
      super(other);
   }

   @Override
   public void set(Vector other)
   {
      super.set(other);
   }

   public void setAndNormalize(Vector other)
   {
      set(other);
      normalize();
   }

   public double angle(Vector other)
   {
      double vDot = dot(other) / (length() * other.length());
      return Math.acos(Math.min(1.0, Math.max(-1.0, vDot)));
   }

   public void cross(VectorReadOnly v1, VectorReadOnly v2)
   {
      double x = v1.getY() * v2.getZ() - v1.getZ() * v2.getY();
      double y = v1.getZ() * v2.getX() - v1.getX() * v2.getZ();
      double z = v1.getX() * v2.getY() - v1.getY() * v2.getX();
      set(x, y, z);
   }

   public double dot(VectorReadOnly other)
   {
      return getX() * other.getX() + getY() * other.getY() + getZ() * other.getZ();
   }

   public double length()
   {
      return Math.sqrt(lengthSquared());
   }

   public double lengthSquared()
   {
      return dot(this);
   }

   public void normalize()
   {
      if (containsNaN())
         return;
      scale(1.0 / length());
   }

   @Override
   public void applyTransform(Transform transform)
   {
      transform.transform(this);
   }

   @Override
   public boolean epsilonEquals(Vector other, double epsilon)
   {
      return super.epsilonEquals(other, epsilon);
   }
}
