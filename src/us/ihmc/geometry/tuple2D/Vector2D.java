package us.ihmc.geometry.tuple2D;

import java.io.Serializable;

import us.ihmc.geometry.transform.interfaces.Transform;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.geometry.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Vector2DReadOnly;

public class Vector2D extends Tuple2D<Vector2D> implements Serializable, Vector2DBasics<Vector2D>
{
   private static final long serialVersionUID = -1422872858238666884L;

   public Vector2D()
   {
      super();
   }

   public Vector2D(double x, double y)
   {
      super(x, y);
   }

   public Vector2D(double[] vectorArray)
   {
      super(vectorArray);
   }

   public Vector2D(Tuple2DReadOnly<?> other)
   {
      super(other);
   }

   @Override
   public void set(Vector2D other)
   {
      set((Tuple2DReadOnly<?>) other);
   }

   public void setAndNormalize(Vector2DReadOnly<?> other)
   {
      set(other);
      normalize();
   }

   public double angle(Vector2D other)
   {
      double vDot = dot(other) / (length() * other.length());
      return Math.acos(Math.min(1.0, Math.max(-1.0, vDot)));
   }

   public double cross(Vector2DReadOnly<?> other)
   {
      return cross(this, other);
   }

   public static double cross(Vector2DReadOnly<Vector2D> v1, Vector2DReadOnly<?> v2)
   {
      return v1.getX() * v2.getY() - v1.getY() * v2.getX();
   }

   public double dot(Vector2DReadOnly<Vector2D> other)
   {
      return getX() * other.getX() + getY() * other.getY();
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
   public boolean epsilonEquals(Vector2D other, double epsilon)
   {
      return epsilonEquals((Tuple2DReadOnly<?>) other, epsilon);
   }
}
