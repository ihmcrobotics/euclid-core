package us.ihmc.geometry.tuple2D;

import java.io.Serializable;

import us.ihmc.geometry.transform.interfaces.Transform;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.geometry.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Vector2DReadOnly;

public class Vector2D32 extends Tuple2D32<Vector2D32> implements Serializable, Vector2DBasics<Vector2D32>
{
   private static final long serialVersionUID = 6380132073713315352L;

   public Vector2D32()
   {
      super();
   }

   public Vector2D32(float x, float y)
   {
      super(x, y);
   }

   public Vector2D32(float[] vectorArray)
   {
      super(vectorArray);
   }

   public Vector2D32(Tuple2DReadOnly<?> other)
   {
      super(other);
   }

   @Override
   public void set(Vector2D32 other)
   {
      set((Tuple2DReadOnly<?>) other);
   }

   public void setAndNormalize(Vector2DReadOnly<?> other)
   {
      set(other);
      normalize();
   }

   public float angle(Vector2D32 other)
   {
      double vDot = dot(other) / (length() * other.length());
      return (float) Math.acos(Math.min(1.0, Math.max(-1.0, vDot)));
   }

   public float cross(Vector2DReadOnly<?> other)
   {
      return cross(this, other);
   }

   public static float cross(Vector2DReadOnly<Vector2D32> v1, Vector2DReadOnly<?> v2)
   {
      return (float) (v1.getX() * v2.getY() - v1.getY() * v2.getX());
   }

   public float dot(Vector2DReadOnly<Vector2D32> other)
   {
      return (float) (getX() * other.getX() + getY() * other.getY());
   }

   public float length()
   {
      return (float) Math.sqrt(lengthSquared());
   }

   public float lengthSquared()
   {
      return dot(this);
   }

   public void normalize()
   {
      if (containsNaN())
         return;
      scale(1.0f / length());
   }

   @Override
   public void applyTransform(Transform transform)
   {
      transform.transform(this);
   }
}
