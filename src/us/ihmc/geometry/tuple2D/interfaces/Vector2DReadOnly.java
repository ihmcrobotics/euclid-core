package us.ihmc.geometry.tuple2D.interfaces;

public interface Vector2DReadOnly<T extends Vector2DReadOnly<T>> extends Tuple2DReadOnly<T>
{
   default double length()
   {
      return Math.sqrt(lengthSquared());
   }

   default double lengthSquared()
   {
      return dot(this);
   }

   default double dot(Vector2DReadOnly<?> other)
   {
      return getX() * other.getX() + getY() * other.getY();
   }

   default double angle(Vector2DReadOnly<?> other)
   {
      double vDot = dot(other) / (length() * other.length());
      return Math.acos(Math.min(1.0, Math.max(-1.0, vDot)));
   }

   default double cross(Vector2DReadOnly<?> other)
   {
      return cross(this, other);
   }

   public static double cross(Vector2DReadOnly<?> v1, Vector2DReadOnly<?> v2)
   {
      return v1.getX() * v2.getY() - v1.getY() * v2.getX();
   }
}
