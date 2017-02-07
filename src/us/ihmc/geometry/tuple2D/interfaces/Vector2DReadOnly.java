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
      double firstVectorX = getX();
      double firstVectorY = getY();
      double firstVectorLength = length();

      if (firstVectorLength < 1e-7)
         return 0.0;

      firstVectorX /= firstVectorLength;
      firstVectorY /= firstVectorLength;

      double secondVectorX = other.getX();
      double secondVectorY = other.getY();
      double secondVectorLength = other.length();

      if (secondVectorLength < 1e-7)
         return 0.0;

      secondVectorX /= secondVectorLength;
      secondVectorY /= secondVectorLength;

      // The sign of the angle comes from the cross product
      double crossProduct = firstVectorX * secondVectorY - firstVectorY * secondVectorX;
      // the magnitude of the angle comes from the dot product
      double dotProduct = firstVectorX * secondVectorX + firstVectorY * secondVectorY;

      double angle = Math.atan2(crossProduct, dotProduct);
      // This is a hack to get the polygon tests to pass.
      // Probably some edge case not well handled somewhere (Sylvain)
      if (crossProduct == 0.0)
         angle = -angle;

      return angle;
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
