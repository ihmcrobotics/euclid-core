package us.ihmc.geometry.tuple3D.interfaces;

public interface Vector3DReadOnly<T extends Vector3DReadOnly<T>> extends Tuple3DReadOnly<T>
{
   default double length()
   {
      return Math.sqrt(lengthSquared());
   }

   default double lengthSquared()
   {
      return dot(this);
   }

   default double dot(Vector3DReadOnly<?> other)
   {
      return getX() * other.getX() + getY() * other.getY() + getZ() * other.getZ();
   }

   default double angle(Vector3DReadOnly<?> other)
   {
      double normalizedDot = dot(other) / (length() * other.length());
      return Math.acos(Math.min(1.0, Math.max(-1.0, normalizedDot)));
   }
}
