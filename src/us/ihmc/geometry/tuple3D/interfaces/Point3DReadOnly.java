package us.ihmc.geometry.tuple3D.interfaces;

public interface Point3DReadOnly<T extends Point3DReadOnly<T>> extends Tuple3DReadOnly<T>
{
   default double distance(Point3DReadOnly<?> other)
   {
      return Math.sqrt(distanceSquared(other));
   }

   default double distanceSquared(Point3DReadOnly<?> other)
   {
      double dx = getX() - other.getX();
      double dy = getY() - other.getY();
      double dz = getZ() - other.getZ();
      return dx * dx + dy * dy + dz * dz;
   }
}
