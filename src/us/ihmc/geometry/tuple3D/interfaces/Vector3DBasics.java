package us.ihmc.geometry.tuple3D.interfaces;

import us.ihmc.geometry.transform.interfaces.Transform;

public interface Vector3DBasics<T extends Vector3DBasics<T>> extends Tuple3DBasics<T>, Vector3DReadOnly<T>
{
   default void normalize()
   {
      if (containsNaN())
         return;
      scale(1.0 / length());
   }

   default void setAndNormalize(Vector3DReadOnly<?> other)
   {
      set(other);
      normalize();
   }

   default void cross(Vector3DReadOnly<?> v1, Vector3DReadOnly<?> v2)
   {
      double x = v1.getY() * v2.getZ() - v1.getZ() * v2.getY();
      double y = v1.getZ() * v2.getX() - v1.getX() * v2.getZ();
      double z = v1.getX() * v2.getY() - v1.getY() * v2.getX();
      set(x, y, z);
   }

   @Override
   default void applyTransform(Transform transform)
   {
      transform.transform(this);
   }

}
