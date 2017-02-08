package us.ihmc.geometry.tuple3D.interfaces;

import us.ihmc.geometry.transform.interfaces.Transform;

public interface Point3DBasics<T extends Point3DBasics<T>> extends Tuple3DBasics<T>, Point3DReadOnly<T>
{
   @Override
   default void applyTransform(Transform transform)
   {
      transform.transform(this);
   }
}
