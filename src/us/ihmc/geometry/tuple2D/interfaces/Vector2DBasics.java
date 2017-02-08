package us.ihmc.geometry.tuple2D.interfaces;

import us.ihmc.geometry.transform.interfaces.Transform;

public interface Vector2DBasics<T extends Vector2DBasics<T>> extends Tuple2DBasics<T>, Vector2DReadOnly<T>
{
   default void setAndNormalize(Vector2DReadOnly<?> other)
   {
      set(other);
      normalize();
   }

   default void normalize()
   {
      if (containsNaN())
         return;
      scale(1.0 / length());
   }

   @Override
   default void applyTransform(Transform transform)
   {
      transform.transform(this);
   }

   default void applyTransform(Transform transform, boolean checkIfTransformInXYplane)
   {
      transform.transform(this, checkIfTransformInXYplane);
   }
}
