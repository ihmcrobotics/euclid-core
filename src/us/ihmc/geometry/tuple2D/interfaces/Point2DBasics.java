package us.ihmc.geometry.tuple2D.interfaces;

import us.ihmc.geometry.transform.interfaces.Transform;

public interface Point2DBasics<T extends Point2DBasics<T>> extends Tuple2DBasics<T>, Point2DReadOnly<T>
{
   @Override
   default void applyTransform(Transform transform)
   {
      transform.transform(this);
   }

   default void applyTransform(Transform transform, boolean checkIfTransformInXYplane)
   {
      transform.transform(this);
   }
}
