package us.ihmc.geometry.interfaces;

import us.ihmc.geometry.transform.interfaces.Transform;

public interface Transformable
{
   public abstract void applyTransform(Transform transform);
}
