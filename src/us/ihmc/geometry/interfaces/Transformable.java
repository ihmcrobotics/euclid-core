package us.ihmc.geometry.interfaces;

import us.ihmc.geometry.transform.interfaces.Transform;

/**
 * Base interface for any object that can be transformed with a {@link Transform}.
 * 
 * @author Sylvain
 *
 */
public interface Transformable
{
   /**
    * Transform this using the given {@code transform}.
    * 
    * @param transform the transform to use on this. Not modified.
    */
   public abstract void applyTransform(Transform transform);
}
