package us.ihmc.geometry.tuple3D.interfaces;

import us.ihmc.geometry.transform.AffineTransform;
import us.ihmc.geometry.transform.QuaternionBasedTransform;
import us.ihmc.geometry.transform.RigidBodyTransform;
import us.ihmc.geometry.transform.interfaces.Transform;

/**
 * Write and read interface for a 3 dimensional vector.
 * <p>
 * A 3D vector represents a physical quantity with a magnitude and a direction. For instance, it can
 * be used to represent a 3D velocity, force, or translation from one 3D point to another.
 * </p>
 * <p>
 * Although a point and vector hold onto the same type of information, the distinction is made
 * between them as they represent different geometry objects and are typically not handled the same
 * way:
 * <ul>
 * <li>a point represents the coordinate of a location in space. A notable difference with a vector
 * is that the distance between two points has a physical meaning. When a point is transformed with
 * a homogeneous transformation matrix, a point's coordinates are susceptible to be scaled, rotated,
 * and translated.
 * <li>a vector is not constrained to a location in space. Instead, a vector represents some
 * physical quantity that has a direction and a magnitude such as: a velocity, a force, the
 * translation from one point to another, etc. When a vector is transformed with a homogeneous
 * transformation matrix, its components are susceptible to be scaled and rotated, but never to be
 * translated.
 * </ul>
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 * @param <T> The final type of the vector used.
 */
public interface Vector3DBasics<T extends Vector3DBasics<T>> extends Tuple3DBasics<T>, Vector3DReadOnly<T>
{
   /**
    * Normalizes this vector such that its magnitude is equal to 1 after calling this method and its
    * direction remains unchanged.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if this vector contains {@link Double#NaN}, this method is ineffective.
    * </ul>
    * </p>
    */
   default void normalize()
   {
      if (containsNaN())
         return;
      scale(1.0 / length());
   }

   /**
    * Sets this vector to {@code other} and then calls {@link #normalize()}.
    * 
    * @param other the other vector to copy the values from. Not modified.
    */
   default void setAndNormalize(Tuple3DReadOnly<?> other)
   {
      set(other);
      normalize();
   }

   /**
    * Sets this vector to the cross product of {@code vector1} and {@code vector2}.
    * <p>
    * this = vector1 {@code x} vector2
    * </p>
    * 
    * @param vector1 the first vector in the cross product. Not modified.
    * @param vector2 the second vector in the cross product. Not modified.
    */
   default void cross(Vector3DReadOnly<?> vector1, Vector3DReadOnly<?> vector2)
   {
      double x = vector1.getY() * vector2.getZ() - vector1.getZ() * vector2.getY();
      double y = vector1.getZ() * vector2.getX() - vector1.getX() * vector2.getZ();
      double z = vector1.getX() * vector2.getY() - vector1.getY() * vector2.getX();
      set(x, y, z);
   }

   /**
    * Transforms this vector by the given {@code transform}.
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates a vector.
    * <li>{@link QuaternionBasedTransform} rotates a vector.
    * <li>{@link AffineTransform} scales then rotates a vector.
    * </ul>
    * </p>
    * 
    * @param transform the geometric transform to apply on this vector. Not modified.
    */
   @Override
   default void applyTransform(Transform transform)
   {
      transform.transform(this);
   }
}
