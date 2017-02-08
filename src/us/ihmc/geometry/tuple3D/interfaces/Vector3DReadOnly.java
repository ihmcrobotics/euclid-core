package us.ihmc.geometry.tuple3D.interfaces;

/**
 * Read-only interface for a 3 dimensional vector.
 * <p>
 * A 3D vector represents a physical quantity with a magnitude and a direction.
 * For instance, it can be used to represent a 3D velocity, force, or translation from one 3D point to another.
 * </p>
 * <p>
 * Although a point and vector hold onto the same type of information, the distinction is made between them
 * as they represent different geometry objects and are typically not handled the same way:
 * <ul>
 *    <li> a point represents the coordinate of a location in space.
 *     A notable difference with a vector is that the distance between two points has a physical meaning.
 *     When a point is transformed with a homogeneous transformation matrix,
 *     a point's coordinates are susceptible to be scaled, rotated, and translated.
 *    <li> a vector is not constrained to a location in space. Instead, a vector represents some
 *     physical quantity that has a direction and a magnitude such as: a velocity, a force, the translation from one
 *     point to another, etc.
 *     When a vector is transformed with a homogeneous transformation matrix,
 *     its components are susceptible to be scaled and rotated, but never to be translated.
 * </ul> 
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 * @param <T> The final type of the vector used.
 */
public interface Vector3DReadOnly<T extends Vector3DReadOnly<T>> extends Tuple3DReadOnly<T>
{
   /**
    * Calculates and returns the magnitude of this vector.
    * 
    * @return the magnitude of this vector.
    */
   default double length()
   {
      return Math.sqrt(lengthSquared());
   }

   /**
    * Calculates and returns the square of the magnitude of this vector.
    * <p>
    * This method is usually preferred over {@link #length()}
    * when calculation speed matters and knowledge of the actual magnitude does not,
    * i.e. when comparing several vectors by theirs magnitude.
    * </p>
    * 
    * @return the square of the magnitude of this vector.
    */
   default double lengthSquared()
   {
      return dot(this);
   }

   /**
    * Calculates and returns the value of the dot product of this vector with {@code other}.
    * 
    * @param other the other vector used for the dot product. Not modified.
    * @return the value of the dot product.
    */
   default double dot(Vector3DReadOnly<?> other)
   {
      return getX() * other.getX() + getY() * other.getY() + getZ() * other.getZ();
   }

   /**
    * Calculates and returns the angle in radians from this vector to {@code other}.
    * <p>
    * The computed angle is in the range [0; <i>pi</i>].
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    *    <li> if the length of either vector is below {@code 1.0E-7}, this method fails and returns an angle of {@code 0.0} radian.
    * </ul>
    * </p>
    * 
    * @param other the other vector used to compute the angle. Not modified.
    * @return the value of the angle from this vector to {@code other}.
    */
   default double angle(Vector3DReadOnly<?> other)
   {
      double thisLength = length();

      if (thisLength < 1.0e-7)
         return 0.0;

      double otherLength = other.length();

      if (otherLength < 1.0e-7)
         return 0.0;

      double normalizedDot = dot(other) / (thisLength * otherLength);

      return Math.acos(Math.min(1.0, Math.max(-1.0, normalizedDot)));
   }
}
