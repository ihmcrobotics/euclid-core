package us.ihmc.geometry.tuple3D.interfaces;

/**
 * Read-only interface for a 3 dimensional point.
 * <p>
 * A 3D point represents the 3D coordinates of a location in space.
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
 * @param <T> The final type of the point used.
 */
public interface Point3DReadOnly<T extends Point3DReadOnly<T>> extends Tuple3DReadOnly<T>
{
   /**
    * Calculates and returns the distance between this point
    * and {@code other}.
    * 
    * @param other the other point used to measure the distance.
    * @return the distance between the two points.
    */
   default double distance(Point3DReadOnly<?> other)
   {
      return Math.sqrt(distanceSquared(other));
   }

   /**
    * Calculates and returns the square of the distance between this point
    * and {@code other}.
    * <p>
    * This method is usually preferred over {@link #distance(Point3DReadOnly)}
    * when calculation speed matters and knowledge of the actual distance does not,
    * i.e. when comparing distances between several pairs of points.
    * </p>
    * 
    * @param other the other point used to measure the square of the distance.
    * @return the square of the distance between the two points.
    */
   default double distanceSquared(Point3DReadOnly<?> other)
   {
      double dx = getX() - other.getX();
      double dy = getY() - other.getY();
      double dz = getZ() - other.getZ();
      return dx * dx + dy * dy + dz * dz;
   }
}
