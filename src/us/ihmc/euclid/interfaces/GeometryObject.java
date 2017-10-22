package us.ihmc.euclid.interfaces;

/**
 * Base interface for any geometry object. A {@code GeometryObject} has to be {@code Transformable},
 * {@code EpsilonComparable}, and {@code Settable}.
 *
 * @author Sylvain
 *
 * @param <T> the final type of the implementation of this interface.
 */
public interface GeometryObject<T extends GeometryObject<T>> extends Transformable, EpsilonComparable<T>, Settable<T>, Clearable
{
   /**
    * Tests if {@code this} and {@code other} represent the same geometry object to an
    * {@code epsilon}.
    * <p>
    * The implementation of this test depends on the type of geometry. For instance, two points will
    * be considered geometrically equal if they are at a distance from each other that is less or
    * equal than {@code epsilon}.
    * </p>
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * {@code this.epsilonEquals(other, epsilon)} and vice versa.
    * </p>
    *
    * @param other the other geometry object to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing the two objects, usually refers to a distance.
    * @return {@code true} if the two objects represent the same geometry, {@code false} otherwise.
    */
   boolean geometricallyEquals(T other, double epsilon);
}
