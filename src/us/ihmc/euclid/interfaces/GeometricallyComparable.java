package us.ihmc.euclid.interfaces;

public interface GeometricallyComparable<T>
{
   /**
    * Tests if {@code this} and {@code other} represent the same geometry to an {@code epsilon}.
    * <p>
    * The implementation of this test depends on the type of geometry. For instance, two points will
    * be considered geometrically equal if they are at a distance from each other that is less or
    * equal than {@code epsilon}.
    * </p>
    *
    * @param other the other geometry object to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing the two objects, usually refers to a distance.
    * @return {@code true} if the two objects represent the same geometry, {@code false} otherwise.
    */
   boolean geometricallyEquals(T other, double epsilon);
}