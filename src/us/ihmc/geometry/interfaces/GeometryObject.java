package us.ihmc.geometry.interfaces;

/**
 * Base interface for any geometry object. A {@code GeometryObject} has to be {@code Transformable},
 * {@code EpsilonComparable}, and {@code Settable}.
 *
 * @author Sylvain
 *
 * @param <T> the final type of the implementation of this interface.
 */
public interface GeometryObject<T extends GeometryObject<T>> extends Transformable, EpsilonComparable<T>, Settable<T>
{

}
