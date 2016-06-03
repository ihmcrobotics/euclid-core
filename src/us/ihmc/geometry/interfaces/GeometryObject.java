package us.ihmc.geometry.interfaces;

public interface GeometryObject<T extends GeometryObject<T>> extends Transformable, EpsilonComparable<T>, Settable<T>
{

}
