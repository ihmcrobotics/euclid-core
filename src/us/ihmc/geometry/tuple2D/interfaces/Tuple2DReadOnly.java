package us.ihmc.geometry.tuple2D.interfaces;

import us.ihmc.geometry.interfaces.EpsilonComparable;

public interface Tuple2DReadOnly<T extends Tuple2DReadOnly<T>> extends EpsilonComparable<T>
{

   double get(int index);

   double getX();

   double getY();

}