package us.ihmc.geometry.tuple4D.interfaces;

import us.ihmc.geometry.interfaces.EpsilonComparable;
import us.ihmc.geometry.tuple4D.Tuple4DTools;

public interface Tuple4DReadOnly<T extends Tuple4DReadOnly<T>> extends EpsilonComparable<T>
{
   double getX();

   double getY();

   double getZ();

   double getS();

   @Override
   default boolean epsilonEquals(T other, double epsilon)
   {
      return Tuple4DTools.epsilonEquals(this, other, epsilon);
   }
}