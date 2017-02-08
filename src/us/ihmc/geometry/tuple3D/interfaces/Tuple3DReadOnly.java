package us.ihmc.geometry.tuple3D.interfaces;

import us.ihmc.geometry.interfaces.EpsilonComparable;
import us.ihmc.geometry.tuple3D.Tuple3DTools;

public interface Tuple3DReadOnly<T extends Tuple3DReadOnly<T>> extends EpsilonComparable<T>
{

   double get(int index);

   double getX();

   double getY();

   double getZ();

   @Override
   default boolean epsilonEquals(T other, double epsilon)
   {
      return Tuple3DTools.epsilonEquals(this, other, epsilon);
   }
}