package us.ihmc.geometry.tuple3D;

import java.util.Random;

import us.ihmc.geometry.tuple3D.interfaces.Tuple3DReadOnly;

public abstract class Tuple3DReadOnlyTest<T extends Tuple3DReadOnly<T>>
{
   public static final int NUMBER_OF_ITERATIONS = 100;

   public abstract T createEmptyTuple();

   public abstract T createTuple(double x, double y, double z);

   public abstract T createRandomTuple(Random random);

   public abstract double getEpsilon();

}