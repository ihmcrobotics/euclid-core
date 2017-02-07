package us.ihmc.geometry.tuple2D;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;

import java.util.Random;

import org.junit.Assert;
import org.junit.Test;

import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.tuple3D.Tuple3DTest;

public abstract class Tuple2D32Test<T extends Tuple2D32<T>> extends Tuple2DBasicsTest<T>
{
   public static final int NUMBER_OF_ITERATIONS = Tuple3DTest.NUMBER_OF_ITERATIONS;

   @Test
   public void testTuple()
   {
      T tuple = createEmptyTuple();
      Assert.assertTrue(tuple.getX32() == 0);
      Assert.assertTrue(tuple.getY32() == 0);
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      T tuple1 = createEmptyTuple();
      GeometryBasicsRandomTools.randomizeTuple2D(random, tuple1);

      int newHashCode, previousHashCode;
      newHashCode = tuple1.hashCode();
      assertEquals(newHashCode, tuple1.hashCode());

      previousHashCode = tuple1.hashCode();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         tuple1.set(i % 2, random.nextFloat());
         newHashCode = tuple1.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-6;
   }
}
