package us.ihmc.geometry.tuple3D;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;

import java.util.Random;

import org.junit.Test;

import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;

public abstract class Tuple3D32Test<T extends Tuple3D32<T>> extends Tuple3DBasicsTest<T>
{
   public static final int NUMBER_OF_ITERATIONS = Tuple3DTest.NUMBER_OF_ITERATIONS;

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D32 tuple1 = GeometryBasicsRandomTools.generateRandomPoint32(random);

      int newHashCode, previousHashCode;
      newHashCode = tuple1.hashCode();
      assertEquals(newHashCode, tuple1.hashCode());

      previousHashCode = tuple1.hashCode();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         tuple1.set(i % 3, random.nextFloat());
         newHashCode = tuple1.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }
}
