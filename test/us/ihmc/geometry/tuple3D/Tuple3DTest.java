package us.ihmc.geometry.tuple3D;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;

import java.util.Random;

import org.junit.Test;

public abstract class Tuple3DTest<T extends Tuple3D<T>> extends Tuple3DBasicsTest<T>
{
   public static final int NUMBER_OF_ITERATIONS = 100;

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D tuple1 = createEmptyTuple();

      tuple1.setX(random.nextDouble());
      tuple1.setY(random.nextDouble());
      tuple1.setZ(random.nextDouble());

      int newHashCode, previousHashCode;
      newHashCode = tuple1.hashCode();
      assertEquals(newHashCode, tuple1.hashCode());

      previousHashCode = tuple1.hashCode();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         tuple1.set(i % 3, random.nextDouble());
         newHashCode = tuple1.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }
}
