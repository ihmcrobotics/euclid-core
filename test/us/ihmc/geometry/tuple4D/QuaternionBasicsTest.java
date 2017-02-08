package us.ihmc.geometry.tuple4D;

import static org.junit.Assert.*;

import java.util.Random;

import us.ihmc.geometry.testingTools.GeometryBasicsTestTools;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionBasics;

public abstract class QuaternionBasicsTest<T extends QuaternionBasics<T>> extends Tuple4DBasicsTest<T>
{
   @Override
   public void testSetDoubles()
   {
      Random random = new Random(621541L);
      T quaternion = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test set(double x, double y, double z, double s);
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         double s = random.nextDouble();
         quaternion.set(x, y, z, s);

         // The method should normalize, so assertNotEquals is used.
         assertNotEquals(quaternion.getX(), x, getEpsilon());
         assertNotEquals(quaternion.getY(), y, getEpsilon());
         assertNotEquals(quaternion.getZ(), z, getEpsilon());
         assertNotEquals(quaternion.getS(), s, getEpsilon());
         assertEquals(1.0, quaternion.length(), getEpsilon());

         T original = createRandomTuple(random);
         x = original.getX();
         y = original.getY();
         z = original.getZ();
         s = original.getS();
         quaternion.set(x, y, z, s);
         GeometryBasicsTestTools.assertQuaternionEquals(original, quaternion, getEpsilon());
      }
   }
}