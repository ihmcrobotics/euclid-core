package us.ihmc.geometry.tuple3D;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;

import java.util.Random;

import org.junit.Assert;
import org.junit.Test;

import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.testingTools.GeometryBasicsTestTools;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DReadOnly;

public class Point3D32Test extends Point3DBasicsTest<Point3D32>
{
   private static final double EPS = 1.0e-10;

   @Test
   public void testPoint3D32()
   {
      Random random = new Random(621541L);

      { // Test Point32()
         Point3D32 point = new Point3D32();

         Assert.assertTrue(0 == point.getX());
         Assert.assertTrue(0 == point.getY());
         Assert.assertTrue(0 == point.getZ());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Point32(float x, float y, float z)
         Point3D32 point;

         float newX = random.nextFloat();
         float newY = random.nextFloat();
         float newZ = random.nextFloat();

         point = new Point3D32(newX, newY, newZ);

         Assert.assertTrue(newX == point.getX32());
         Assert.assertTrue(newY == point.getY32());
         Assert.assertTrue(newZ == point.getZ32());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Point32(float[] pointArray)
         float[] randomPoint32Array = {random.nextFloat(), random.nextFloat(), random.nextFloat()};

         Point3D32 point = new Point3D32(randomPoint32Array);

         Assert.assertTrue(randomPoint32Array[0] == point.getX32());
         Assert.assertTrue(randomPoint32Array[1] == point.getY32());
         Assert.assertTrue(randomPoint32Array[2] == point.getZ32());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Point32(TupleBasics tuple)
         Point3D32 point;
         Point3D32 point2 = GeometryBasicsRandomTools.generateRandomPoint3D32(random);
         point = new Point3D32((Tuple3DReadOnly<?>) point2);
         GeometryBasicsTestTools.assertTuple3DEquals(point, point2, EPS);
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      Point3D32 tuple1 = createRandomTuple(random);

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

   @Override
   public Point3D32 createEmptyTuple()
   {
      return new Point3D32();
   }

   @Override
   public Point3D32 createRandomTuple(Random random)
   {
      return GeometryBasicsRandomTools.generateRandomPoint3D32(random);
   }

   @Override
   public Point3D32 createTuple(double x, double y, double z)
   {
      return new Point3D32((float) x, (float) y, (float) z);
   }

   @Override
   public double getEpsilon()
   {
      return 2.0e-7;
   }
}
