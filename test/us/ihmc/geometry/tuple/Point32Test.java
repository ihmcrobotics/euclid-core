package us.ihmc.geometry.tuple;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Assert;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.testingTools.GeometryBasicsTestTools;
import us.ihmc.geometry.tuple3D.Point3D32;
import us.ihmc.geometry.tuple3D.Tuple32;
import us.ihmc.geometry.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DReadOnly;

public class Point32Test extends Tuple32Test
{
   private static final double EPS = 1.0e-10;

   @Test
   public void testPoint32()
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
         Point3D32 point2 = GeometryBasicsRandomTools.generateRandomPoint32(random);
         point = new Point3D32((Tuple3DReadOnly) point2);
         GeometryBasicsTestTools.assertTupleEquals(point, point2, EPS);
      }
   }

   @Test
   public void testSet()
   {
      Random random = new Random(65465131L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         float newX = random.nextFloat();
         float newY = random.nextFloat();
         float newZ = random.nextFloat();

         Point3D32 point = new Point3D32(newX, newY, newZ);

         Point3D32 point2 = new Point3D32();
         point2.set(point);
         GeometryBasicsTestTools.assertTupleEquals(point, point2, EPS);
      }
   }

   @Test
   public void testDistance()
   {
      Random random = new Random(654135L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         float newX1 = random.nextFloat();
         float newY1 = random.nextFloat();
         float newZ1 = random.nextFloat();

         float newX2 = random.nextFloat();
         float newY2 = random.nextFloat();
         float newZ2 = random.nextFloat();

         Point3D32 point = new Point3D32(newX1, newY1, newZ1);
         Point3D32 point2 = new Point3D32(newX2, newY2, newZ2);

         float distance = (float) Math.sqrt(point.distanceSquared((Point3DReadOnly) point2));
         float distance2 = point.distance(point2);

         Assert.assertTrue(distance == distance2);
      }
   }

   @Test
   @Ignore
   public void testDistanceL1()
   {
      fail("Not yet implemented");
   }

   @Test
   @Ignore
   public void testDistanceLinf()
   {
      fail("Not yet implemented");
   }

   @Test
   public void testDistanceSquared()
   {
      Random random = new Random(654135L);
      float newX1 = random.nextFloat();
      float newY1 = random.nextFloat();
      float newZ1 = random.nextFloat();

      float newX2 = random.nextFloat();
      float newY2 = random.nextFloat();
      float newZ2 = random.nextFloat();

      Point3D32 point = new Point3D32(newX1, newY1, newZ1);
      Point3D32 point2 = new Point3D32(newX2, newY2, newZ2);

      float distance = point.distanceSquared((Point3DReadOnly) point2);
      float dx = point.getX32() - point2.getX32();
      float dy = point.getY32() - point2.getY32();
      float dz = point.getZ32() - point2.getZ32();
      float expectedDistance = dx * dx + dy * dy + dz * dz;

      Assert.assertEquals(distance, expectedDistance, EPS);
   }

   @Test
   @Ignore
   public void testApplyTransform()
   {
      fail("Not yet implemented");
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(621541L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Point3D32 point1 = new Point3D32();
         Point3D32 point2 = new Point3D32();

         float epsilon = random.nextFloat();

         GeometryBasicsRandomTools.randomizeTuple(random, point1);

         point2.setX(point1.getX32() + 0.1f * epsilon);
         point2.setY(point1.getY32() + 0.1f * epsilon);
         point2.setZ(point1.getZ32() + 0.1f * epsilon);
         assertTrue(point1.epsilonEquals(point2, epsilon));

         point2.setX(point1.getX32() - 0.1f * epsilon);
         point2.setY(point1.getY32() - 0.1f * epsilon);
         point2.setZ(point1.getZ32() - 0.1f * epsilon);
         assertTrue(point1.epsilonEquals(point2, epsilon));

         point2.setX(point1.getX32() - 1.1f * epsilon);
         point2.setY(point1.getY32() - 0.1f * epsilon);
         point2.setZ(point1.getZ32() - 0.1f * epsilon);
         assertFalse(point1.epsilonEquals(point2, epsilon));

         point2.setX(point1.getX32() - 0.1f * epsilon);
         point2.setY(point1.getY32() - 1.1f * epsilon);
         point2.setZ(point1.getZ32() - 0.1f * epsilon);
         assertFalse(point1.epsilonEquals(point2, epsilon));

         point2.setX(point1.getX32() - 0.1f * epsilon);
         point2.setY(point1.getY32() - 0.1f * epsilon);
         point2.setZ(point1.getZ32() - 1.1f * epsilon);
         assertFalse(point1.epsilonEquals(point2, epsilon));

         point2.setX(point1.getX32() + 1.1f * epsilon);
         point2.setY(point1.getY32() + 0.1f * epsilon);
         point2.setZ(point1.getZ32() + 0.1f * epsilon);
         assertFalse(point1.epsilonEquals(point2, epsilon));

         point2.setX(point1.getX32() + 0.1f * epsilon);
         point2.setY(point1.getY32() + 1.1f * epsilon);
         point2.setZ(point1.getZ32() + 0.1f * epsilon);
         assertFalse(point1.epsilonEquals(point2, epsilon));

         point2.setX(point1.getX32() + 0.1f * epsilon);
         point2.setY(point1.getY32() + 0.1f * epsilon);
         point2.setZ(point1.getZ32() + 1.1f * epsilon);
         assertFalse(point1.epsilonEquals(point2, epsilon));

         point2.setX(point1.getX32() + epsilon - (float) 1.0e-7);
         point2.setY(point1.getY32() + epsilon - (float) 1.0e-7);
         point2.setZ(point1.getZ32() + epsilon - (float) 1.0e-7);
         assertTrue(point1.epsilonEquals(point2, epsilon));

         point2.setX(point1.getX32() - epsilon + (float) 1.0e-7);
         point2.setY(point1.getY32() - epsilon + (float) 1.0e-7);
         point2.setZ(point1.getZ32() - epsilon + (float) 1.0e-7);
         assertTrue(point1.epsilonEquals(point2, epsilon));
      }
   }

   @Override
   public Tuple32 createEmptyTuple32()
   {
      return new Point3D32();
   }
}
