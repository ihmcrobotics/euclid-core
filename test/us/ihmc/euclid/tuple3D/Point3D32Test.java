package us.ihmc.euclid.tuple3D;

import org.junit.Assert;
import org.junit.Test;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;

import java.util.Random;

import static org.junit.Assert.*;

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
         Point3D32 point2 = EuclidCoreRandomTools.generateRandomPoint3D32(random);
         point = new Point3D32(point2);
         EuclidCoreTestTools.assertTuple3DEquals(point, point2, EPS);
      }
   }

   @Override
   public void testSetters() throws Exception
   {
      super.testSetters();

      Random random = new Random(621541L);
      Point3D32 tuple1 = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test setX(float x)
         float x = random.nextFloat();
         tuple1.setX(x);
         assertEquals(tuple1.getX32(), x, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test setY(float y)
         float y = random.nextFloat();
         tuple1.setY(y);
         assertEquals(tuple1.getY32(), y, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test setZ(float z)
         float z = random.nextFloat();
         tuple1.setZ(z);
         assertEquals(tuple1.getZ32(), z, getEpsilon());
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
         tuple1.setElement(i % 3, random.nextFloat());
         newHashCode = tuple1.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }

   @Test
   public void testGeometricallyEquals() throws Exception {
      super.testGeometricallyEquals();

      Point3D32 pointA;
      Point3D32 pointB;
      Random random = new Random(621541L);

      for (int i = 0; i < 100; ++i) {
         double epsilon = random.nextDouble();
         pointA = EuclidCoreRandomTools.generateRandomPoint3D32(random);
         pointB = EuclidCoreRandomTools.generateRandomPoint3D32(random);

         if (pointA.epsilonEquals(pointB, getEpsilon())) {
            assertTrue(pointA.geometricallyEquals(pointB, Math.sqrt(3)*getEpsilon()));
         } else {
            if (Math.sqrt((pointA.getX() - pointB.getX()) * (pointA.getX() - pointB.getX()) + (pointA.getY() - pointB.getY()) * (pointA.getY() - pointB.getY()) + (pointA.getZ() - pointB.getZ()) * (pointA.getZ() - pointB.getZ())) <= getEpsilon()) {
               assertTrue(pointA.geometricallyEquals(pointB, getEpsilon()));
            } else {
               assertFalse(pointA.geometricallyEquals(pointB, getEpsilon()));
            }
         }

         pointA = EuclidCoreRandomTools.generateRandomPoint3D32(random);

         pointB = new Point3D32(pointA);
         Vector3D perturb = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, 0.99 * epsilon);
         pointB.add(perturb);

         assertTrue(pointA.geometricallyEquals(pointB, epsilon));

         pointB = new Point3D32(pointA);
         perturb = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, 1.01 * epsilon);
         pointB.add(perturb);

         assertFalse(pointA.geometricallyEquals(pointB, epsilon));
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
      return EuclidCoreRandomTools.generateRandomPoint3D32(random);
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
