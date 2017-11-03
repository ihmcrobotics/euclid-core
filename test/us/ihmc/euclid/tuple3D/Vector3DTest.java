package us.ihmc.euclid.tuple3D;

import org.junit.Assert;
import org.junit.Test;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;

import java.util.Random;

import static org.junit.Assert.*;

public class Vector3DTest extends Vector3DBasicsTest<Vector3D>
{
   @Test
   public void testVector()
   {
      Random random = new Random(621541L);
      Vector3D vector = new Vector3D();

      { // Test Vector()
         Assert.assertTrue(0 == vector.getX());
         Assert.assertTrue(0 == vector.getY());
         Assert.assertTrue(0 == vector.getZ());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Vector(double x, double y, double z)
         double newX = random.nextDouble();
         double newY = random.nextDouble();
         double newZ = random.nextDouble();

         vector = new Vector3D(newX, newY, newZ);

         Assert.assertTrue(newX == vector.getX());
         Assert.assertTrue(newY == vector.getY());
         Assert.assertTrue(newZ == vector.getZ());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Vector(double[] vectorArray)
         double[] randomVectorArray = {random.nextDouble(), random.nextDouble(), random.nextDouble()};
         Vector3D vectorArray = new Vector3D(randomVectorArray);

         Assert.assertTrue(randomVectorArray[0] == vectorArray.getX());
         Assert.assertTrue(randomVectorArray[1] == vectorArray.getY());
         Assert.assertTrue(randomVectorArray[2] == vectorArray.getZ());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Vector(TupleBasics tuple)
         Vector3D vector2 = new Vector3D();
         vector2.setX(random.nextDouble());
         vector2.setY(random.nextDouble());
         vector2.setZ(random.nextDouble());

         vector = new Vector3D(vector2);

         Assert.assertTrue(vector.getX() == vector2.getX());
         Assert.assertTrue(vector.getY() == vector2.getY());
         Assert.assertTrue(vector.getZ() == vector2.getZ());
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      Vector3D tuple1 = createRandomTuple(random);

      int newHashCode, previousHashCode;
      newHashCode = tuple1.hashCode();
      assertEquals(newHashCode, tuple1.hashCode());

      previousHashCode = tuple1.hashCode();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         tuple1.setElement(i % 3, random.nextDouble());
         newHashCode = tuple1.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }

   @Test
   public void testGeometricallyEquals() throws Exception
   {
      super.testGeometricallyEquals();

      Vector3D vectorA;
      Vector3D vectorB;
      Random random = new Random(621541L);

      for (int i = 0; i < 100; ++i)
      {
         vectorA = EuclidCoreRandomTools.generateRandomVector3D(random);
         vectorB = EuclidCoreRandomTools.generateRandomVector3D(random);

         if (vectorA.epsilonEquals(vectorB, getEpsilon()))
         {
            assertTrue(vectorA.geometricallyEquals(vectorB, Math.sqrt(3) * getEpsilon()));
         }
         else
         {
            if (Math.sqrt((vectorA.getX() - vectorB.getX()) * (vectorA.getX() - vectorB.getX()) + (vectorA.getY() - vectorB.getY()) * (vectorA.getY() - vectorB
                  .getY()) + (vectorA.getZ() - vectorB.getZ()) * (vectorA.getZ() - vectorB.getZ())) <= getEpsilon())
            {
               assertTrue(vectorA.geometricallyEquals(vectorB, getEpsilon()));
            }
            else
            {
               assertFalse(vectorA.geometricallyEquals(vectorB, getEpsilon()));
            }
         }

         vectorA = EuclidCoreRandomTools.generateRandomVector3D(random);
         vectorB = new Vector3D(vectorA);

         assertTrue(vectorA.geometricallyEquals(vectorB, 0));

         vectorB.set(vectorA.getX() + 0.9d * getEpsilon(), vectorA.getY(), vectorA.getZ());

         assertTrue(vectorA.geometricallyEquals(vectorB, getEpsilon()));

         vectorB.set(vectorA.getX() + 1.1d * getEpsilon(), vectorA.getY(), vectorA.getZ());

         assertFalse(vectorA.geometricallyEquals(vectorB, getEpsilon()));

         vectorB.set(vectorA.getX(), vectorA.getY() + 0.9d * getEpsilon(), vectorA.getZ());

         assertTrue(vectorA.geometricallyEquals(vectorB, getEpsilon()));

         vectorB.set(vectorA.getX(), vectorA.getY() + 1.1d * getEpsilon(), vectorA.getZ());

         assertFalse(vectorA.geometricallyEquals(vectorB, getEpsilon()));

         vectorB.set(vectorA.getX(), vectorA.getY(), vectorA.getZ() + 0.9d * getEpsilon());

         assertTrue(vectorA.geometricallyEquals(vectorB, getEpsilon()));

         vectorB.set(vectorA.getX(), vectorA.getY(), vectorA.getZ() + 1.1d * getEpsilon());

         assertFalse(vectorA.geometricallyEquals(vectorB, getEpsilon()));
      }
   }

   @Override
   public Vector3D createEmptyTuple()
   {
      return new Vector3D();
   }

   @Override
   public Vector3D createRandomTuple(Random random)
   {
      return EuclidCoreRandomTools.generateRandomRotationVector(random);
   }

   @Override
   public Vector3D createTuple(double x, double y, double z)
   {
      return new Vector3D(x, y, z);
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-15;
   }
}
