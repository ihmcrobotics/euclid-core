package us.ihmc.euclid.tuple2D;

import org.junit.Assert;
import org.junit.Test;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;

import java.util.Random;

import static org.junit.Assert.*;

public class Vector2D32Test extends Vector2DBasicsTest<Vector2D32>
{
   @Test
   public void testVector2D32()
   {
      Random random = new Random(621541L);
      Vector2D32 vector = new Vector2D32();

      { // Test Vector2D32()
         Assert.assertTrue(0 == vector.getX32());
         Assert.assertTrue(0 == vector.getY32());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Vector2D32(float x, float y, float z)
         float newX = random.nextFloat();
         float newY = random.nextFloat();

         vector = new Vector2D32(newX, newY);

         Assert.assertTrue(newX == vector.getX32());
         Assert.assertTrue(newY == vector.getY32());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Vector2D32(float[] vectorArray)
         float[] randomVector2DArray = {random.nextFloat(), random.nextFloat()};

         Vector2D32 vectorArray = new Vector2D32(randomVector2DArray);

         Assert.assertTrue(randomVector2DArray[0] == vectorArray.getX32());
         Assert.assertTrue(randomVector2DArray[1] == vectorArray.getY32());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Vector2D32(TupleBasics tuple)
         Vector2D32 vector2 = EuclidCoreRandomTools.generateRandomVector2D32(random);
         vector = new Vector2D32(vector2);

         Assert.assertTrue(vector.getX32() == vector2.getX32());
         Assert.assertTrue(vector.getY32() == vector2.getY32());
      }
   }

   @Override
   public void testSetters() throws Exception
   {
      super.testSetters();

      Random random = new Random(621541L);
      Vector2D32 tuple1 = createEmptyTuple();

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
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      Point2D32 point = EuclidCoreRandomTools.generateRandomPoint2D32(random);

      int newHashCode, previousHashCode;
      newHashCode = point.hashCode();
      assertEquals(newHashCode, point.hashCode());

      previousHashCode = point.hashCode();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         point.setElement(i % 2, random.nextFloat());
         newHashCode = point.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }

   @Test
   public void testGeometricallyEquals() throws Exception
   {
      super.testGeometricallyEquals();

      Vector2D32 vectorA;
      Vector2D32 vectorB;
      Random random = new Random(621541L);

      for (int i = 0; i < 100; ++i)
      {
         vectorA = EuclidCoreRandomTools.generateRandomVector2D32(random);
         vectorB = EuclidCoreRandomTools.generateRandomVector2D32(random);

         if (vectorA.epsilonEquals(vectorB, getEpsilon()))
         {
            assertTrue(vectorA.geometricallyEquals(vectorB, Math.sqrt(3) * getEpsilon()));
         }
         else
         {
            if (Math.sqrt(EuclidCoreTools.normSquared(vectorA.getX() - vectorB.getX(), vectorA.getY() - vectorB.getY())) <= getEpsilon())
            {
               assertTrue(vectorA.geometricallyEquals(vectorB, getEpsilon()));
            }
            else
            {
               assertFalse(vectorA.geometricallyEquals(vectorB, getEpsilon()));
            }
         }

         vectorA = EuclidCoreRandomTools.generateRandomVector2D32(random);
         vectorB = new Vector2D32(vectorA);

         assertTrue(vectorA.geometricallyEquals(vectorB, 0));

         vectorB.set(vectorA.getX() + 0.9d * getEpsilon(), vectorA.getY());

         assertTrue(vectorA.geometricallyEquals(vectorB, getEpsilon()));

         vectorB.set(vectorA.getX() + 1.1d * getEpsilon(), vectorA.getY());

         assertFalse(vectorA.geometricallyEquals(vectorB, getEpsilon()));

         vectorB.set(vectorA.getX(), vectorA.getY() + 0.9d * getEpsilon());

         assertTrue(vectorA.geometricallyEquals(vectorB, getEpsilon()));

         vectorB.set(vectorA.getX(), vectorA.getY() + 1.1d * getEpsilon());

         assertFalse(vectorA.geometricallyEquals(vectorB, getEpsilon()));
      }
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-6;
   }

   @Override
   public Vector2D32 createEmptyTuple()
   {
      return new Vector2D32();
   }

   @Override
   public Vector2D32 createTuple(double x, double y)
   {
      return new Vector2D32((float) x, (float) y);
   }

   @Override
   public Vector2D32 createRandomTuple(Random random)
   {
      return EuclidCoreRandomTools.generateRandomVector2D32(random);
   }
}
