package us.ihmc.geometry.axisAngle;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.geometry.axisAngle.interfaces.AxisAngleReadOnly;

public abstract class AxisAngleReadOnlyTest<T extends AxisAngleReadOnly<?>>
{
   public static final int NUMBER_OF_ITERATIONS = 100;

   public abstract T createEmptyAxisAngle();

   public abstract T createAxisAngle(double ux, double uy, double uz, double angle);


   @Test
   public void testContainsNaN()
   {
      T axisAngle;

      axisAngle = createAxisAngle(0.0, 0.0, 0.0, 0.0);
      assertFalse(axisAngle.containsNaN());
      axisAngle = createAxisAngle(Double.NaN, 0.0, 0.0, 0.0);
      assertTrue(axisAngle.containsNaN());
      axisAngle = createAxisAngle(0.0, Double.NaN, 0.0, 0.0);
      assertTrue(axisAngle.containsNaN());
      axisAngle = createAxisAngle(0.0, 0.0, Double.NaN, 0.0);
      assertTrue(axisAngle.containsNaN());
      axisAngle = createAxisAngle(0.0, 0.0, 0.0, Double.NaN);
      assertTrue(axisAngle.containsNaN());
   }

   @Test
   public void testGetAngle()
   {
      Random random = new Random(564648L);
      T axisAngle;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double expectedAngle = random.nextInt(100);
         axisAngle = createAxisAngle(0.0, 0.0, 0.0, expectedAngle);
         double actualAngle = axisAngle.getAngle();

         assertTrue(expectedAngle == actualAngle);
      }
   }

   @Test
   public void testGetX()
   {
      Random random = new Random(564648L);
      T axisAngle;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double expectedX = random.nextInt(100);
         axisAngle = createAxisAngle(expectedX, 0.0, 0.0, 0.0);
         double actualX = axisAngle.getX();

         assertTrue(expectedX == actualX);
      }
   }

   @Test
   public void testGetY()
   {
      Random random = new Random(564648L);
      T axisAngle;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double expectedY = random.nextInt(100);
         axisAngle = createAxisAngle(0.0, expectedY, 0.0, 0.0);
         double actualY = axisAngle.getY();

         assertTrue(expectedY == actualY);
      }
   }

   @Test
   public void testGetZ()
   {
      Random random = new Random(564648L);
      T axisAngle;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double expectedZ = random.nextInt(100);
         axisAngle = createAxisAngle(0.0, 0.0, expectedZ, 0.0);
         double actualZ = axisAngle.getZ();

         assertTrue(expectedZ == actualZ);
      }
   }

   @Test
   public void testGetAngle32()
   {
      Random random = new Random(564648L);
      T axisAngle;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         float expectedAngle = random.nextInt(100);
         axisAngle = createAxisAngle(0.0, 0.0, 0.0, expectedAngle);
         float actualAngle = axisAngle.getAngle32();

         assertTrue(expectedAngle == actualAngle);
      }
   }

   @Test
   public void testGetX32()
   {
      Random random = new Random(564648L);
      T axisAngle;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         float expectedX = random.nextInt(100);
         axisAngle = createAxisAngle(expectedX, 0.0, 0.0, 0.0);
         float actualX = axisAngle.getX32();

         assertTrue(expectedX == actualX);
      }
   }

   @Test
   public void testGetY32()
   {
      Random random = new Random(564648L);
      T axisAngle;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         float expectedY = random.nextInt(100);
         axisAngle = createAxisAngle(0.0, expectedY, 0.0, 0.0);
         float actualY = axisAngle.getY32();

         assertTrue(expectedY == actualY);
      }
   }

   @Test
   public void testGetZ32()
   {
      Random random = new Random(564648L);
      T axisAngle;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         float expectedZ = random.nextInt(100);
         axisAngle = createAxisAngle(0.0, 0.0, expectedZ, 0.0);
         float actualZ = axisAngle.getZ32();

         assertTrue(expectedZ == actualZ);
      }
   }
}