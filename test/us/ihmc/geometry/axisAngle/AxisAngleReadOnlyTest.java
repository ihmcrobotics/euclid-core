package us.ihmc.geometry.axisAngle;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Test;

import us.ihmc.geometry.EuclidCoreTools;
import us.ihmc.geometry.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.testingTools.GeometryBasicsTestTools;
import us.ihmc.geometry.tuple3D.RotationVectorConversion;
import us.ihmc.geometry.tuple3D.Vector3D;

public abstract class AxisAngleReadOnlyTest<T extends AxisAngleReadOnly>
{
   public static final int NUMBER_OF_ITERATIONS = 100;

   public abstract T createEmptyAxisAngle();

   public abstract T createAxisAngle(double ux, double uy, double uz, double angle);

   public abstract T createRandomAxisAngle(Random random);

   public abstract double getEpsilon();

   public abstract double getSmallestEpsilon();

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
   public void testIsAxisUnitary() throws Exception
   {
      Random random = new Random(51651L);
      AxisAngle axisAngle = new AxisAngle();

      for (int i = 0; i < 20; i++)
      {
         double smallScale = GeometryBasicsRandomTools.generateRandomDouble(random, 0.90, 0.95);
         double bigScale = GeometryBasicsRandomTools.generateRandomDouble(random, 1.05, 1.10);

         double ux = GeometryBasicsRandomTools.generateRandomDouble(random, 1.0);
         double uy = GeometryBasicsRandomTools.generateRandomDouble(random, 1.0);
         double uz = GeometryBasicsRandomTools.generateRandomDouble(random, 1.0);
         double angle = GeometryBasicsRandomTools.generateRandomDouble(random, 10.0);
         double norm = EuclidCoreTools.norm(ux, uy, uz);

         ux /= norm;
         uy /= norm;
         uz /= norm;

         axisAngle.set(ux, uy, uz, angle);
         assertTrue(axisAngle.isAxisUnitary(getEpsilon()));

         axisAngle.set(ux * smallScale, uy, uz, angle);
         assertFalse(axisAngle.isAxisUnitary(getEpsilon()));
         axisAngle.set(ux, uy * smallScale, uz, angle);
         assertFalse(axisAngle.isAxisUnitary(getEpsilon()));
         axisAngle.set(ux, uy, uz * smallScale, angle);
         assertFalse(axisAngle.isAxisUnitary(getEpsilon()));
         axisAngle.set(ux, uy, uz, angle * smallScale);
         assertTrue(axisAngle.isAxisUnitary(getEpsilon()));

         axisAngle.set(ux * bigScale, uy, uz, angle);
         assertFalse(axisAngle.isAxisUnitary(getEpsilon()));
         axisAngle.set(ux, uy * bigScale, uz, angle);
         assertFalse(axisAngle.isAxisUnitary(getEpsilon()));
         axisAngle.set(ux, uy, uz * bigScale, angle);
         assertFalse(axisAngle.isAxisUnitary(getEpsilon()));
         axisAngle.set(ux, uy, uz, angle * bigScale);
         assertTrue(axisAngle.isAxisUnitary(getEpsilon()));
      }
   }

   @Test
   public void testIsZOnly() throws Exception
   {
      Random random = new Random(23905872L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double ux = random.nextDouble();
         double uy = random.nextDouble();
         double uz = random.nextDouble();
         double angle = random.nextDouble();
         T axisAngle = createEmptyAxisAngle();

         axisAngle = createAxisAngle(ux, uy, uz, angle);
         assertFalse(axisAngle.isZOnly(getEpsilon()));

         axisAngle = createAxisAngle(0.0, uy, uz, angle);
         assertFalse(axisAngle.isZOnly(getEpsilon()));

         axisAngle = createAxisAngle(ux, 0.0, uz, angle);
         assertFalse(axisAngle.isZOnly(getEpsilon()));

         axisAngle = createAxisAngle(0.0, 0.0, uz, angle);
         assertTrue(axisAngle.isZOnly(getEpsilon()));

         axisAngle = createAxisAngle(2.0 * getEpsilon(), 0.0, uz, angle);
         assertFalse(axisAngle.isZOnly(getEpsilon()));
         axisAngle = createAxisAngle(0.0, 2.0 * getEpsilon(), uz, angle);
         assertFalse(axisAngle.isZOnly(getEpsilon()));
      }
   }

   @Test
   public void testCheckIfIsZOnly() throws Exception
   {
      Random random = new Random(23905872L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double qx = random.nextDouble();
         double qy = random.nextDouble();
         double qz = random.nextDouble();
         double qs = random.nextDouble();
         T axisAngle = createEmptyAxisAngle();

         axisAngle = createAxisAngle(qx, qy, qz, qs);

         try
         {
            axisAngle.checkIfIsZOnly(getEpsilon());
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a RuntimeException");
         }

         axisAngle = createAxisAngle(0.0, qy, qz, qs);
         try
         {
            axisAngle.checkIfIsZOnly(getEpsilon());
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a RuntimeException");
         }

         axisAngle = createAxisAngle(qx, 0.0, qz, qs);
         try
         {
            axisAngle.checkIfIsZOnly(getEpsilon());
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a RuntimeException");
         }

         axisAngle = createAxisAngle(0.0, 0.0, qz, qs);
         axisAngle.checkIfIsZOnly(getEpsilon());
      }
   }

   @Test
   public void testGetRotationVector()
   {
      Random random = new Random(2343456L);
      T axisAngle;
      Vector3D actualVector = new Vector3D();
      Vector3D expectedVector = new Vector3D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         axisAngle = createRandomAxisAngle(random);

         actualVector.setToNaN();
         axisAngle.getRotationVector(actualVector);
         RotationVectorConversion.convertAxisAngleToRotationVector(axisAngle, expectedVector);

         GeometryBasicsTestTools.assertRotationVectorEquals(actualVector, expectedVector, getEpsilon());
      }
   }

   @Test
   public void testGetDoubleArray()
   {
      Random random = new Random(3513515L);
      T axisAngle;

      { // Test get(double[] axisAngleArray)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            double[] axisAngleArray = new double[4];
            axisAngle = createRandomAxisAngle(random);
            axisAngle.get(axisAngleArray);

            assertTrue(axisAngle.getX() == axisAngleArray[0]);
            assertTrue(axisAngle.getY() == axisAngleArray[1]);
            assertTrue(axisAngle.getZ() == axisAngleArray[2]);
            assertTrue(axisAngle.getAngle() == axisAngleArray[3]);
         }
      }

      { // Test get(double[] axisAngleArray, int startIndex)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            int startIndex = random.nextInt(20);
            double[] axisAngleArray = new double[startIndex + 4 + random.nextInt(10)];
            axisAngle = createRandomAxisAngle(random);

            axisAngle.get(startIndex, axisAngleArray);

            assertTrue(axisAngle.getX() == axisAngleArray[startIndex + 0]);
            assertTrue(axisAngle.getY() == axisAngleArray[startIndex + 1]);
            assertTrue(axisAngle.getZ() == axisAngleArray[startIndex + 2]);
            assertTrue(axisAngle.getAngle() == axisAngleArray[startIndex + 3]);
         }
      }
   }

   @Test
   public void testGetFloatArray()
   {
      Random random = new Random(3513515L);
      T axisAngle;

      { // Test get(float[] axisAngleArray)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            float[] axisAngleArray = new float[4];
            axisAngle = createRandomAxisAngle(random);
            axisAngle.get(axisAngleArray);

            assertTrue(axisAngle.getX32() == axisAngleArray[0]);
            assertTrue(axisAngle.getY32() == axisAngleArray[1]);
            assertTrue(axisAngle.getZ32() == axisAngleArray[2]);
            assertTrue(axisAngle.getAngle32() == axisAngleArray[3]);
         }
      }

      { // Test get(float[] axisAngleArray, int startIndex)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            int startIndex = random.nextInt(20);
            float[] axisAngleArray = new float[startIndex + 4 + random.nextInt(10)];
            axisAngle = createRandomAxisAngle(random);

            axisAngle.get(startIndex, axisAngleArray);

            assertTrue(axisAngle.getX32() == axisAngleArray[startIndex + 0]);
            assertTrue(axisAngle.getY32() == axisAngleArray[startIndex + 1]);
            assertTrue(axisAngle.getZ32() == axisAngleArray[startIndex + 2]);
            assertTrue(axisAngle.getAngle32() == axisAngleArray[startIndex + 3]);
         }
      }
   }

   @Test
   public void testGetWithIndex() throws Exception
   {
      Random random = new Random(324234L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         T axisAngle = createRandomAxisAngle(random);
         assertTrue(axisAngle.getX() == axisAngle.get(0));
         assertTrue(axisAngle.getY() == axisAngle.get(1));
         assertTrue(axisAngle.getZ() == axisAngle.get(2));
         assertTrue(axisAngle.getAngle() == axisAngle.get(3));

         assertTrue(axisAngle.getX32() == axisAngle.get32(0));
         assertTrue(axisAngle.getY32() == axisAngle.get32(1));
         assertTrue(axisAngle.getZ32() == axisAngle.get32(2));
         assertTrue(axisAngle.getAngle32() == axisAngle.get32(3));
      }
   }

   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(621541L);

      T axisAngle = createRandomAxisAngle(random);

      assertFalse(axisAngle.equals(createEmptyAxisAngle()));
      assertFalse(axisAngle.equals((Object) createEmptyAxisAngle()));
      assertTrue(axisAngle.equals((Object) axisAngle));
      assertFalse(axisAngle.equals(null));
      assertFalse(axisAngle.equals(new double[5]));

      double x = axisAngle.getX();
      double y = axisAngle.getY();
      double z = axisAngle.getZ();
      double angle = axisAngle.getAngle();

      assertTrue(axisAngle.equals(createAxisAngle(x, y, z, angle)));

      assertFalse(axisAngle.equals(createAxisAngle(x + getSmallestEpsilon(), y, z, angle)));
      assertFalse(axisAngle.equals(createAxisAngle(x - getSmallestEpsilon(), y, z, angle)));

      assertFalse(axisAngle.equals(createAxisAngle(x, y + getSmallestEpsilon(), z, angle)));
      assertFalse(axisAngle.equals(createAxisAngle(x, y - getSmallestEpsilon(), z, angle)));

      assertFalse(axisAngle.equals(createAxisAngle(x, y, z + getSmallestEpsilon(), angle)));
      assertFalse(axisAngle.equals(createAxisAngle(x, y, z - getSmallestEpsilon(), angle)));

      assertFalse(axisAngle.equals(createAxisAngle(x, y, z, angle + getSmallestEpsilon())));
      assertFalse(axisAngle.equals(createAxisAngle(x, y, z, angle - getSmallestEpsilon())));
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(621541L);
      double epsilon = random.nextDouble();

      T axisAngle = createRandomAxisAngle(random);
      double x = axisAngle.getX();
      double y = axisAngle.getY();
      double z = axisAngle.getZ();
      double angle = axisAngle.getAngle();

      assertTrue(axisAngle.epsilonEquals(createAxisAngle(x + 0.999 * epsilon, y, z, angle), epsilon));
      assertTrue(axisAngle.epsilonEquals(createAxisAngle(x - 0.999 * epsilon, y, z, angle), epsilon));

      assertTrue(axisAngle.epsilonEquals(createAxisAngle(x, y + 0.999 * epsilon, z, angle), epsilon));
      assertTrue(axisAngle.epsilonEquals(createAxisAngle(x, y - 0.999 * epsilon, z, angle), epsilon));

      assertTrue(axisAngle.epsilonEquals(createAxisAngle(x, y, z + 0.999 * epsilon, angle), epsilon));
      assertTrue(axisAngle.epsilonEquals(createAxisAngle(x, y, z - 0.999 * epsilon, angle), epsilon));

      assertTrue(axisAngle.epsilonEquals(createAxisAngle(x, y, z, angle + 0.999 * epsilon), epsilon));
      assertTrue(axisAngle.epsilonEquals(createAxisAngle(x, y, z, angle - 0.999 * epsilon), epsilon));

      assertFalse(axisAngle.epsilonEquals(createAxisAngle(x + 1.001 * epsilon, y, z, angle), epsilon));
      assertFalse(axisAngle.epsilonEquals(createAxisAngle(x - 1.001 * epsilon, y, z, angle), epsilon));

      assertFalse(axisAngle.epsilonEquals(createAxisAngle(x, y + 1.001 * epsilon, z, angle), epsilon));
      assertFalse(axisAngle.epsilonEquals(createAxisAngle(x, y - 1.001 * epsilon, z, angle), epsilon));

      assertFalse(axisAngle.epsilonEquals(createAxisAngle(x, y, z + 1.001 * epsilon, angle), epsilon));
      assertFalse(axisAngle.epsilonEquals(createAxisAngle(x, y, z - 1.001 * epsilon, angle), epsilon));

      assertFalse(axisAngle.epsilonEquals(createAxisAngle(x, y, z, angle + 1.001 * epsilon), epsilon));
      assertFalse(axisAngle.epsilonEquals(createAxisAngle(x, y, z, angle - 1.001 * epsilon), epsilon));
   }
}