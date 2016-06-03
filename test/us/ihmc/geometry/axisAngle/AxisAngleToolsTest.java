package us.ihmc.geometry.axisAngle;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;

public class AxisAngleToolsTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testContainsNaN() throws Exception
   {
      AxisAngle axisAngle = new AxisAngle();
      axisAngle.set(0.0, 0.0, 0.0, 0.0);
      assertFalse(AxisAngleTools.containsNaN(axisAngle));
      axisAngle.set(Double.NaN, 0.0, 0.0, 0.0);
      assertTrue(AxisAngleTools.containsNaN(axisAngle));
      axisAngle.set(0.0, Double.NaN, 0.0, 0.0);
      assertTrue(AxisAngleTools.containsNaN(axisAngle));
      axisAngle.set(0.0, 0.0, Double.NaN, 0.0);
      assertTrue(AxisAngleTools.containsNaN(axisAngle));
      axisAngle.set(0.0, 0.0, 0.0, Double.NaN);
      assertTrue(AxisAngleTools.containsNaN(axisAngle));
   }

   @Test
   public void testAxisNormSquared() throws Exception
   {
      Random random = new Random(51651L);

      assertTrue(AxisAngleTools.axisNormSquared(1.0, 0.0, 0.0, 0.0) == 1.0);
      assertTrue(AxisAngleTools.axisNormSquared(0.0, 1.0, 0.0, 0.0) == 1.0);
      assertTrue(AxisAngleTools.axisNormSquared(0.0, 0.0, 1.0, 0.0) == 1.0);
      assertTrue(AxisAngleTools.axisNormSquared(0.0, 0.0, 0.0, 1.0) == 0.0);

      assertTrue(AxisAngleTools.axisNormSquared(-1.0, 0.0, 0.0, 0.0) == 1.0);
      assertTrue(AxisAngleTools.axisNormSquared(0.0, -1.0, 0.0, 0.0) == 1.0);
      assertTrue(AxisAngleTools.axisNormSquared(0.0, 0.0, -1.0, 0.0) == 1.0);

      assertTrue(AxisAngleTools.axisNormSquared(2.0, 0.0, 0.0, 0.0) == 4.0);
      assertTrue(AxisAngleTools.axisNormSquared(0.0, 2.0, 0.0, 0.0) == 4.0);
      assertTrue(AxisAngleTools.axisNormSquared(0.0, 0.0, 2.0, 0.0) == 4.0);

      assertTrue(AxisAngleTools.axisNormSquared(-2.0, 0.0, 0.0, 0.0) == 4.0);
      assertTrue(AxisAngleTools.axisNormSquared(0.0, -2.0, 0.0, 0.0) == 4.0);
      assertTrue(AxisAngleTools.axisNormSquared(0.0, 0.0, -2.0, 0.0) == 4.0);

      for (int i = 0; i < 20; i++)
      {
         double ux = GeometryBasicsRandomTools.generateRandomDouble(random, 10.0);
         double uy = GeometryBasicsRandomTools.generateRandomDouble(random, 10.0);
         double uz = GeometryBasicsRandomTools.generateRandomDouble(random, 10.0);
         double angle = GeometryBasicsRandomTools.generateRandomDouble(random, 10.0);
         double actualNormSquared = AxisAngleTools.axisNormSquared(ux, uy, uz, angle);
         double expectedNormSquared = ux * ux + uy * uy + uz * uz;
         assertTrue(actualNormSquared == expectedNormSquared);

         AxisAngle axisAngle = new AxisAngle(ux, uy, uz, angle);
         actualNormSquared = AxisAngleTools.axisNormSquared(axisAngle);
         assertTrue(actualNormSquared == expectedNormSquared);
         assertTrue(axisAngle.getX() == ux);
         assertTrue(axisAngle.getY() == uy);
         assertTrue(axisAngle.getZ() == uz);
         assertTrue(axisAngle.getAngle() == angle);
      }
   }

   @Test
   public void testAxisNorm() throws Exception
   {
      Random random = new Random(51651L);

      assertTrue(AxisAngleTools.axisNorm(1.0, 0.0, 0.0, 0.0) == 1.0);
      assertTrue(AxisAngleTools.axisNorm(0.0, 1.0, 0.0, 0.0) == 1.0);
      assertTrue(AxisAngleTools.axisNorm(0.0, 0.0, 1.0, 0.0) == 1.0);
      assertTrue(AxisAngleTools.axisNorm(0.0, 0.0, 0.0, 1.0) == 0.0);

      assertTrue(AxisAngleTools.axisNorm(-1.0, 0.0, 0.0, 0.0) == 1.0);
      assertTrue(AxisAngleTools.axisNorm(0.0, -1.0, 0.0, 0.0) == 1.0);
      assertTrue(AxisAngleTools.axisNorm(0.0, 0.0, -1.0, 0.0) == 1.0);

      assertTrue(AxisAngleTools.axisNorm(2.0, 0.0, 0.0, 0.0) == 2.0);
      assertTrue(AxisAngleTools.axisNorm(0.0, 2.0, 0.0, 0.0) == 2.0);
      assertTrue(AxisAngleTools.axisNorm(0.0, 0.0, 2.0, 0.0) == 2.0);

      assertTrue(AxisAngleTools.axisNorm(-2.0, 0.0, 0.0, 0.0) == 2.0);
      assertTrue(AxisAngleTools.axisNorm(0.0, -2.0, 0.0, 0.0) == 2.0);
      assertTrue(AxisAngleTools.axisNorm(0.0, 0.0, -2.0, 0.0) == 2.0);

      for (int i = 0; i < 20; i++)
      {
         double ux = GeometryBasicsRandomTools.generateRandomDouble(random, 10.0);
         double uy = GeometryBasicsRandomTools.generateRandomDouble(random, 10.0);
         double uz = GeometryBasicsRandomTools.generateRandomDouble(random, 10.0);
         double angle = GeometryBasicsRandomTools.generateRandomDouble(random, 10.0);
         double actualNormSquared = AxisAngleTools.axisNorm(ux, uy, uz, angle);
         double expectedNormSquared = Math.sqrt(ux * ux + uy * uy + uz * uz);
         assertTrue(actualNormSquared == expectedNormSquared);

         AxisAngle axisAngle = new AxisAngle(ux, uy, uz, angle);
         actualNormSquared = AxisAngleTools.axisNorm(axisAngle);
         assertTrue(actualNormSquared == expectedNormSquared);
         assertTrue(axisAngle.getX() == ux);
         assertTrue(axisAngle.getY() == uy);
         assertTrue(axisAngle.getZ() == uz);
         assertTrue(axisAngle.getAngle() == angle);
      }
   }

   @Test
   public void testIsAxisUnitary() throws Exception
   {
      Random random = new Random(51651L);

      assertTrue(AxisAngleTools.isAxisUnitary(1.0, 0.0, 0.0, 0.0, EPSILON));
      assertTrue(AxisAngleTools.isAxisUnitary(0.0, 1.0, 0.0, 0.0, EPSILON));
      assertTrue(AxisAngleTools.isAxisUnitary(0.0, 0.0, 1.0, 0.0, EPSILON));
      assertFalse(AxisAngleTools.isAxisUnitary(0.0, 0.0, 0.0, 1.0, EPSILON));

      assertTrue(AxisAngleTools.isAxisUnitary(-1.0, 0.0, 0.0, 0.0, EPSILON));
      assertTrue(AxisAngleTools.isAxisUnitary(0.0, -1.0, 0.0, 0.0, EPSILON));
      assertTrue(AxisAngleTools.isAxisUnitary(0.0, 0.0, -1.0, 0.0, EPSILON));

      assertFalse(AxisAngleTools.isAxisUnitary(2.0, 0.0, 0.0, 0.0, EPSILON));
      assertFalse(AxisAngleTools.isAxisUnitary(0.0, 2.0, 0.0, 0.0, EPSILON));
      assertFalse(AxisAngleTools.isAxisUnitary(0.0, 0.0, 2.0, 0.0, EPSILON));

      assertFalse(AxisAngleTools.isAxisUnitary(-2.0, 0.0, 0.0, 0.0, EPSILON));
      assertFalse(AxisAngleTools.isAxisUnitary(0.0, -2.0, 0.0, 0.0, EPSILON));
      assertFalse(AxisAngleTools.isAxisUnitary(0.0, 0.0, -2.0, 0.0, EPSILON));

      for (int i = 0; i < 20; i++)
      {
         double smallScale = GeometryBasicsRandomTools.generateRandomDouble(random, 0.90, 0.95);
         double bigScale = GeometryBasicsRandomTools.generateRandomDouble(random, 1.05, 1.10);

         double ux = GeometryBasicsRandomTools.generateRandomDouble(random, 1.0);
         double uy = GeometryBasicsRandomTools.generateRandomDouble(random, 1.0);
         double uz = GeometryBasicsRandomTools.generateRandomDouble(random, 1.0);
         double angle = GeometryBasicsRandomTools.generateRandomDouble(random, 10.0);
         double norm = AxisAngleTools.axisNorm(ux, uy, uz, angle);

         ux /= norm;
         uy /= norm;
         uz /= norm;

         assertTrue(AxisAngleTools.isAxisUnitary(ux, uy, uz, angle, EPSILON));

         assertFalse(AxisAngleTools.isAxisUnitary(ux * smallScale, uy, uz, angle, EPSILON));
         assertFalse(AxisAngleTools.isAxisUnitary(ux, uy * smallScale, uz, angle, EPSILON));
         assertFalse(AxisAngleTools.isAxisUnitary(ux, uy, uz * smallScale, angle, EPSILON));
         assertTrue(AxisAngleTools.isAxisUnitary(ux, uy, uz, angle * smallScale, EPSILON));

         assertFalse(AxisAngleTools.isAxisUnitary(ux * bigScale, uy, uz, angle, EPSILON));
         assertFalse(AxisAngleTools.isAxisUnitary(ux, uy * bigScale, uz, angle, EPSILON));
         assertFalse(AxisAngleTools.isAxisUnitary(ux, uy, uz * bigScale, angle, EPSILON));
         assertTrue(AxisAngleTools.isAxisUnitary(ux, uy, uz, angle * bigScale, EPSILON));

         AxisAngle axisAngle = new AxisAngle(ux, uy, uz, angle);
         assertTrue(AxisAngleTools.isAxisUnitary(axisAngle, EPSILON));
         assertTrue(axisAngle.getX() == ux);
         assertTrue(axisAngle.getY() == uy);
         assertTrue(axisAngle.getZ() == uz);
         assertTrue(axisAngle.getAngle() == angle);
      }
   }
}
