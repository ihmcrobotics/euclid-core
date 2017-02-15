package us.ihmc.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static us.ihmc.geometry.EuclidCoreTools.EPS_NORM_FAST_SQRT;

import java.util.Random;

import org.junit.Test;

import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;

public class GeometryBasicsToolsTest
{
   @Test
   public void testFastSquareRoot() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < 1000; i++)
      {
         double squaredValue = 1.5 * random.nextDouble();
         squaredValue = Math.min(squaredValue, 1.0 - EPS_NORM_FAST_SQRT);

         double actualValue = EuclidCoreTools.fastSquareRoot(squaredValue);
         double expectedValue = Math.sqrt(squaredValue);
         assertEquals(expectedValue, actualValue, Double.MIN_VALUE);
      }

      for (int i = 0; i < 1000; i++)
      {
         double squaredValue = 0.8 + 1.5 * random.nextDouble();
         squaredValue = Math.max(squaredValue, 1.0 + EPS_NORM_FAST_SQRT);

         double actualValue = EuclidCoreTools.fastSquareRoot(squaredValue);
         double expectedValue = Math.sqrt(squaredValue);
         assertEquals(expectedValue, actualValue, Double.MIN_VALUE);
      }

      for (int i = 0; i < 1000; i++)
      {
         double squaredValue = GeometryBasicsRandomTools.generateRandomDouble(random, 1.0 - EPS_NORM_FAST_SQRT, 1.0 + EPS_NORM_FAST_SQRT);
         squaredValue = Math.max(squaredValue, 1.0 + EPS_NORM_FAST_SQRT);

         double actualValue = EuclidCoreTools.fastSquareRoot(squaredValue);
         double expectedValue = Math.sqrt(squaredValue);
         assertEquals(expectedValue, actualValue, Double.MIN_VALUE);
      }
   }

   @Test
   public void testContainsNaNWith2Elements() throws Exception
   {
      assertFalse(EuclidCoreTools.containsNaN(0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(Double.NaN, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, Double.NaN));
   }

   @Test
   public void testContainsNaNWith3Elements() throws Exception
   {
      assertFalse(EuclidCoreTools.containsNaN(0.0, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(Double.NaN, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, Double.NaN, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, 0.0, Double.NaN));
   }

   @Test
   public void testContainsNaNWith4Elements() throws Exception
   {
      assertFalse(EuclidCoreTools.containsNaN(0.0, 0.0, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(Double.NaN, 0.0, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, Double.NaN, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, 0.0, Double.NaN, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, 0.0, 0.0, Double.NaN));
   }

   @Test
   public void testContainsNaNWith9Elements() throws Exception
   {
      assertFalse(EuclidCoreTools.containsNaN(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0));
      assertTrue(EuclidCoreTools.containsNaN(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN));
   }

   @Test
   public void testContainsNaNWithArray() throws Exception
   {
      assertFalse(EuclidCoreTools.containsNaN(new double[0]));
      assertFalse(EuclidCoreTools.containsNaN(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
      assertTrue(EuclidCoreTools.containsNaN(new double[] {Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
      assertTrue(EuclidCoreTools.containsNaN(new double[] {0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
      assertTrue(EuclidCoreTools.containsNaN(new double[] {0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
      assertTrue(EuclidCoreTools.containsNaN(new double[] {0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0}));
      assertTrue(EuclidCoreTools.containsNaN(new double[] {0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0}));
      assertTrue(EuclidCoreTools.containsNaN(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0}));
      assertTrue(EuclidCoreTools.containsNaN(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0}));
      assertTrue(EuclidCoreTools.containsNaN(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0}));
      assertTrue(EuclidCoreTools.containsNaN(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN}));
   }

   @Test
   public void testNormSquaredWith2Elements() throws Exception
   {
      assertTrue(EuclidCoreTools.normSquared(1.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 1.0) == 1.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 0.0) == 0.0);

      assertTrue(EuclidCoreTools.normSquared(-1.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, -1.0) == 1.0);

      assertTrue(EuclidCoreTools.normSquared(2.0, 0.0) == 4.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 2.0) == 4.0);

      assertTrue(EuclidCoreTools.normSquared(-2.0, 0.0) == 4.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, -2.0) == 4.0);
   }

   @Test
   public void testNormSquaredWith3Elements() throws Exception
   {
      assertTrue(EuclidCoreTools.normSquared(1.0, 0.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 1.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 0.0, 1.0) == 1.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 0.0, 0.0) == 0.0);

      assertTrue(EuclidCoreTools.normSquared(-1.0, 0.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, -1.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 0.0, -1.0) == 1.0);

      assertTrue(EuclidCoreTools.normSquared(2.0, 0.0, 0.0) == 4.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 2.0, 0.0) == 4.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 0.0, 2.0) == 4.0);

      assertTrue(EuclidCoreTools.normSquared(-2.0, 0.0, 0.0) == 4.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, -2.0, 0.0) == 4.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 0.0, -2.0) == 4.0);
   }

   @Test
   public void testNormSquaredWith4Elements() throws Exception
   {
      assertTrue(EuclidCoreTools.normSquared(1.0, 0.0, 0.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 1.0, 0.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 0.0, 1.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 0.0, 0.0, 1.0) == 1.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 0.0, 0.0, 0.0) == 0.0);

      assertTrue(EuclidCoreTools.normSquared(-1.0, 0.0, 0.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, -1.0, 0.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 0.0, -1.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 0.0, 0.0, -1.0) == 1.0);

      assertTrue(EuclidCoreTools.normSquared(2.0, 0.0, 0.0, 0.0) == 4.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 2.0, 0.0, 0.0) == 4.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 0.0, 2.0, 0.0) == 4.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 0.0, 0.0, 2.0) == 4.0);

      assertTrue(EuclidCoreTools.normSquared(-2.0, 0.0, 0.0, 0.0) == 4.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, -2.0, 0.0, 0.0) == 4.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 0.0, -2.0, 0.0) == 4.0);
      assertTrue(EuclidCoreTools.normSquared(0.0, 0.0, 0.0, -2.0) == 4.0);
   }

   @Test
   public void testNormWith2Elements() throws Exception
   {
      assertTrue(EuclidCoreTools.norm(1.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.norm(0.0, 1.0) == 1.0);
      assertTrue(EuclidCoreTools.norm(0.0, 0.0) == 0.0);

      assertTrue(EuclidCoreTools.norm(-1.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.norm(0.0, -1.0) == 1.0);

      assertTrue(EuclidCoreTools.norm(2.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.norm(0.0, 2.0) == 2.0);

      assertTrue(EuclidCoreTools.norm(-2.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.norm(0.0, -2.0) == 2.0);
   }

   @Test
   public void testNormWith3Elements() throws Exception
   {
      assertTrue(EuclidCoreTools.norm(1.0, 0.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.norm(0.0, 1.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.norm(0.0, 0.0, 1.0) == 1.0);
      assertTrue(EuclidCoreTools.norm(0.0, 0.0, 0.0) == 0.0);

      assertTrue(EuclidCoreTools.norm(-1.0, 0.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.norm(0.0, -1.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.norm(0.0, 0.0, -1.0) == 1.0);

      assertTrue(EuclidCoreTools.norm(2.0, 0.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.norm(0.0, 2.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.norm(0.0, 0.0, 2.0) == 2.0);

      assertTrue(EuclidCoreTools.norm(-2.0, 0.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.norm(0.0, -2.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.norm(0.0, 0.0, -2.0) == 2.0);
   }

   @Test
   public void testNormWith4Elements() throws Exception
   {
      assertTrue(EuclidCoreTools.norm(1.0, 0.0, 0.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.norm(0.0, 1.0, 0.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.norm(0.0, 0.0, 1.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.norm(0.0, 0.0, 0.0, 1.0) == 1.0);
      assertTrue(EuclidCoreTools.norm(0.0, 0.0, 0.0, 0.0) == 0.0);

      assertTrue(EuclidCoreTools.norm(-1.0, 0.0, 0.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.norm(0.0, -1.0, 0.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.norm(0.0, 0.0, -1.0, 0.0) == 1.0);
      assertTrue(EuclidCoreTools.norm(0.0, 0.0, 0.0, -1.0) == 1.0);

      assertTrue(EuclidCoreTools.norm(2.0, 0.0, 0.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.norm(0.0, 2.0, 0.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.norm(0.0, 0.0, 2.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.norm(0.0, 0.0, 0.0, 2.0) == 2.0);

      assertTrue(EuclidCoreTools.norm(-2.0, 0.0, 0.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.norm(0.0, -2.0, 0.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.norm(0.0, 0.0, -2.0, 0.0) == 2.0);
      assertTrue(EuclidCoreTools.norm(0.0, 0.0, 0.0, -2.0) == 2.0);
   }
}
