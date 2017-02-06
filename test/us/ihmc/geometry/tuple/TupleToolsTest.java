package us.ihmc.geometry.tuple;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.geometry.tuple2D.Point2D;
import us.ihmc.geometry.tuple2D.Tuple2D;
import us.ihmc.geometry.tuple3D.Point3D;
import us.ihmc.geometry.tuple3D.Tuple3D;
import us.ihmc.geometry.tuple3D.Tuple3DTools;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DBasics;

public class TupleToolsTest
{
   private static final int NUMBER_OF_ITERATIONS = 100;

   @Test
   public void testContainsNaN() throws Exception
   {
      Tuple3DBasics point = new Point3D();
      assertFalse(Tuple3DTools.containsNaN(point));
      point.set(Double.NaN, 0.0, 0.0);
      assertTrue(Tuple3DTools.containsNaN(point));
      point.set(0.0, Double.NaN, 0.0);
      assertTrue(Tuple3DTools.containsNaN(point));
      point.set(0.0, 0.0, Double.NaN);
      assertTrue(Tuple3DTools.containsNaN(point));
   }

   @Test
   public void testEpsilonEqualsTuple() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D tuple1 = new Point3D();
      Tuple3D tuple2 = new Point3D();

      double epsilon = random.nextDouble();

      tuple1.setX(random.nextDouble());
      tuple1.setY(random.nextDouble());
      tuple1.setZ(random.nextDouble());

      tuple2.setX(tuple1.getX() + 0.1 * epsilon);
      tuple2.setY(tuple1.getY() + 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() + 0.1 * epsilon);
      assertTrue(Tuple3DTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - 0.1 * epsilon);
      tuple2.setY(tuple1.getY() - 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() - 0.1 * epsilon);
      assertTrue(Tuple3DTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - 1.1 * epsilon);
      tuple2.setY(tuple1.getY() - 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() - 0.1 * epsilon);
      assertFalse(Tuple3DTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - 0.1 * epsilon);
      tuple2.setY(tuple1.getY() - 1.1 * epsilon);
      tuple2.setZ(tuple1.getZ() - 0.1 * epsilon);
      assertFalse(Tuple3DTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - 0.1 * epsilon);
      tuple2.setY(tuple1.getY() - 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() - 1.1 * epsilon);
      assertFalse(Tuple3DTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() + 1.1 * epsilon);
      tuple2.setY(tuple1.getY() + 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() + 0.1 * epsilon);
      assertFalse(Tuple3DTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX()+ 0.1 * epsilon);
      tuple2.setY(tuple1.getY()+ 1.1 * epsilon);
      tuple2.setZ(tuple1.getZ()+ 0.1 * epsilon);
      assertFalse(Tuple3DTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() + 0.1 * epsilon);
      tuple2.setY(tuple1.getY() + 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() + 1.1 * epsilon);
      assertFalse(Tuple3DTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() + epsilon);
      tuple2.setY(tuple1.getY() + epsilon);
      tuple2.setZ(tuple1.getZ() + epsilon);
      assertTrue(Tuple3DTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - epsilon);
      tuple2.setY(tuple1.getY() - epsilon);
      tuple2.setZ(tuple1.getZ() - epsilon);
      assertTrue(Tuple3DTools.epsilonEquals(tuple1, tuple2, epsilon));

   }

   @Test
   public void testEpsilonEqualsTuple2D() throws Exception
   {
      Random random = new Random(621541L);
      Tuple2D tuple1 = new Point2D();
      Tuple2D tuple2 = new Point2D();

      double epsilon = random.nextDouble();

      tuple1.setX(random.nextDouble());
      tuple1.setY(random.nextDouble());

      tuple2.setX(tuple1.getX() + 0.1 * epsilon);
      tuple2.setY(tuple1.getY() + 0.1 * epsilon);
      assertTrue(Tuple3DTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - 0.1 * epsilon);
      tuple2.setY(tuple1.getY() - 0.1 * epsilon);
      assertTrue(Tuple3DTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - 1.1 * epsilon);
      tuple2.setY(tuple1.getY() - 0.1 * epsilon);
      assertFalse(Tuple3DTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - 0.1 * epsilon);
      tuple2.setY(tuple1.getY() - 1.1 * epsilon);
      assertFalse(Tuple3DTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() + 1.1 * epsilon);
      tuple2.setY(tuple1.getY() + 0.1 * epsilon);
      assertFalse(Tuple3DTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX()+ 0.1 * epsilon);
      tuple2.setY(tuple1.getY()+ 1.1 * epsilon);
      assertFalse(Tuple3DTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() + epsilon);
      tuple2.setY(tuple1.getY() + epsilon);
      assertTrue(Tuple3DTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - epsilon);
      tuple2.setY(tuple1.getY() - epsilon);
      assertTrue(Tuple3DTools.epsilonEquals(tuple1, tuple2, epsilon));
   }

   @Test
   public void testInterpolate() throws Exception
   {
      Random random = new Random(3665L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.interpolate(double a, double b, double alpha)
         double a = random.nextDouble();
         double b = random.nextDouble();
         double alpha = random.nextDouble();

         double result = Tuple3DTools.interpolate(a, b, alpha);
         double expected = a + alpha * (b - a);
         assertEquals(result, expected, 1.0e-10);

         alpha = 0.5;
         result = Tuple3DTools.interpolate(a, b, alpha);
         assertTrue(result == 0.5 * a + 0.5 * b);
         alpha = 0.0;
         result = Tuple3DTools.interpolate(a, b, alpha);
         assertTrue(result == a);
         alpha = 1.0;
         result = Tuple3DTools.interpolate(a, b, alpha);
         assertTrue(result == b);

         for (alpha = -2.0; alpha <= 2.0; alpha += 0.1)
         {
            result = Tuple3DTools.interpolate(a, b, alpha);
            assertEquals(result, a + alpha * (b - a), 1.0e-10);
         }
      }
   }
}
