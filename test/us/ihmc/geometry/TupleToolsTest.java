package us.ihmc.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.geometry.tuple2D.Point2D;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.geometry.tuple3D.Point3D;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.geometry.tuple4D.Vector4D;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DBasics;

public class TupleToolsTest
{
   private static final int NUMBER_OF_ITERATIONS = 100;

   @Test
   public void testEpsilonEqualsTuple2D() throws Exception
   {
      Random random = new Random(621541L);
      Tuple2DBasics tuple1 = new Point2D();
      Tuple2DBasics tuple2 = new Point2D();

      double epsilon = random.nextDouble();

      tuple1.setX(random.nextDouble());
      tuple1.setY(random.nextDouble());

      tuple2.setX(tuple1.getX() + 0.1 * epsilon);
      tuple2.setY(tuple1.getY() + 0.1 * epsilon);
      assertTrue(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - 0.1 * epsilon);
      tuple2.setY(tuple1.getY() - 0.1 * epsilon);
      assertTrue(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - 1.1 * epsilon);
      tuple2.setY(tuple1.getY() - 0.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - 0.1 * epsilon);
      tuple2.setY(tuple1.getY() - 1.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() + 1.1 * epsilon);
      tuple2.setY(tuple1.getY() + 0.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() + 0.1 * epsilon);
      tuple2.setY(tuple1.getY() + 1.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() + epsilon);
      tuple2.setY(tuple1.getY() + epsilon);
      assertTrue(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - epsilon);
      tuple2.setY(tuple1.getY() - epsilon);
      assertTrue(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));
   }

   @Test
   public void testEpsilonEqualsTuple3D() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3DBasics tuple1 = new Point3D();
      Tuple3DBasics tuple2 = new Point3D();

      double epsilon = random.nextDouble();

      tuple1.setX(random.nextDouble());
      tuple1.setY(random.nextDouble());
      tuple1.setZ(random.nextDouble());

      tuple2.setX(tuple1.getX() + 0.1 * epsilon);
      tuple2.setY(tuple1.getY() + 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() + 0.1 * epsilon);
      assertTrue(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - 0.1 * epsilon);
      tuple2.setY(tuple1.getY() - 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() - 0.1 * epsilon);
      assertTrue(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - 1.1 * epsilon);
      tuple2.setY(tuple1.getY() - 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() - 0.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - 0.1 * epsilon);
      tuple2.setY(tuple1.getY() - 1.1 * epsilon);
      tuple2.setZ(tuple1.getZ() - 0.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - 0.1 * epsilon);
      tuple2.setY(tuple1.getY() - 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() - 1.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() + 1.1 * epsilon);
      tuple2.setY(tuple1.getY() + 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() + 0.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() + 0.1 * epsilon);
      tuple2.setY(tuple1.getY() + 1.1 * epsilon);
      tuple2.setZ(tuple1.getZ() + 0.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() + 0.1 * epsilon);
      tuple2.setY(tuple1.getY() + 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() + 1.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() + epsilon);
      tuple2.setY(tuple1.getY() + epsilon);
      tuple2.setZ(tuple1.getZ() + epsilon);
      assertTrue(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - epsilon);
      tuple2.setY(tuple1.getY() - epsilon);
      tuple2.setZ(tuple1.getZ() - epsilon);
      assertTrue(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));
   }

   @Test
   public void testEpsilonEqualsTuple4D() throws Exception
   {
      Random random = new Random(621541L);
      Vector4DBasics tuple1 = new Vector4D();
      Vector4DBasics tuple2 = new Vector4D();

      double epsilon = random.nextDouble();

      tuple1.setX(random.nextDouble());
      tuple1.setY(random.nextDouble());
      tuple1.setZ(random.nextDouble());
      tuple1.setS(random.nextDouble());

      tuple2.setX(tuple1.getX() + 0.1 * epsilon);
      tuple2.setY(tuple1.getY() + 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() + 0.1 * epsilon);
      tuple2.setS(tuple1.getS() + 0.1 * epsilon);
      assertTrue(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - 0.1 * epsilon);
      tuple2.setY(tuple1.getY() - 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() - 0.1 * epsilon);
      tuple2.setS(tuple1.getS() - 0.1 * epsilon);
      assertTrue(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - 1.1 * epsilon);
      tuple2.setY(tuple1.getY() - 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() - 0.1 * epsilon);
      tuple2.setS(tuple1.getS() - 0.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - 0.1 * epsilon);
      tuple2.setY(tuple1.getY() - 1.1 * epsilon);
      tuple2.setZ(tuple1.getZ() - 0.1 * epsilon);
      tuple2.setS(tuple1.getS() - 0.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - 0.1 * epsilon);
      tuple2.setY(tuple1.getY() - 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() - 1.1 * epsilon);
      tuple2.setS(tuple1.getS() - 0.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - 0.1 * epsilon);
      tuple2.setY(tuple1.getY() - 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() - 0.1 * epsilon);
      tuple2.setS(tuple1.getS() - 1.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() + 1.1 * epsilon);
      tuple2.setY(tuple1.getY() + 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() + 0.1 * epsilon);
      tuple2.setS(tuple1.getS() + 0.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() + 0.1 * epsilon);
      tuple2.setY(tuple1.getY() + 1.1 * epsilon);
      tuple2.setZ(tuple1.getZ() + 0.1 * epsilon);
      tuple2.setS(tuple1.getS() + 0.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() + 0.1 * epsilon);
      tuple2.setY(tuple1.getY() + 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() + 1.1 * epsilon);
      tuple2.setS(tuple1.getS() + 0.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() + 0.1 * epsilon);
      tuple2.setY(tuple1.getY() + 0.1 * epsilon);
      tuple2.setZ(tuple1.getZ() + 0.1 * epsilon);
      tuple2.setS(tuple1.getS() + 1.1 * epsilon);
      assertFalse(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() + epsilon);
      tuple2.setY(tuple1.getY() + epsilon);
      tuple2.setZ(tuple1.getZ() + epsilon);
      tuple2.setS(tuple1.getZ() + epsilon);
      assertTrue(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));

      tuple2.setX(tuple1.getX() - epsilon);
      tuple2.setY(tuple1.getY() - epsilon);
      tuple2.setZ(tuple1.getZ() - epsilon);
      tuple2.setS(tuple1.getS() - epsilon);
      assertTrue(TupleTools.epsilonEquals(tuple1, tuple2, epsilon));
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

         double result = TupleTools.interpolate(a, b, alpha);
         double expected = a + alpha * (b - a);
         assertEquals(result, expected, 1.0e-10);

         alpha = 0.5;
         result = TupleTools.interpolate(a, b, alpha);
         assertTrue(result == 0.5 * a + 0.5 * b);
         alpha = 0.0;
         result = TupleTools.interpolate(a, b, alpha);
         assertTrue(result == a);
         alpha = 1.0;
         result = TupleTools.interpolate(a, b, alpha);
         assertTrue(result == b);

         for (alpha = -2.0; alpha <= 2.0; alpha += 0.1)
         {
            result = TupleTools.interpolate(a, b, alpha);
            assertEquals(result, a + alpha * (b - a), 1.0e-10);
         }
      }
   }
}
