package us.ihmc.geometry.tuple4D;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.testingTools.GeometryBasicsTestTools;
import us.ihmc.geometry.tuple4D.interfaces.Tuple4DReadOnly;

public class Vector4DTest extends Vector4DBasicsTest<Vector4D>
{
   private static final double EPS = 1.0e-10;

   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(3453L);

      { // Test empty constructor
         Vector4D vector = new Vector4D();
         assertTrue(vector.getX() == 0.0);
         assertTrue(vector.getY() == 0.0);
         assertTrue(vector.getZ() == 0.0);
         assertTrue(vector.getS() == 0.0);
      }

      { // Test Vector4D(double x, double y, double z, double s)
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         double s = random.nextDouble();
         Vector4D vector = new Vector4D(x, y, z, s);
         assertTrue(vector.getX() == x);
         assertTrue(vector.getY() == y);
         assertTrue(vector.getZ() == z);
         assertTrue(vector.getS() == s);
      }

      { // Test Vector4D(double[] vectorArray)
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         double s = random.nextDouble();
         double[] vectorArray = {x, y, z, s};
         Vector4D vector = new Vector4D(vectorArray);
         assertTrue(vector.getX() == x);
         assertTrue(vector.getY() == y);
         assertTrue(vector.getZ() == z);
         assertTrue(vector.getS() == s);
      }

      { // Test Vector4D(QuaternionReadOnly quaternion)
         Quaternion quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         Vector4D vector = new Vector4D(quaternion);
         GeometryBasicsTestTools.assertTuple4DEquals(quaternion, vector, EPS);
      }

      { // Test Vector4D(Tuple4DReadOnly other)
         Tuple4DReadOnly<?> quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         Vector4D vector = new Vector4D(quaternion);
         GeometryBasicsTestTools.assertTuple4DEquals(quaternion, vector, EPS);
      }
   }

   @Override
   public Vector4D createEmptyTuple()
   {
      return new Vector4D();
   }

   @Override
   public Vector4D createRandomTuple(Random random)
   {
      return GeometryBasicsRandomTools.generateRandomVector4D(random);
   }

   @Override
   public Vector4D createTuple(double x, double y, double z, double s)
   {
      return new Vector4D(x, y, z, s);
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-15;
   }
}
