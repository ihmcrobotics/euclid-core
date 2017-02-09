package us.ihmc.geometry.tuple4D;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.testingTools.GeometryBasicsTestTools;
import us.ihmc.geometry.tuple4D.interfaces.Tuple4DReadOnly;

public class Vector4D32Test extends Vector4DBasicsTest<Vector4D32>
{
   private static final double EPS = 1.0e-10;

   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(3453L);

      { // Test empty constructor
         Vector4D32 vector = new Vector4D32();
         assertTrue(vector.getX32() == 0.0f);
         assertTrue(vector.getY32() == 0.0f);
         assertTrue(vector.getZ32() == 0.0f);
         assertTrue(vector.getS32() == 0.0f);
      }

      { // Test Vector4D32(float x, float y, float z, float s)
         float x = random.nextFloat();
         float y = random.nextFloat();
         float z = random.nextFloat();
         float s = random.nextFloat();
         Vector4D32 vector = new Vector4D32(x, y, z, s);
         assertTrue(vector.getX32() == x);
         assertTrue(vector.getY32() == y);
         assertTrue(vector.getZ32() == z);
         assertTrue(vector.getS32() == s);
      }

      { // Test Vector4D32(float[] vectorArray)
         float x = random.nextFloat();
         float y = random.nextFloat();
         float z = random.nextFloat();
         float s = random.nextFloat();
         float[] vectorArray = {x, y, z, s};
         Vector4D32 vector = new Vector4D32(vectorArray);
         assertTrue(vector.getX32() == x);
         assertTrue(vector.getY32() == y);
         assertTrue(vector.getZ32() == z);
         assertTrue(vector.getS32() == s);
      }

      { // Test Vector4D32(QuaternionReadOnly quaternion)
         Quaternion32 quaternion = GeometryBasicsRandomTools.generateRandomQuaternion32(random);
         Vector4D32 vector = new Vector4D32(quaternion);
         GeometryBasicsTestTools.assertTuple4DEquals(quaternion, vector, EPS);
      }

      { // Test Vector4D32(Tuple4DReadOnly other)
         Tuple4DReadOnly<?> quaternion = GeometryBasicsRandomTools.generateRandomQuaternion32(random);
         Vector4D32 vector = new Vector4D32(quaternion);
         GeometryBasicsTestTools.assertTuple4DEquals(quaternion, vector, EPS);
      }
   }

   @Override
   public Vector4D32 createEmptyTuple()
   {
      return new Vector4D32();
   }

   @Override
   public Vector4D32 createRandomTuple(Random random)
   {
      return GeometryBasicsRandomTools.generateRandomVector4D32(random);
   }

   @Override
   public Vector4D32 createTuple(double x, double y, double z, double s)
   {
      return new Vector4D32((float) x, (float) y, (float) z, (float) s);
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-6;
   }
}
