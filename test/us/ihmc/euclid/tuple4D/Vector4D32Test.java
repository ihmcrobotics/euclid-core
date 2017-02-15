package us.ihmc.euclid.tuple4D;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.euclid.tuple4D.Vector4D32;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;

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
         Quaternion32 quaternion = EuclidCoreRandomTools.generateRandomQuaternion32(random);
         Vector4D32 vector = new Vector4D32(quaternion);
         EuclidCoreTestTools.assertTuple4DEquals(quaternion, vector, EPS);
      }

      { // Test Vector4D32(Tuple4DReadOnly other)
         Tuple4DReadOnly quaternion = EuclidCoreRandomTools.generateRandomQuaternion32(random);
         Vector4D32 vector = new Vector4D32(quaternion);
         EuclidCoreTestTools.assertTuple4DEquals(quaternion, vector, EPS);
      }

      { // Test Vector4D(Vector3DReadOnly vector3D)
         Vector3DReadOnly vector3D = EuclidCoreRandomTools.generateRandomVector3D(random);
         Vector4D32 vector = new Vector4D32(vector3D);
         for (int i = 0; i < 3; i++)
            assertTrue(vector.get32(i) == vector3D.get32(i));
         assertTrue(vector.getS32() == 0.0f);
      }

      { // Test Vector4D(Point3DReadOnly vector3D)
         Point3DReadOnly point3D = EuclidCoreRandomTools.generateRandomPoint3D(random);
         Vector4D32 vector = new Vector4D32(point3D);
         for (int i = 0; i < 3; i++)
            assertTrue(vector.get32(i) == point3D.get32(i));
         assertTrue(vector.getS32() == 1.0f);
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      Vector4D32 vector = createRandomTuple(random);

      int newHashCode, previousHashCode;
      newHashCode = vector.hashCode();
      assertEquals(newHashCode, vector.hashCode());

      previousHashCode = vector.hashCode();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         vector.set(i % 4, random.nextDouble());
         newHashCode = vector.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
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
      return EuclidCoreRandomTools.generateRandomVector4D32(random);
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
