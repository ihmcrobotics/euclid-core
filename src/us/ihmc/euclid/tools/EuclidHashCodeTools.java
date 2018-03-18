package us.ihmc.euclid.tools;

public class EuclidHashCodeTools
{
   /**
    * Long used for the multiplication factor in each step of the hash.
    */
   private final static long MULTIPLIER = 31L;

   /**
    * Returns a hash bit stream as an integer hash value.
    *
    * @param bits the bits to turn into int hash code.
    * @return final integer hash value.
    */
   public static int toIntHashCode(long bits)
   {
      return (int) (bits ^ bits >> 32);
   }

   /**
    * Calls {@link #combineHashCode(long, long)} after converting {@code value} to long bits.
    *
    * @param hashCode long hash code bit stream to add to.
    * @param value double value to add to the hash code.
    * @return new hash code bit stream
    */
   public static long addToHashCode(long hashCode, double value)
   {
      return combineHashCode(hashCode, Double.doubleToLongBits(value));
   }

   /**
    * Calls {@link #combineHashCode(long, long)} after converting {@code value} to int bits.
    *
    * @param hashCode long hash code bit stream to add to.
    * @param value float value to add to the hash code.
    * @return new hash code bit stream
    */
   public static long addToHashCode(long hashCode, float value)
   {
      return combineHashCode(hashCode, Float.floatToIntBits(value));
   }

   /**
    * Combine the two hash code bit streams.
    *
    * @param hashCode1 first hash code to combine.
    * @param hashCode2 second hash code to combine.
    * @return combined hash code, {@code hashCode1} + {@link MULTIPLIER} * {@code hashCode2}
    */
   public static long combineHashCode(long hashCode1, long hashCode2)
   {
      return hashCode1 + MULTIPLIER * hashCode2;
   }
}
