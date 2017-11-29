package us.ihmc.euclid.tools;

import us.ihmc.euclid.exceptions.InvalidHashException;

public class EuclidHashCodeTools
{
   /**
    * Default long value that {@link bits} is initialized to when a
    * new hash is begun.
    */
   private static long DEFAULT_INIT = 1L;

   /**
    * Long used for the multiplication factor in each step
    * of the hash.
    */
   private static long MULTIPLIER = 31L;

   /**
    * Current internal state of hash.
    */
   private static long bits = 0L;

   /**
    * Whether or not a hash is in progress.
    */
   private static boolean hashing = false;

   /**
    * Checks that a hash is in progress. Used to enforce proper usage.
    * 
    * @throws InvalidHashException if a hash has not been started
    */
   private static void checkHashStarted() throws InvalidHashException
   {
      if (!hashing)
         throw new InvalidHashException("Cannot add to or end a hash that has not started.");
   }
   
   /**
    * Begins a hash.
    * 
    * @throws InvalidHashException if a hash is already in progress
    */
   public static void beginHash() throws InvalidHashException
   {
      beginHash(DEFAULT_INIT);
   }

   /**
    * Begins a hash.
    *
    * @throws InvalidHashException if a hash is already in progress
    */
   public static void beginHash(long init) throws InvalidHashException
   {
      if (hashing)
         throw new InvalidHashException("Cannot begin a hash while another hash is in progress.");
      
      hashing = true;
      
      bits = init;
   }
   
   /**
    * Ends and returns a hash.
    *
    * @return final integer hash value.
    * @throws InvalidHashException if a hash has not been started
    */
   public static int endHash() throws InvalidHashException
   {
      checkHashStarted();

      hashing = false;

      return (int) (bits ^ bits >> 32);
   }

   /**
    * Add the next double value to the hash.
    * 
    * @param value double to add to hash.
    * @throws InvalidHashException if a hash has not been started
    */
   public static void hash(double value) throws InvalidHashException
   {
      checkHashStarted();
      
      bits = MULTIPLIER * bits + Double.doubleToLongBits(value);
   }

   /**
    * Add the next float value to the hash.
    *
    * @param value float to add to hash.
    * @throws InvalidHashException if a hash has not been started
    */
   public static void hash(float value) throws InvalidHashException
   {
      checkHashStarted();
      
      bits = MULTIPLIER * bits + Float.floatToIntBits(value);
   }
}
