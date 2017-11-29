package us.ihmc.euclid.exceptions;

public class InvalidHashException extends RuntimeException
{
   /**
    * Constructs an {@code InvalidHashException} with no detail message.
    */
   public InvalidHashException()
   {
      super();
   }

   /**
    * Constructs an {@code InvalidHashException} with the specified detail message.
    *
    * @param message the detail message.
    */
   public InvalidHashException(String message)
   {
      super(message);
   }
}
