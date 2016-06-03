package us.ihmc.geometry.interfaces;

public interface Settable<T>
{
   public abstract boolean containsNaN();

   public abstract void set(T other);

   public abstract void setToNaN();

   public abstract void setToZero();
}
