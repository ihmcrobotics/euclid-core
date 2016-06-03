package us.ihmc.geometry.interfaces;

public interface EpsilonComparable<T>
{
   public abstract boolean epsilonEquals(T other, double epsilon);
}
