/**
 * @Copyright (c) all right reserved
 * 
 * @File    : StringCompareMethod.java
 * @Date    : 2019-08-06 23:32:45
 * @Author  : Sophistt
 * @Desc    : Java file
 */

public class StringCompareMethod {
    public static void main(String [] args) {
        String strA = "Hello";
        String strB = "hello";
        
        // public boolen equals
        System.out.println(strA.equals(strB));
        // public int equalsIgnoreCase
        System.out.println(strA.equalsIgnoreCase(strB));
        // public int compareTo
        System.out.println(strA.compareTo(strB));
        // public int compareToIgnoreCase
        System.out.println(strA.compareToIgnoreCase(strB));
    }
}
