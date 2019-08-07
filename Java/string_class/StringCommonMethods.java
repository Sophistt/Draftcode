/**
 * @Copyright (c) all right reserved
 * 
 * @File    : StringCommonMethods.java
 * @Date    : 2019-08-07 22:55:52
 * @Author  : Sophistt
 * @Desc    : Java file
 */

public class StringCommonMethods {
    public static void main(String [] args) {

        // ------------------------------ Search --------------------------------------
        String  strA = "Hello world";
        // public boolen contains(String s)
        System.out.println(strA.contains("wang"));
        // public int indexOf(String str(, int fromIndex))
        System.out.println(strA.indexOf("world"));
        System.out.println(strA.indexOf("wang"));
        // public int lastIndexOf(String str(, int fromIndex))
        System.out.println(strA.lastIndexOf("world"));
        // public boolen startsWith(String str(, int index))
        System.out.println(strA.startsWith("hello"));
        // public boolen endsWith(String str(, int index))
        System.out.println(strA.endsWith("ld"));

        // ----------------------------- Substitute -----------------------------------
        // public String replaceAll(String regex, String replacement)
        System.out.println(strA.replaceAll("l", "x"));
        // public String replaceAll(String regex, String replacement)
        System.out.println(strA.replaceFirst("l", "x"));

        // ----------------------------- Split -----------------------------------
        // public String [] split
        String strB = "192.168.1.1";
        String result [] = strB.split("\\.");
        for (int i = 0; i < result.length; i++ ) {
            System.out.println(result[i]);
        }

        // ----------------------------- SubString -----------------------------------
        // pulic String substring(int beginIndex(, int endIndex))
        String strC = "Sophistt-image-kent.jpg";
        int beginIndex = strC.indexOf("-", strC.indexOf("image")) + 1; 
        int endIndex = strC.indexOf(".");
        System.out.println(strC.substring(beginIndex, endIndex));

        // ----------------------------- String format -----------------------------------
        String name = "kent";
        int age = 18;
        double score = 98.765432;
        System.out.println(String.format("Name: %s, Age: %d, Score: %5.2f", name, age, score));

    }
}
