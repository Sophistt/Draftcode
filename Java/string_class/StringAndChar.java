/**
 * @Copyright (c) all right reserved
 * 
 * @File    : StringAndChar.java
 * @Date    : 2019-08-06 22:54:15
 * @Author  : Sophistt
 * @Desc    : Java file
 */

public class StringAndChar {
   
    public static void main(String [] args) {
        
        // String.charAt(int index)
        // @return: char at index of String
        String strA = "Hello world!";
        System.out.println(strA.charAt(7));

        // Change low case characters to up case
        String strB = "helloworld";
        char [] temp = strB.toCharArray();
        for(int i = 0; i < temp.length; i++) 
            temp[i] -= 32;
        
        // Change char [] to String
        System.out.println(new String(temp));
        // change char [0:5] to String
        System.out.println(new String(temp, 0, 5));

        String strC = "21654657";
        System.out.println(isNumber(strC));
    }

    // Judge whether all characters in String are number
    public static boolean isNumber(String str) {
        char [] temp = str.toCharArray();
        for (int i = 0; i < temp.length; i++ ) {
            if(temp[i] < '0' || temp[i] > '9')
                return false;
        }
        return true;
    }
}
