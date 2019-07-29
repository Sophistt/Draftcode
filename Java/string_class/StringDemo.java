
/**
 * String class has its own two memory pools including the static memory pool
 * and the variables memory pool.
 */

public class StringDemo {
    
    public static void main(String args []) {
        String strA = "Example";
        String strB = "Example";
        // strA and strB point to the same heap memory address 
        System.out.println(strA == strB);

        String strC = new String ("Example");
        // strA and strC point to different heap memory address
        System.out.println(strA == strC);

        // The correct method to compare to String.
        System.out.println(strA.equals(strC));
    }
}
