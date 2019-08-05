
/**
 * String class has its own two memory pools including the static memory pool
 * and the variables memory pool.
 */

public class StringDemo {
    
    public static void main(String args []) {
        System.out.println("Compare two Strings");

        String strA = "Example";
        String strB = "Example";
        // The second way to create a String class
        String strC = new String ("Example");
        
        // strA and strB point to the same heap memory address (String Pool)
        // @return: true
        System.out.println(strA == strB);

        // strA and strC point to different heap memory address (String Pool and normal heap memroy respectively)
        // @return: false
        System.out.println(strA == strC);

        // The correct method to compare to String.
        // @return: true
        System.out.println(strA.equals(strC));

        // Each const String is an anonymous object.
        // @return: true
        System.out.println("Example".equals(strA));

        //--------------------------------------------------------------------------------
        // The differences between two ways to create a String class:
        // Direct assignment and Constructor instantiation
        System.out.println();
        System.out.println("Differences between two ways to create a String class");
        // Search String Pool, if "Example" exists, create an object named strA in stack memory pointing to "Example"
        // if not, create "Example" in String Pool first, and then create strA in statck memory Pointing to it.
        String strD = "Instance";
        System.out.println("HashCode of strD: " + System.identityHashCode(strD));

        // Point to the same address in String Pool as strA
        String strE = "Instance";
        System.out.println("HashCode of strE: " + System.identityHashCode(strE));
        
        // Create a new object in normal heap memory (not String Pool) and then create stcC in stack memory pointing to it.
        String strF = new String("Instance");
        System.out.println("HashCode of strF: " + System.identityHashCode(strF));
        //--------------------------------------------------------------------------------

        //--------------------------------------------------------------------------------
        // Static String Pool and Running String Pool
        // The purpose of String Pool design is object sharing. However, String Pool is divided into Static String Pool and 
        // Running String Pool.
        System.out.println();
        System.out.println("Static String Pool and Running String Pool");

        // Static String Pool: All Strings in it will be created and dealed before the program running
        String strG = "Hello world";
        String strH = "Hello " + "world";
        System.out.println("Compare strG and strH(static String Pool): " + (strG == strH));

        // Running String Pool: Strings in it will be created during the running process of the program
        String info = "world";
        String strI = "Hello " + info;
        System.out.println("Compare strG and strI(static String Pool and runing String Pool): " + (strG == strI));
        //--------------------------------------------------------------------------------
    }
}
