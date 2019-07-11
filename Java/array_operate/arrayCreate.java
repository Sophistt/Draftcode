
public class arrayCreate {
    public static void main(String args []) {
        // dynamic array
        int array [] = new int [5]; 

        System.out.println("length of array: " + array.length);
        
        // static array 
        int sta_array [] = new int [] {0, 0};  // static array means give value of array when initialize it.
        
        System.out.println("length of array: " + sta_array.length);
        
        // 2D array
        int data [][] = new int [][] {{1,2,3,4,5}, {1,2,3}, {5,7,8,9}};
        
        // Use for loop to call 2dArray
        for (int i = 0; i < data.length; i++) {
            for (int j = 0; j < data[i].length; j++) {
                System.out.print(data[i][j] + "、");
            }
            System.out.println();
        }
        
        // Use foreach to call 2dArray
        for (int temp [] : data) {
            for (int num : temp) {
                System.out.print(num + "、");
            }
            System.out.println();
        }
    }
}
