
class ArrayUtil {
    // Define array utilize method by oneself 
    public static void sort(int data []) {
        for (int j = 0; j < data.length; j++) {
            for (int i = 0; i < data.length - j - 1; i++) {
                if (data[i] > data[i + 1]) {
                    int temp = data[i];
                    data[i] = data[i + 1];
                    data[i + 1] = temp;
                }
            }
        } 
    }

    public static void printArray(int data []) {
        for (int temp : data) {
            System.out.print(temp + "、");
        }
    }
}

public class ArrayMethod {
    public static void main(String args []) {
       int data [] = new int [] {5, 0, 6, 3, 1, 7, 9, 10};

       // Use my method
       ArrayUtil.sort(data);

       // Use java standard lib method
       java.util.Arrays.sort(data);

       ArrayUtil.printArray(data);

       // --------------------------------------------------
       
       int dataA [] = new int [] {1,2,3,4,5,6,7,8,9};
       int dataB [] = new int [] {11,22,33,44,55,66,77,88,99};
        
       // copy 5 elements of dataA to dataB beginning from index 3
       System.arraycopy(dataA, 5, dataB, 3, 3);

       ArrayUtil.printArray(dataB);
    }
}
