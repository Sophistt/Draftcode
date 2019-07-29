

public class ArrayDemo {
    public static void main(String [] args) {
        int data [] = new int [] {1, 23, 56};

        for (int i = 0; i < data.length; i++) {
            System.out.println(data[i]);
        }

        // foreach == for(int i=0; i<data.length; i++)
        for (int temp : data) {
            System.out.println(temp);
        }
    }
}