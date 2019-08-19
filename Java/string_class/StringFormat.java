
public class StringFormat {
    public static void main(String args []) {
        String name = "Jack";
        int age = 90;
        double score = 98.456483;

        // Explanation: %5.2f means 5 digits with 2 decimal places
        System.out.println(String.format("Name: %s, Age: %d, Score: %5.2f", name, age, score));
    }
}
