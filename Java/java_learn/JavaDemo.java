
// In general, a java file only owns one class
class Person {
    
    // Fields 
    String name;
    double age;

    // Methods
    public void information() {
        System.out.println("Name: " + name + " Age: " + age);
    }

}

public class JavaDemo {
    public static void main(String args[]) {
        Person per = new Person();
        per.name = "Jack";
        per.age = 50;
        per.information();
    }
}
