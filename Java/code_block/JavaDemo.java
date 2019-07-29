
class Message {
    public static String getMessage() {
        return "China";
    }
}


class Person {

    // Class members
    private String name;
    private int age;
    private static String country;

    // Static code block for initialization of static members
    // Only called once
    static {
        country = Message.getMessage();
    }
    
    // Construction methods
    public Person() {
        this("None", 0);
    }

    public Person(String name, int age) {
        this.setName(name);
        this.setAge(age);
    }
    
    // Setter
    public void setName(String name) {
        this.name = name;
    }

    public void setAge(int age) {
        this.age = age;
    }

    // Getter
    public String getName() {
        return this.name;
    }

    public int getAge() {
        return this.age;
    }

    public String getInfo() {
        return "Name:" + this.name +
            " Age:" + this.age +
            " Country:" + country;
    }
    
    // Static methods
    public static void setCountry(String c) {
        country = c;
    }

    public static String getCountry() {
        return country;
    }
}



public class JavaDemo {

    // Code here will be called before main function.
    static {
        System.out.println("******** Process initialization *******");
    }

    public static void main(String args[]) {
        System.out.println(Person.getCountry());

        Person.setCountry("America");

        Person per = new Person("Smith", 10);

        System.out.println(per.getInfo());
    }
}
