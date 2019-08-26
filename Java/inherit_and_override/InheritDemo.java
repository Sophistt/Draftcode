
// Super Class
class Person {
    
    private String name;
    private int age;
    
    public Person() {
        this.name = "";
        this.age = 0;
    }

    public Person(String name, int age) {
        this.name = name;
        this.age = age;
    }

    public void setName(String name) {
        this.name = name;
    }

    public void setAge(int age) {
        this.age = age;
    }

    public String getName() {
        return this.name;
    }

    public int getAge() {
        return this.age;
    }
}

// Derived Class
class Student extends Person {
    private String school;
   
    // Derived Class calls the default constructor of Super Class automatically.
    public Student() {
        this.school = "";
    }

    public Student(String name, int age, String school) {
        super(name, age);  // Calls the parameters constructor.
        this.school = school;
    }

    public void setSchool(String school) {
        this.school = school;
    }

    public String getSchool() {
        return this.school;
    }
}



public class InheritDemo {
    public static void main(String args []) {
        Student stu = new Student();
        
        stu.setName("Jack");
        stu.setAge(18);
        stu.setSchool("MIT");
        
        System.out.println(String.format("Name: %s, Age: %d, School: %s", stu.getName(), stu.getAge(), stu.getSchool()));
    }
}


