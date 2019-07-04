

/**
 * Simple Class Emp
 * 
 * Note:
 * Class name is meaningful.
 * All class members are private.
 * Class has its own non-parameters construct method.
 * Class has getInfo() method.
 */
class Emp {
    
    // Class member
    private String name;
    private String dept;
    private int age;
    private int salary;

    // Construct method
    public Emp() {
        this(null, null, 0, 0);
    }
    public Emp(String name) {
        this(name, null, 0, 0);
    }

    public Emp(String name, String dept, int age, int salary) {
        this.name = name;
        this.dept = dept;
        this.age = age;
        this.salary = salary;
    }
    
    // @Setter
    public void setName(String name) {
        this.name = name;
    }

    public void setDept(String dept) {
        this.dept = dept;
    }
    
    public void setAge(int age) {
        this.age = age;
    }
    
    public void setSalary(int salary) {
        this.salary = salary;
    }
    
    // @Getter
    public String getName() {
        return this.name;
    }

    public String getDept() {
        return this.dept;
    }

    public int getAge() {
        return age;
    }

    public int getSalary() {
        return salary;
    }

    public String getInfo() {
        return "Name:" + this.name +
            " Dept:" + this.dept +
            " Age:" + this.age + 
            " Salary:" + this.salary;
    }
}

public class JavaDemo {
    public static void main(String args[]) {
        Emp emp = new Emp("Smith");
        System.out.println(emp.getInfo());
    }
}
