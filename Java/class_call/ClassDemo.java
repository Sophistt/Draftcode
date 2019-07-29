

class Car {

    private String name;
    private double prize;
    private Person owner;
    
    public Car() {
        this(null, 0);
    };

    public Car(String name, double prize) {
        setName(name);
        setPrize(prize);
    };
    
    public void setName(String name) {
        this.name = name;
    };
    
    public void setPrize(double prize) {
        this.prize = prize;
    };

    public void setOwner(Person person) {
        this.owner = person;
    }

    public String getName() {
        return this.name;
    };
    
    public double getPrize() {
        return this.prize;
    };

    public Person getOwner() {
        return this.owner;
    }

    public void getInfo() {
        System.out.println("Name: " + this.name + "、Prize: " + this.prize);
    };
}


class Person {
    
    private String name;
    private int age;
    private Car car;
    private Person children [];
    
    public Person() {
        this(null, 0);
    }

    public Person(String name, int age) {
        setName(name);
        setAge(age);
    }

    public void setName(String name) {
        this.name = name;
    }

    public void setAge(int age) {
        this.age = age;
    }

    public void setCar(Car car) {
        this.car = car;
    }

    public void setChildren(Person children []) {
        this.children = children;
    }

    public String getName() {
        return this.name;
    }

    public int getAge() {
        return this.age;
    }

    public Car getCar() {
        return this.car;
    }

    public Person [] getChildren() {
        return this.children;
    }

    public void getInfo() {
        System.out.println("Name: " + this.name + "、Age: " + this.age);
    }
}

public class ClassDemo {
    public static void main(String args []) {
        Car car_1 = new Car ("Benz", 350000.00);
        car_1.getInfo();
        Person per_1 = new Person ("jack", 20);
        per_1.setCar(car_1);
        per_1.getInfo();

        Person childA = new Person("Tom", 5);
        Person childB = new Person("Li", 4);

        per_1.setChildren(new Person [] {childA, childB});
        
        for(int i = 0; i < per_1.getChildren().length; i++) {
            per_1.getChildren()[i].getInfo();
        }
    }
}
