/**
 * @Copyright (c) all right reserved
 * 
 * @File    : PolyDemo.java
 * @Date    : 2019-09-05 22:33:50
 * @Author  : Sophistt
 * @Desc    : Java file
 */

class Message {
    public String print() {
        return "Message";
    }
}

class DataMessage extends Message {
    public String print() {
        return "DataMessage";
    }
}

class WebMessage extends Message {
    public String print() {
        return "WebMessage";
    }

    // Method Polymorphism 
    // Advantage: Use the same function's name to execute different methods
    public String print(int num) {
        return "WebMessage" + num;
    }
}

public class PolyDemo {
    public static void main(String [] args) {

        msgPrint(new DataMessage());
        msgPrint(new WebMessage());
    }

    // Class Polymorphism -- upward transformation
    // Advantage: Use super class to unify input 
    public static void msgPrint(Message msg) {
        System.out.println(msg.print());
    }

}
