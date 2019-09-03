/**
 * @Copyright (c) all right reserved
 * 
 * @File    : ArrayDemo.java
 * @Date    : 2019-09-03 22:01:15
 * @Author  : Sophistt
 * @Desc    : Java file
 */

class Array {
    /*
    * Super class
    * Create an array in specified length.
    * @method: addData, incrementArray, getData
    */
    private int [] data;
    private int foot;

    public Array(int len) {
        if (len > 0) this.data = new int [len];
        else this.data = new int [1];
    }

    public boolean addData(int num) {
        if (this.foot < this.data.length) {
            this.data[this.foot ++] = num;
            return true;
        }
        else {
            return false;
        }
    }

    public void incrementArray(int num) {
        int [] newData = new int [this.data.length + num];
        System.arraycopy(this.data, 0, newData, 0, this.data.length);
        this.data = newData;
    }

    public int [] getData() {
        return this.data;
    }

}

class SortArray extends Array {
    // Derived class of Array
    public SortArray(int len) {
        super(len);
    }

    public int [] getData() {  // override getData() in super class
        java.util.Arrays.sort(super.getData());
        return super.getData();
    }
}

class ReverseArray extends Array {
    // Derived class of Array
    public ReverseArray(int len) {
        super(len);
    }

    public int [] getData() {
        int center = super.getData().length / 2;
        int head = 0;
        int tail = super.getData().length - 1;

        for (int i = 0; i < center ; i++) {
            int temp = super.getData()[head];
            super.getData()[head] = super.getData()[tail];
            super.getData()[tail] = temp;
            head++;
            tail--;
        }

        return super.getData();
    }
}

public class ArrayDemo {
    public static void main(String [] args) {
        ReverseArray newArray = new ReverseArray(5);
        System.out.println(newArray.addData(1));
        System.out.println(newArray.addData(10));
        System.out.println(newArray.addData(5));
        System.out.println(newArray.addData(20));
        System.out.println(newArray.addData(8));
        newArray.incrementArray(3);
        System.out.println(newArray.addData(6));
        System.out.println(newArray.addData(15));
        System.out.println(newArray.addData(17));
        
        int result [] = newArray.getData();

        for (int i = 0; i < result.length; i++ ) {
            System.out.println(result[i]);
        }
    }
}
