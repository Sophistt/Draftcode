/* *****************************************************************************
 *  Name:
 *  Date:
 *  Description:
 **************************************************************************** */

import edu.princeton.cs.algs4.StdRandom;
import java.util.Iterator;

public class RandomizedQueue<Item> implements Iterable<Item> {

  private int num;
  private Item[] a;

  public RandomizedQueue() {
    num = 0;
    a = (Item[]) new Object[1];
  }

  public boolean isEmpty() { return num == 0; }

  public int size() { return num; }

  public void enqueue(Item item) {
    if (item == null) {
      throw new java.lang.IllegalArgumentException();
    }
    if (num == a.length) resize(num *2);
    a[num++] = item;

  }

  public Item dequeue() {
    if (isEmpty()) {
      throw new java.util.NoSuchElementException();
    }
    int rnd = StdRandom.uniform(num);
    Item item = a[rnd];
    a[rnd] = a[--num];
    a[num] = null;
    if (num > 0 && num == a.length / 4) resize(a.length /2);
    return item;
  }

  public Item sample() {
    if (isEmpty()) {
      throw new java.util.NoSuchElementException();
    }
    int rnd = StdRandom.uniform(num);
    Item item = a[rnd];
    return item;
  }

  private void resize(int max) {
    Item[] temp = (Item[]) new Object[max];
    for (int i = 0; i < num; i++)
      temp[i] = a[i];
    a = temp;
  }

  @Override
  public Iterator<Item> iterator() {
    return new RandomIterator();
  }

  private class  RandomIterator implements Iterator<Item> {
    private Item[] randomList;
    private int index;

    public RandomIterator() {
      index = 0;
      randomList = (Item[]) new Object[num];
      for (int i = 0; i < num; i++)
      {
        randomList[i] = a[i];
      }

      for (int i = 1; i < num; i++)
      {
        int rnd = StdRandom.uniform(i);
        Item tmp = randomList[i];
        randomList[i] = randomList[rnd];
        randomList[rnd] = tmp;
      }
    }

    public boolean hasNext() { return index < num; }

    public Item next() {
      if (!hasNext()) throw new java.util.NoSuchElementException();
      return randomList[index++];
    }

    public void remove() { throw new java.lang.UnsupportedOperationException(); }

  }

  public static void main(String[] args) {

  }
}
