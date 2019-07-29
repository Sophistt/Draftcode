/* *****************************************************************************
 *  Name:
 *  Date:
 *  Description:
 **************************************************************************** */

import java.util.Iterator;

public class Deque<Item> implements Iterable<Item> {

  private class Node {
    Item item;
    Node next;
    Node previous;
  }

  private Node first;
  private Node last;
  private int N;


  public Deque() {
    this.first = null;
    this.last = null;
    this.N = 0;
  }

  public boolean isEmpty() { return first == null; }

  public int size() { return N; }

  public void addFirst(Item item) {
    if (item == null) {
      throw new java.lang.IllegalArgumentException("Can't add null item");
    }

    Node oldfirst = first;
    first = new Node();
    first.item = item;
    first.next = oldfirst;
    if (N > 0)
      oldfirst.previous = first;
    else
      last = first;

    N++;
  }

  public void addLast(Item item) {
    if (item == null) {
      throw new java.lang.IllegalArgumentException("Can't add null item");
    }

    Node oldlast = last;
    last = new Node();
    last.item = item;
    last.previous = oldlast;
    if (N > 0)
      oldlast.next = last;
    else
      first = last;
    N++;
  }

  public Item removeFirst() {
    if (isEmpty()) {
      throw new java.util.NoSuchElementException();
    }
    else {
      Item item = first.item;
      first = first.next;
      if (N != 1) {
        first.previous = null;
      }
      N--;
      return item;
    }
  }

  public Item removeLast() {
    if (isEmpty())
    {
      throw new java.util.NoSuchElementException();
    }
    else {
      Item item = last.item;
      last = last.previous;
      if (N != 1) {
        last.next = null;
      }
      N--;
      return item;
    }
  }

  public Iterator<Item> iterator() {
    return new ListIterator();
  }

  private class ListIterator implements Iterator<Item> {

    private Node current = first;
    @Override
    public boolean hasNext() {
      return current != null;
    }

    public void remove() {
      throw new java.lang.UnsupportedOperationException();
    }

    public Item next() {
      if (hasNext()) {
        Item item = current.item;
        current = current.next;
        return item;
      }

      else {
        throw new java.util.NoSuchElementException();
      }
    }
  }



  public static void main(String[] arg) {
    Deque<Integer> deque = new Deque<Integer>();
    System.out.print(deque.isEmpty());
    deque.addFirst(19);
    System.out.print(deque.removeLast());



  }

}
