

class SubItem {
    private int sid;
    private String title;
    private Item item;

    public SubItem(int sid, String title) {
        this.sid = sid;
        this.title = title;
    }
    
    public void setItem(Item item) {
        this.item = item;
    }

    public Item getItem() {
        return this.item;
    }

    public String getInfo() {
        return "ID: " + this.sid + ". Title: " + this.title;
    }
}

class Item {
    private int iid;
    private String title;
    private SubItem subItem [];

    public Item(int iid, String title) {
        this.iid = iid;
        this.title = title;
    }

    public void setSubItem(SubItem subItem []) {
        this.subItem = subItem;
    }

    public SubItem [] getSubItem() {
        return this.subItem;
    }

    public String getInfo() {
        return "ID: " + this.iid + ". Title: " + this.title;
    }
}

public class OneToMulti {
    public static void main(String args []) {
        // Step 1
        Item item = new Item(10, "Book");
        SubItem subItem_1 = new SubItem(101, "Programing book");
        SubItem subItem_2 = new SubItem(102, "Comic book");

        item.setSubItem(new SubItem [] {subItem_1, subItem_2});
        subItem_1.setItem(item);
        // Step 2
        System.out.println(item.getInfo());
        for(int i = 0; i < item.getSubItem().length; i++) {
            System.out.println("\t|- " + item.getSubItem()[i].getInfo());
        }
    }
}
