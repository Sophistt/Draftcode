
class Channel {
    
    private String id;

    @Deprecated  // Expired API, warnings will be showed while compiling
    public String connection() {
        return "Connect to somewhere";
    }

    public void connect() {
        System.out.println("Connect to somewhere.");
    }
}

class DatabaseChannel extends Channel {
    
    // We can override connect method in derived class. 
    @Override  // Explicit define an override function
    public void connect() {
        System.out.println("Connect to database.");
    }
}

// Use keyword final to avoid being inherited.
final class Publisher {
    
    // Use keyword final to define a const variable
    public final static int NODE_ID = 1;
    
    public void publish() {
        System.out.println("NODE_ID: " + NODE_ID);
    }
}

public class OverrideDemo {
    public static void main(String args []) {
        DatabaseChannel channel = new DatabaseChannel();
        Publisher publisher = new Publisher();
        channel.connect();
        publisher.publish();
        @SuppressWarnings({"deprecation"})  // Suppress warnings while compiling
        channel.connection();
    }
}
