

class StringUtil {
    private String str;

    StringUtil(String str) {
        this.str = str;
    }

    public String getStr() {
        return this.str;
    }
    
    // Simple method facing process.
    public static int [] countChar(String str) {
        char data [] = str.toCharArray();
        
        int count [] = new int [2];
        
        for (int i = 0; i < data.length; i++) {
            if (data[i] == 'n' || data[i] == 'N')
                count[0]++;
            if (data[i] == 'm' || data[i] == 'M')
                count[1]++;
        }
        return count;
    }
}

// Use inheritance to achieve OOP.
class StringCount extends StringUtil {
    
    private int count [];
    
    StringCount(String str) {
        super(str);
    }

    public int [] countChar() {
        this.count = new int [2];
        
        char data [] = super.getStr().toCharArray();

        for (int i = 0; i < data.length; i++) {
            if (data[i] == 'n' || data[i] == 'N')
                this.count[0]++;
            if (data[i] == 'm' || data[i] == 'M')
                this.count[1]++;
        }
        return count;
    }
}

public class InheritPratice {
    public static void main(String args []) {
        // String for experiment.
        String str = "Stars change but my mind remains.";
        
        int result [] = StringUtil.countChar(str);
        System.out.println("Number of n: " + result[0] + ", Number of m: " + result[1]);

        StringCount strCount = new StringCount(str);
        int cResult [] = strCount.countChar();
        System.out.println("Number of n: " + cResult[0] + ", Number of m: " + cResult[1]);

    }
}
