// All vaiables and methods must be defined in class including main().
//
// Main class use public as a symbol.
public class Hello {
	// Name of main function is unchangable.
	public static void main(String args[]) {
		System.out.println("Hello World!");

		int ch = 2;

		switch (ch) {
			case 2 : {
					 System.out.println("ch = 2");
					 break;
			}
			
			default: {
					 System.out.println("ch != 2");
			}
		}

		printMessage();
		System.out.println(sum(100));
	}

	public static void printMessage() {
		System.out.println("Hello message");
	}
	
	/**
	 * Recursive method
	 * 1. 初始条件 if （num == 1
	 * 2. 函数内调用自身 return num + sum()
	 * 3. 修改输入参数  sum(num - 1)
	 *
	 */
	public static int sum(int num) {
		if(num == 1) {
			return 1;
		}
		return num + sum(num - 1);
	}
}
