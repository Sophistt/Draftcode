#include <iostream>

using namespace std;

int swapByPointer(int *first_ptr, int *second_ptr)
{
    int temp;
    temp = *first_ptr;
    *first_ptr = *second_ptr;
    *second_ptr = temp;

    return 0;
}
int swapByReference(int &first_ref, int &second_ref)
{
    int temp;
    temp = first_ref;
    first_ref = second_ref;
    second_ref = temp;

    return 0;
}

int main(int argc, char const *argv[])
{
    int first = -5, second = 4;
    swapByPointer(&first, &second);
    cout << "First = " << first << " Second = " << second << endl;
    swapByReference(first, second);
    cout << "First = " << first << " Second = " << second << endl;

    return 0;
}