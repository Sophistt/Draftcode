#include <iostream>
#include <cstddef>
#include <algorithm>

using namespace std;

int find(int const array[] , int const dimension, int &min, int &max)
{
    max = *max_element(array, array + dimension);
    min = *min_element(array, array + dimension);

    return 0;
}

int main(int argc, char const *argv[])
{
    int const array[] = {5, -6, 21, 15, -8};
    size_t const dimension = sizeof(array) / sizeof(*array);

    int min, max;
    find(array, dimension, min, max);
    cout << "min= " << min << endl;
    cout << "max= " << max << endl;

    return 0;
}