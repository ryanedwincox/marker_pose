#include "combinations.h"

Combinations::Combinations()
{
//    r = 4;
//    maxNumCombinations = 210;
    w = 0;

}




/* arr[]  ---> Input Array
   n      ---> Size of input array
   r      ---> Size of a combination to be printed
   index  ---> Current index in data[]
   data[] ---> Temporary array to store current combination
   i      ---> index of current element in arr[]     */
void Combinations::combinationUtil(int arr[], int n, int index, int data[], int i)
{
    // Current cobination is ready, print it
    if (index == r)
    {
        for (int j=0; j<r; j++)
        {
            combinations[w][j] = data[j];
            // printf("***%d ",data[j]);
        }
        // printf("\n");
        w++;
        return;
    }

    // When no more elements are there to put in data[]
    if (i >= n)
        return;

    // current is included, put next at next location
    data[index] = arr[i];
    combinationUtil(arr, n, index+1, data, i+1);

    // current is excluded, replace it with next (Note that
    // i+1 is passed, but index is not changed)
    combinationUtil(arr, n, index, data, i+1);
}

// The main function that prints all combinations of size r
// in arr[] of size n. This function mainly uses combinationUtil()
void Combinations::printCombination(int arr[], int n)
{
    // A temporary array to store all combination one by one
    int data[r];
    numCombinations = factorial(n)/(factorial(r)*(factorial(n-r)));

    // Print all combination using temprary array 'data[]'
//    combinationUtil(arr, n, 0, data, 0);
}

unsigned int Combinations::factorial(unsigned int n)
{
    if (n == 0)
    {
       return 1;
    }
    return n * factorial(n - 1);
}

//// Driver program to test above functions
//int main()
//{
//    int arr[] = {0, 1, 2, 3, 4, 5};
//    // const int r = 4;
//    int n = sizeof(arr)/sizeof(arr[0]);
//    const int numCombinations = factorial(n)/(factorial(r)*(factorial(n-r)));
//    // int (*combinations)[r] = new int[numCombinations][r];
//    std::cout << numCombinations << std::endl;
//    printCombination(arr, n);

//    for (int i=0; i<numCombinations; i++)
//    {
//        for (int j=0; j<r; j++)
//        {
//            printf("%d ",combinations[i][j]);
//        }
//        printf("\n");
//    }

//    return 0;
//}
