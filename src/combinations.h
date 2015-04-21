#ifndef COMBINATIONS_H
#define COMBINATIONS_H

class Combinations
{

public:
    Combinations();
    void combinationUtil(int arr[], int n, int index, int data[], int i);
    void printCombination(int arr[], int n);
    unsigned int factorial(unsigned int n);

    static const int r = 4;
    static const int maxNumCombinations = 210; // 10c4

    int (*combinations)[r];
    int numCombinations;

private:
    int w;
};

#endif // COMBINATIONS_H
