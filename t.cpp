#include <iostream>
#include <math.h>
using namespace std;

#define F32_2_PI 6.28318530717958f
#define F32_PI 3.14159265358979f
#define F32_PI_2 1.57079632679489f

float degreeChange(float curr, float prev)
{
    float change;

    if ((prev > 90) && (curr < -90))
    {
        change = (180 - prev) + (180 + curr);
    }
    else if ((prev < -90) && (curr > 90))
    {
        change = -(180 + prev) - (180 - curr);
    }
    else
    {
        change = curr - prev;
    }

    return change;
}

int main()
{
    float curr, prev;
    cout << "Enter cuur and prev: ";
    cin >> curr >> prev;
    cout << angleChange(curr, prev) << endl;
    return 0;
}