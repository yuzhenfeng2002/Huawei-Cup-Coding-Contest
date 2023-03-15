#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <string>
// #include "CMap.h"
using namespace std;

bool readUntilOK()
{
    char line[1024];
    while (fgets(line, sizeof line, stdin))
    {
        if (line[0] == 'O' && line[1] == 'K')
        {
            return true;
        }
        // do something
    }
    return false;
}

int main()
{
    readUntilOK();
    puts("OK");
    fflush(stdout);
    int frameID;
    int money;
    int handle_num;
    string line;
    while (scanf("%d", &frameID) != EOF)
    {
        readUntilOK();
        int lineSpeed = 3;
        double angleSpeed = 1.5;
        for (int robotId = 0; robotId < 4; robotId++)
        {
            printf("forward %d %d\n", robotId, lineSpeed);
            printf("rotate %d %f\n", robotId, angleSpeed);
        }
        printf("OK\n", frameID);
        fflush(stdout);
    }
    return 0;
    return 0;
}
