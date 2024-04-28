
/*----- Includes -----*/
#include "commandParser.h"
#include "commands.h"

#include <stdio.h> // TODO: remove for embedded.

/*----- Main -----*/
int main()
{
    commandParser parser;
    parser.addCommand(example1, "e1", 0);
    parser.addCommand(example2, "e2", 3);

    parser.parseAndExecute("e1\n");
    parser.parseAndExecute("e2\n"); // If not enough args, they just become 0... Can be fixed, LATER....
    parser.parseAndExecute("e2 10 11 12\n");
    parser.parseAndExecute("e2");
}

